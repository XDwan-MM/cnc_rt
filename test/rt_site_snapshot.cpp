#include <QCoreApplication>
#include <QSharedMemory>
#include <QFile>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QCryptographicHash>
#include <QDateTime>
#include <QTemporaryDir>
#include <QUuid>
#include <QProcess>
#include <QProcessEnvironment>
#include <cstdio>
#include <cstring>
#include <memory>
#include "ncdata.h"

static bool writeFile(const QString &path, const QByteArray &bytes)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::NewOnly) &&
           file.write(bytes) == bytes.size() && file.flush();
}

static QJsonObject reg(const LadReg &r)
{ return {{"type", int(r.type)}, {"idx", r.idx}, {"bit", int(r.bit)}}; }

static bool capture(const QString &key, const QString &output)
{
    // No shmInit(): it can create/reset memory and register a process.
    QSharedMemory memory(key);
    if (!memory.attach(QSharedMemory::ReadOnly)) {
        fprintf(stderr, "Read-only attach failed: %s\n", qPrintable(memory.errorString()));
        return false;
    }
    const int size = memory.size();
    if (size <= 0 || size > 256 * 1024 * 1024 || !QDir().mkdir(output)) {
        fprintf(stderr, "Invalid segment size or output directory already exists\n");
        return false;
    }
    // No lock (do not delay RT writers); not an atomic scan.
    const QByteArray bytes(static_cast<const char *>(memory.constData()), size);
    memory.detach();
    if (!writeFile(output + "/shared-memory.bin", bytes)) return false;
    QJsonObject result{
        {"key", key}, {"capturedUtc", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
        {"bytes", size}, {"expectedBytes", int(sizeof(NcData))},
        {"sha256", QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex())},
        {"atomic", false}, {"layoutVersionExpected", CNC_SHM_LAYOUT_VERSION}
    };
    auto snapshot = std::make_unique<NcData>();
    bool compatible = size == sizeof(NcData);
    if (compatible) {
        memcpy(snapshot.get(), bytes.constData(), sizeof(NcData));
        const auto &h = snapshot->header;
        compatible = h.magic == CNC_SHM_MAGIC && h.headerSize == sizeof(ShmHeader) &&
                     h.dataSize == sizeof(NcData) && h.layoutVersion == CNC_SHM_LAYOUT_VERSION;
    }
    result["layoutCompatible"] = compatible;
    if (compatible) {
        const auto &l = snapshot->ladData;
        const bool bounds = l.info.instrCount >= 0 && l.info.instrCount <= LAD_IL_MAX_NUM &&
            l.info.regCount >= 0 && l.info.regCount <= LAD_REG_MAX_NUM &&
            l.info.ioCount >= 0 && l.info.ioCount <= LAD_IO_MAX_NUM &&
            l.info.regLockCount >= 0 && l.info.regLockCount <= LAD_REG_LOCK_MAX_NUM;
        result["ladderCountsValid"] = bounds;
        if (bounds) {
            QJsonArray instructions, regs, mappings, locks;
            for (int i = 0; i < l.info.instrCount; ++i)
                instructions.append(QJsonObject{{"opcode", int(l.instrs[i].opcode)},
                    {"regIdx", l.instrs[i].regIdx}, {"state", int(l.instrs[i].state)}});
            for (int i = 0; i < l.info.regCount; ++i) regs.append(reg(l.regs[i]));
            for (int i = 0; i < l.info.ioCount; ++i) {
                const auto &m = l.ioMappings[i];
                mappings.append(QJsonObject{{"src", reg(m.src)}, {"dir", reg(m.dir)},
                    {"flag", int(m.flag)}, {"type", int(m.type)}});
            }
            for (int i = 0; i < l.info.regLockCount; ++i) {
                const auto &r = l.regLocks[i];
                locks.append(QJsonObject{{"reg", reg(r.reg)}, {"lockflag", r.lockflag},
                    {"newflag", r.newflag}, {"newval", r.newval}});
            }
            result["ladder"] = QJsonObject{{"version", l.info.version},
                {"isRunning", l.info.isRunning}, {"instrs", instructions}, {"regs", regs},
                {"ioMappings", mappings}, {"regLocks", locks}};
        }
        const RegData &r = snapshot->regData;
        QJsonObject banks;
#define BANK(name) banks[#name] = QString::fromLatin1(QByteArray(r.name, sizeof(r.name)).toHex())
        BANK(regA); BANK(regR); BANK(regX); BANK(regY); BANK(regF); BANK(regG);
        BANK(regI); BANK(regQ); BANK(regK);
#undef BANK
        result["registerBytesHex"] = banks;
        // Full parameters, alarms and variables are retained in the raw image.
    }
    if (!writeFile(output + "/snapshot.json", QJsonDocument(result).toJson())) return false;
    printf("Captured read-only: %s (layout %s; non-atomic snapshot)\n",
           qPrintable(output), compatible ? "compatible" : "UNKNOWN, raw image retained");
    return true;
}

static int selfTest(const QString &script)
{
    const QString key = "cnc-site-test-" + QUuid::createUuid().toString();
    QTemporaryDir dir;
    QSharedMemory owner(key);
    if (!dir.isValid() || !owner.create(sizeof(NcData))) return 1;
    auto *data = static_cast<NcData *>(owner.data());
    memset(data, 0, sizeof(*data));
    data->header.magic = CNC_SHM_MAGIC;
    data->header.headerSize = sizeof(ShmHeader);
    data->header.dataSize = sizeof(NcData);
    data->header.layoutVersion = CNC_SHM_LAYOUT_VERSION;
    data->ladData.info.instrCount = data->ladData.info.regCount = 1;
    data->ladData.instrs[0] = {OP_AND, 0, LAD_FORCE_ENABLE};
    data->ladData.regs[0] = {REG_R, 1000, 3};
    data->regData.regF[0] = 0x40;
    const QByteArray before(static_cast<const char *>(owner.constData()), sizeof(NcData));
    if (!capture(key, dir.path() + "/ok")) return 1;
    if (memcmp(before.constData(), owner.constData(), sizeof(NcData))) return 1;
    QFile json(dir.path() + "/ok/snapshot.json");
    if (!json.open(QIODevice::ReadOnly)) return 1;
    const auto result = QJsonDocument::fromJson(json.readAll()).object();
    if (!result["layoutCompatible"].toBool() ||
        result["ladder"].toObject()["instrs"].toArray()[0].toObject()["state"].toInt() != LAD_FORCE_ENABLE)
        return 1;
    data->header.layoutVersion++;
    if (!capture(key, dir.path() + "/incompatible")) return 1;
    QFile incompatible(dir.path() + "/incompatible/snapshot.json");
    if (!incompatible.open(QIODevice::ReadOnly)) return 1;
    if (QJsonDocument::fromJson(incompatible.readAll()).object()["layoutCompatible"].toBool()) return 1;
    const QString missing = key + "-missing";
    if (capture(missing, dir.path() + "/absent")) return 1;
    QSharedMemory check(missing);
    if (check.attach(QSharedMemory::ReadOnly)) return 1;
    if (!script.isEmpty()) {
        data->header.layoutVersion = CNC_SHM_LAYOUT_VERSION;
        const QByteArray beforeScript(static_cast<const char *>(owner.constData()), sizeof(NcData));
        const QString firmware = dir.path() + "/firmware";
        if (!QDir().mkdir(firmware)) return 1;
        for (const auto &part : {"plc", "conf", "data"})
            if (!QDir().mkdir(firmware + "/" + part)) return 1;
        if (!writeFile(firmware + "/version.sh", "test fixture only\n")) return 1;
        QProcess process;
        auto env = QProcessEnvironment::systemEnvironment();
        env.insert("CNC_SHM_KEY", key);
        process.setProcessEnvironment(env);
        process.start("bash", {script, firmware, dir.path() + "/bundle",
            QString::number(QCoreApplication::applicationPid())});
        if (!process.waitForFinished(60000) || process.exitStatus() != QProcess::NormalExit || process.exitCode()) {
            fprintf(stderr, "%s\n", process.readAllStandardError().constData());
            return 1;
        }
        if (memcmp(beforeScript.constData(), owner.constData(), sizeof(NcData)) ||
            !QFile::exists(dir.path() + "/bundle/live/snapshot.json") ||
            !QFile::exists(dir.path() + "/bundle/process/rt-running.bin") ||
            !QFile::exists(dir.path() + "/bundle/SHA256SUMS")) return 1;
        puts("PASS: complete collection script with isolated fixture leaves shared memory unchanged");
    }
    puts("PASS: read-only capture preserves memory, captures force state, rejects decoding incompatible layout, never creates missing segments");
    return 0;
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if ((argc == 2 || argc == 3) && strcmp(argv[1], "--self-test") == 0)
        return selfTest(argc == 3 ? QString::fromLocal8Bit(argv[2]) : QString());
    if (argc != 2 && argc != 3) {
        fprintf(stderr, "Usage: rt_site_snapshot NEW_OUTPUT_DIRECTORY [SHM_KEY]\n");
        return 2;
    }
    const QString key = argc == 3 ? QString::fromLocal8Bit(argv[2]) :
        qEnvironmentVariable("CNC_SHM_KEY", "NcData");
    return capture(key, QString::fromLocal8Bit(argv[1])) ? 0 : 1;
}
