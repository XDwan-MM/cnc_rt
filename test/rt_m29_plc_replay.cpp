#include <algorithm>
#include <cassert>
#include <cstring>
#include <cstdio>
#include <stack>
#include <vector>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QMap>
#include <QCryptographicHash>

// Isolated register memory; execute selected firmware rungs with the real engine.
#define private public
#include "../src/plc/plc_task/plctask.h"
#undef private
#include "../src/plc/plc_task/plctask.cpp"

static LadData ladder;
static RegData registers;
extern "C" LadData *getLadDataPtr() { return &ladder; }
extern "C" RegData *getRegDataPtr() { return &registers; }

static void load(const char *path)
{
    QFile file(QString::fromLocal8Bit(path));
    assert(file.open(QIODevice::ReadOnly));
    QJsonParseError error;
    const auto bytes = file.readAll();
    const auto hash = QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex();
    if (hash != "2322499419de64db07a45af6266b54082ce6999302b6151651e02a57afad4da7")
        qFatal("Ladder differs from the reviewed fixture; validate instruction ranges before replay");
    const auto doc = QJsonDocument::fromJson(bytes, &error);
    assert(error.error == QJsonParseError::NoError);
    const auto root = doc.object();
    const auto regs = root["regs"].toArray();
    assert(regs.size() <= LAD_REG_MAX_NUM);
    for (int i = 0; i < regs.size(); ++i) {
        const auto reg = regs[i].toObject();
        ladder.regs[i].type = static_cast<RegType>(reg["type"].toInt());
        ladder.regs[i].idx = reg["idx"].toInt();
        ladder.regs[i].bit = reg["bit"].toInt();
    }
    const QMap<QString, Opcode> ops = {
        {"LD", OP_LD}, {"LDI", OP_LDI}, {"AND", OP_AND}, {"ANI", OP_ANI},
        {"OR", OP_OR}, {"ORI", OP_ORI}, {"ANB", OP_ANB}, {"ORB", OP_ORB},
        {"OUT", OP_OUT}, {"TMRB", OP_TMRB}
    };
    const auto instrs = root["instrs"].toArray();
    assert(instrs.size() <= LAD_IL_MAX_NUM);
    // Ranges are specific to LadData20260915.json; unknown ops fail.
    for (auto range : {std::pair<int,int>{51,82}, {2050,2071}}) {
        for (int i = range.first; i <= range.second; ++i) {
            auto ins = instrs[i].toObject();
            assert(ops.contains(ins["opcode"].toString()));
            ladder.instrs[i].opcode = ops[ins["opcode"].toString()];
            ladder.instrs[i].regIdx = ins["regIdx"].toInt();
        }
    }
    const auto &out = ladder.regs[ladder.instrs[62].regIdx];
    assert(out.type == REG_R && out.idx == 1 && out.bit == 0);
    const auto &cw = ladder.regs[ladder.instrs[2071].regIdx];
    assert(cw.type == REG_G && cw.idx == 70 && cw.bit == 5);
}

static void bit(char *bank, int idx, int bit, bool value)
{ bank[idx] = (bank[idx] & ~(1 << bit)) | (value ? 1 << bit : 0); }
static bool bit(const char *bank, int idx, int bit)
{ return (bank[idx] & (1 << bit)) != 0; }

static void scan(PlcTask &plc, int count)
{
    PlcContext ctx{};
    plc._currCtx = &ctx;
    for (int cycle = 0; cycle < count; ++cycle) {
        ++plc.timers[64];
        ++plc.timers[65];
        for (auto range : {std::pair<int,int>{51,82}, {2050,2071}}) {
            ctx._logicStack = 0;
            for (ctx.pc = range.first; ctx.pc <= range.second; ++ctx.pc) {
                const auto &ins = ladder.instrs[ctx.pc];
                (plc.*plc._dispatchTable[ins.opcode])(ins);
            }
        }
    }
}

static void prepare(PlcTask &plc)
{
    std::memset(&registers, 0, sizeof(registers));
    std::fill(std::begin(plc.timers), std::end(plc.timers), 0);
    bit(registers.regR, 108, 4, true); // Estop released.
    bit(registers.regR, 1000, 3, true); // Tool clamped.
    bit(registers.regF, 0, 6, true); // Drives healthy.
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    load(argv[1]);
    PlcTask plc;
    for (int preceding = 0; preceding < 3; ++preceding) {
      for (bool rigid : {false, true}) {
        prepare(plc);
        if (preceding != 0) {
            bit(registers.regR, 282, 1, true); // M3 pulse from upstream rung.
            scan(plc, 1);
            bit(registers.regR, 282, 1, false);
            scan(plc, 100);
            assert(bit(registers.regG, 70, 5));
        }
        if (preceding == 2) {
            bit(registers.regR, 284, 1, true); // M5 from upstream rung.
            scan(plc, 100);
            assert(!bit(registers.regG, 70, 5));
            bit(registers.regR, 284, 1, false);
        }
        bit(registers.regF, 76, 3, rigid);
        bit(registers.regR, 643, 5, true);
        scan(plc, 700);
        assert(bit(registers.regG, 70, 5));
        assert(bit(registers.regR, 1, 0));
        printf("PASS: preceding=%s, F76.3=%d, G70.5=1 R1.0=1\n",
               preceding == 0 ? "cold" : preceding == 1 ? "M3" : "M3/M5", rigid);
      }
    }
    prepare(plc);
    bit(registers.regF, 76, 3, true);
    bit(registers.regR, 643, 5, true);
    scan(plc, 700);
    bit(registers.regR, 643, 5, false);
    bit(registers.regF, 76, 3, false);
    scan(plc, 1);
    assert(bit(registers.regR, 500, 3));
    scan(plc, 1);
    assert(!bit(registers.regG, 61, 0));
    bit(registers.regR, 643, 5, true);
    bit(registers.regF, 76, 3, true);
    scan(plc, 1500);
    assert(bit(registers.regR, 1, 0));
    puts("PASS: F76.3 falling triggers termination, next M29 recovers with healthy interlocks");

    prepare(plc);
    bit(registers.regR, 643, 5, true);
    bit(registers.regF, 76, 3, true);
    bit(registers.regR, 284, 0, true); // Panel-stop interlock.
    scan(plc, 1500);
    assert(!bit(registers.regG, 70, 5));
    assert(!bit(registers.regR, 1, 0));
    puts("PASS: held spindle-stop interlock blocks M29 completion");
    bit(registers.regR, 284, 0, false);
    scan(plc, 700);
    assert(bit(registers.regR, 1, 0));
    puts("PASS: releasing spindle-stop interlock restores M29 completion");

    prepare(plc);
    bit(registers.regR, 643, 5, true);
    bit(registers.regF, 76, 3, true);
    bit(registers.regF, 0, 6, false);
    scan(plc, 1);
    assert(bit(registers.regR, 500, 3));
    scan(plc, 1);
    assert(!bit(registers.regG, 61, 0));
    int requests = 0;
    for (int i = 0; i < 1600; ++i) {
        scan(plc, 1);
        if (bit(registers.regG, 61, 0)) ++requests;
        assert(!bit(registers.regR, 1, 0));
        assert(!bit(registers.regR, 560, 1));
    }
    assert(requests == 3);
    puts("PASS: F0.6=0 reproduces repeated 500 ms abort/retry without F76.3 falling");
    bit(registers.regF, 0, 6, true);
    scan(plc, 1500);
    assert(bit(registers.regR, 1, 0));
    puts("PASS: restoring drive health allows completion with healthy clamp/interlocks");

    prepare(plc);
    bit(registers.regR, 643, 5, true);
    bit(registers.regK, 1, 3, true);
    scan(plc, 1500);
    assert(!bit(registers.regG, 70, 5));
    assert(!bit(registers.regR, 1, 0));
    puts("PASS: reverse-tapping selection blocks the CW-only completion rung in this fixture");
    prepare(plc);
    bit(registers.regR, 643, 5, true);
    bit(registers.regR, 1000, 3, false);
    scan(plc, 700);
    assert(!bit(registers.regG, 70, 5));
    // Forcing changes rung evaluation, not the register.
    ladder.instrs[2062].state = LAD_FORCE_ENABLE;
    scan(plc, 700);
    assert(!bit(registers.regR, 1000, 3));
    assert(bit(registers.regR, 999, 5));
    assert(bit(registers.regG, 70, 5));
    assert(bit(registers.regR, 1, 0));
    ladder.instrs[2062].state = LAD_FORCE_NORMAL;
    puts("PASS: forced clamp contact explains G70.5=1 with R1000.3=0 without a ladder edit");
    puts("Scope: selected real rungs only; external interlocks/NC inputs are injected, not a full machine replay.");
}
