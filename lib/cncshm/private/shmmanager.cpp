#include "shmmanager.h"
#include "qdebug.h"
#include "qglobal.h"
#include <QDebug>
#include <QCoreApplication>

#ifdef Q_OS_WIN
#include <windows.h>
#else
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#endif

ShmManager::ShmManager()
    : _shm("NcData") {
}

ShmManager::~ShmManager() {
    if (_isInit) {
        if (!_shm.detach()) {
            qWarning() << "无法分离共享内存：" << _shm.errorString();
        }
    }
    if (_regManager) {
        delete _regManager;
        _regManager = nullptr;
    }
    if (_ladManager) {
        delete _ladManager;
        _ladManager = nullptr;
    }
    if (_hmiSysManager) {
        delete _hmiSysManager;
        _hmiSysManager = nullptr;
    }
    if (_varManager) {
        delete _varManager;
        _varManager = nullptr;
    }
    if (_parManager) {
        delete _parManager;
        _parManager = nullptr;
    }
    if (_alarmManager) {
        delete _alarmManager;
        _alarmManager = nullptr;
    }
}

bool ShmManager::init() {
    qDebug() << "当前共享内存库版本为：" << _version;

    if (_isInit) return false;

    bool needReset = false;

    // 获取当前进程 PID
    uint64_t myPid = static_cast<uint64_t>(QCoreApplication::applicationPid());

    if (_shm.attach()) {
        qDebug() << "共享内存已存在，连接成功";
        _ncData = (NcData*)_shm.data();
        
        // 使用跨平台函数检测
        bool p1Alive = isProcessAlive(_ncData->pid1);
        bool p2Alive = isProcessAlive(_ncData->pid2);

        if (!p1Alive && !p2Alive) {
            qDebug() << "检测到所有记录的进程均已不存在，判定为残留文件";
            needReset = true;
        } else {
            qDebug() << "检测到活跃进程，加入共享内存";
        }
        
    } else {
        qDebug() << "无法连接，尝试创建...";
        if (_shm.create(sizeof(NcData))) {
            _ncData = (NcData*)_shm.data();
            needReset = true;
        } else {
            qWarning() << "无法连接共享内存：" << _shm.errorString();
            qWarning() << "可能原因：上次使用管理员权限(sudo)连接共享内存，但是释放不干净，本次使用非管理员权限执行。";
            qWarning() << "解决方案1：建议使用管理员权限执行。";
            qWarning() << "解决方案2：使用管理员权限删除共享内存段ipcs -m（找到目标段）,ipcrm -m [填入内存段id]。然后删除残留文件sudo rm /tmp/qipc_s*";
            Q_ASSERT(false);
            return false;
        }
    }

    if (!_ncData) return false;

    // 如果判定为残留或新建，清空数据
    if (needReset) {
        qDebug() << "重置共享内存数据";
        memset(_ncData, 0, sizeof(NcData));
    }

    // 注册当前进程 PID
    if (!isProcessAlive(_ncData->pid1)) {
        _ncData->pid1 = myPid;
        qDebug() << "注册 PID1:" << myPid;
    } else if (!isProcessAlive(_ncData->pid2)) {
        _ncData->pid2 = myPid;
        qDebug() << "注册 PID2:" << myPid;
    } else {
        qWarning() << "错误原因：已经有两个进程连接至共享内存，不允许再连接！";
        qWarning() << "解决方案：1. 使用 ps -ef | awk 'NR==1 || /hmi/' 查看hmi进程id";
        qWarning() << "         2. 使用 kill -9 [pid] 杀掉进程，如果杀不掉使用 kill -9 [ppid]";
        Q_ASSERT(false);
        return false;
    }

    // 创建管理器
    _regManager = new RegManager(_ncData->regData);
    _ladManager = new LadManager(_ncData->ladData);
    _hmiSysManager = new HmiSysManager(_ncData->hmiSysData);
    _varManager = new VarManager(_ncData->varData);
    _parManager = new ParManager(_ncData->parData);
    _alarmManager = new AlarmManager(_ncData->alarmDataBuffer);

    setIsInit(true);
    return true;
}

void ShmManager::setIsInit(bool isInit) {
    _isInit = isInit;
    if (_ladManager) {
        _ladManager->setIsShmInit(_isInit);
    }
    if (_hmiSysManager) {
        _hmiSysManager->setIsShmInit(_isInit);
    }
    if (_varManager) {
        _varManager->setIsShmInit(_isInit);
    }
    if (_parManager) {
        _parManager->setIsShmInit(_isInit);
    }
    if (_alarmManager) {
        _alarmManager->setIsShmInit(_isInit);
    }
}

bool ShmManager::isProcessAlive(int pid) {
    if (pid <= 0) return false;

#ifdef Q_OS_WIN
    // Windows 实现
    // 尝试打开进程，只需要查询权限
    HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION, FALSE, (DWORD)pid);
    if (hProcess == NULL) {
        return false; // 无法打开，通常意味着进程不存在
    }
    
    // 获取退出代码来进一步确认
    DWORD exitCode = 0;
    if(GetExitCodeProcess(hProcess, &exitCode)) {
        CloseHandle(hProcess);
        // 如果退出代码是 STILL_ACTIVE (259)，表示进程还在运行
        return (exitCode == STILL_ACTIVE);
    }
    
    CloseHandle(hProcess);
    return false;

#else
    // Linux/macOS/Unix 实现
    // kill(pid, 0) 用于检测进程是否存在
    errno = 0;
    if (kill((pid_t)pid, 0) == 0) {
        return true; // 进程存在
    }
    // 如果返回 -1 且 errno == ESRCH，说明进程不存在
    // 如果 errno == EPERM，说明进程存在但无权操作（由于是同一个用户启动的程序，这种情况很少见，但也视为存在）
    if (errno == EPERM) {
        errno = 0;
        return true;
    }
    errno = 0;
    return false;
#endif
}