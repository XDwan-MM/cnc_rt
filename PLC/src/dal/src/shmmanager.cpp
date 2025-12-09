#include "shmmanager.h"
#include <QDebug>
#include <string.h>

ShmManager::ShmManager()
    : _shm(_shmName) {
}

ShmManager::~ShmManager() {
    if (_isInit) {
        detach();
    }
    if (_regManager) {
        delete _regManager;
    }
    if (_ladManager) {
        delete _ladManager;
    }
}

bool ShmManager::init() {
    // TODO: 日志
    if (_isInit) {
        qDebug() << "已经初始化过共享内存，不要重复初始化";
        return false;
    }

    if (_shm.attach()) {
        qDebug() << "共享内存已存在，连接成功";
        _isCreator = false;
    } else {
        // 如果附加失败，说明共享内存不存在，那么需要创建
        qDebug() << "无法连接到共享内存，尝试创建...";
        if (_shm.create(sizeof(NcData))) {
            qDebug() << "共享内存创建成功";
            _isCreator = true;
        } else {
            // 如果附加和创建都失败了，那就是真的出错了
            qDebug() << "在共享内存连接和创建时均出现错误——" << _shm.errorString();
            return false;
        }
    }

    // 获取共享内存数据
    _ncData = (NcData*)_shm.data();
    if (!_ncData) {
        qDebug() << "从共享内存获取数据指针失败";
        _shm.detach();
        return false;
    }

    // 只有创建者负责将共享内存数据清零
    if (_isCreator) {
        memset(_ncData, 0, sizeof(NcData));
        qDebug() << "创建者已将共享内存初始化为0";
    }

    // 创建管理器
    _regManager = new RegManager(_ncData->regData);
    _ladManager = new LadManager(_ncData->ladData);

    setIsInit(true);
    return true;
}

bool ShmManager::detach() {
    // TODO: 日志
    if (!_shm.detach()) {
        qDebug() << "共享内存从当前进程分离失败";
        return false;
    }
    qDebug() << "共享内存从当前进程分离成功";
    setIsInit(false);
    _isCreator = false;
    return true;
}

void ShmManager::setIsInit(bool isInit) {
    _isInit = isInit;
    _regManager->setIsShmInit(_isInit);
    _ladManager->setIsShmInit(_isInit);
}
