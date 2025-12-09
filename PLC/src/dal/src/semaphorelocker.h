#pragma once

#include <QSystemSemaphore>

class SemaphoreLocker {
public:
    explicit SemaphoreLocker(QSystemSemaphore& semaphore)
        : _semaphore(semaphore) {
        // 在构造函数中请求锁，程序会在此处阻塞直到获取成功
        _semaphore.acquire();
    }

    // 析构函数释放锁
    ~SemaphoreLocker() {
        // 当Locker对象生命周期结束时，自动释放锁
        _semaphore.release();
    }

    // 删除拷贝构造函数和拷贝赋值运算符，防止对象被复制
    SemaphoreLocker(const SemaphoreLocker&) = delete;
    SemaphoreLocker& operator=(const SemaphoreLocker&) = delete;

    // 删除移动操作
    SemaphoreLocker(SemaphoreLocker&&) = delete;
    SemaphoreLocker& operator=(SemaphoreLocker&&) = delete;

private:
    QSystemSemaphore& _semaphore;
};
