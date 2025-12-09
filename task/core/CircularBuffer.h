#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <QReadWriteLock>
#include <array>
#include <cstring>

template <typename T, int BUFFER_SIZE>
class CircularBuffer {
public:
    bool write(const T& data) {
        QWriteLocker locker(&lock);
        if (count >= BUFFER_SIZE) {
            return false;
        }
        buffer[writeIndex] = data;
        writeIndex = (writeIndex + 1) % BUFFER_SIZE;
        count++;
        return true;
    }

    bool read(T& data){
        QReadLocker locker(&lock);
        if (count > 0) {
            data = buffer[readIndex];
            readIndex = (readIndex + 1) % BUFFER_SIZE;
            count--;
            return true;
        }
        return false;
    }

    int getCount() const{
        QReadLocker locker(&lock);
        return count;
    }

    void clear() {
        QWriteLocker locker(&lock);
        writeIndex = 0;
        readIndex = 0;
        count = 0;
    }

private:
    std::array<T, BUFFER_SIZE> buffer;
    int writeIndex = 0;
    int readIndex = 0;
    int count = 0;
    mutable QReadWriteLock lock;
};

template <typename T, int Rows, int Cols>
class DoubleBufferRowsCols {
public:
    DoubleBufferRowsCols() {
        currentReadBuffer = buffer1;
        currentWriteBuffer = buffer2;
    }

    void write(const T data[Rows][Cols]) {
        QWriteLocker locker(&lock);
        std::memcpy(currentWriteBuffer, data, sizeof(buffer1));
        std::swap(currentReadBuffer, currentWriteBuffer);
    }

    void read(T data[Rows][Cols]) {
        QReadLocker locker(&lock);
        std::memcpy(data, currentReadBuffer, sizeof(buffer1));
    }

private:
    T buffer1[Rows][Cols];
    T buffer2[Rows][Cols];
    T (*currentReadBuffer)[Cols];
    T (*currentWriteBuffer)[Cols];
    QReadWriteLock lock;
};

template <typename T>
class DoubleBufferSingle
{
public:
    DoubleBufferSingle() {
        *currentReadBuffer = buffer1;
        *currentWriteBuffer = buffer2;
    }

    void write(const T data) {
        QWriteLocker locker(&lock);
        *currentWriteBuffer = data;
        std::swap(currentReadBuffer, currentWriteBuffer);
    }

    void read(T data) {
        QReadLocker locker(&lock);
        data = *currentReadBuffer;
    }
private:
    T buffer1;
    T buffer2;
    T *currentReadBuffer;
    T *currentWriteBuffer;
    QReadWriteLock lock;
};

template<typename T>
class DoubleBuffer {
public:
    DoubleBuffer() {
        buffers[0] = new T();
        buffers[1] = new T();
    }

    ~DoubleBuffer() {
        delete buffers[0];
        delete buffers[1];
    }

    void write(const T& newData) {
        QWriteLocker locker(&lock);
        *buffers[currentIndex] = newData;
        currentIndex = (currentIndex + 1) % 2;
        count = 1;
    }

    bool read(T& outData) {
        QReadLocker locker(&lock);
        if (count > 0) {
            outData = *buffers[(currentIndex + 1) % 2];
            return true;
        }
        return false;
    }

    void clear() {
        QWriteLocker locker(&lock);
        count = 0;
    }

    int getCount() const {
        QReadLocker locker(&lock);
        return count;
    }

private:
    T* buffers[2];
    int currentIndex = 0;
    int count = 0;
    mutable QReadWriteLock lock;
};
#endif // CIRCULARBUFFER_H
