#ifndef THREADSAFESHAREDDATA_H
#define THREADSAFESHAREDDATA_H

#include <QReadWriteLock>
#include <QMutexLocker>
#include <QObject>
#include <cstring> // For memset

#include "cmd_msg.h"
#include "ci_pmc.h"
#include "CircularBuffer.h"
#include "status_msg.h"

class ThreadSafeSharedData : public QObject
{
    Q_OBJECT
public:
    static const int EMC_COMMOND_BUFFER_SIZE = 200;
    static const int PARSER_OUT_BUFFER_SIZE = 200;

    bool writeEmcCommandBufferData(RCS_CMD_MSG& data) {
        return emcCommandBuffer.write(data);
    }

    int readEmcCommandBufferData(RCS_CMD_MSG& data) {
        return emcCommandBuffer.read(data);
    }

    int getEmcCommandBufferDataCount() {
        return emcCommandBuffer.getCount();
    }

    bool isEmptyEmcCommandBufferData()
    {
        return (emcCommandBuffer.getCount() == 0);
    }


    bool isFullEmcCommandBufferData()
    {
        return (emcCommandBuffer.getCount() == EMC_COMMOND_BUFFER_SIZE);
    }


   void clearEmcCommandBufferData() { emcCommandBuffer.clear(); }


   bool writeParserOutBufferData(MotionDataSend& data) {
       return parserOutBuffer.write(data);
   }

   int readParserOutBufferData(MotionDataSend& data) {
       return parserOutBuffer.read(data);
   }

   int getParserOutBufferDataCount() {
       return parserOutBuffer.getCount();
   }

   bool isEmptyParserOutBufferData()
   {
       return parserOutBuffer.getCount() == 0;
   }

   bool isFullParserOutBufferData()
   {
       return parserOutBuffer.getCount() == PARSER_OUT_BUFFER_SIZE;
   }

   void clearParserOutBufferData() { parserOutBuffer.clear(); }

    static ThreadSafeSharedData& getInstance() {
        static ThreadSafeSharedData instance;
        return instance;
    }

    ThreadSafeSharedData(const ThreadSafeSharedData&) = delete;
    void operator=(const ThreadSafeSharedData&) = delete;

private:
    ThreadSafeSharedData() {
    }


   // DoubleBuffer<commandHmi> commandHmiBuffers;
    CircularBuffer<RCS_CMD_MSG, EMC_COMMOND_BUFFER_SIZE> emcCommandBuffer;
    CircularBuffer<MotionDataSend, PARSER_OUT_BUFFER_SIZE> parserOutBuffer;

};

#endif // THREADSAFESHAREDDATA_H
