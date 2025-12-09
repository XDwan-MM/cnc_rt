#include "threadsafeshareddata_c.h"
#include "ThreadSafeSharedData.h"

bool writeEmcCommandBufferData_C(RCS_CMD_MSG* data) {
    return ThreadSafeSharedData::getInstance().writeEmcCommandBufferData(*data);
}

int readEmcCommandBufferData_C(RCS_CMD_MSG* data){
    return ThreadSafeSharedData::getInstance().readEmcCommandBufferData(*data);
}

int getEmcCommandBufferDataCount_C() {
    return ThreadSafeSharedData::getInstance().getEmcCommandBufferDataCount();
}


void clearEmcCommandBufferData_C() {
    ThreadSafeSharedData::getInstance().clearEmcCommandBufferData();
}

bool writeParserOutBufferBufferData_C(MotionDataSend* data) {
    return ThreadSafeSharedData::getInstance().writeParserOutBufferData(*data);
}

int readParserOutBufferBufferData_C(MotionDataSend* data) {
    return ThreadSafeSharedData::getInstance().readParserOutBufferData(*data);
}

int getParserOutBufferDataCount_C() {
    return ThreadSafeSharedData::getInstance().getParserOutBufferDataCount();
}


void clearParserOutBufferData_C() {
    ThreadSafeSharedData::getInstance().clearParserOutBufferData();
}



bool isEmptyEmcCommanBufferData_C()
{
    return ThreadSafeSharedData::getInstance().isEmptyEmcCommandBufferData();
}

bool isFullEmcCommanBufferData_C()
{
    return ThreadSafeSharedData::getInstance().isFullEmcCommandBufferData();

}

bool isEmptyParserOutBufferData_C()
{
    return ThreadSafeSharedData::getInstance().isEmptyParserOutBufferData();
}

bool isFullParserOutBufferData_C()
{
    return ThreadSafeSharedData::getInstance().isFullParserOutBufferData();
}
