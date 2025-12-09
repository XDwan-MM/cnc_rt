#ifndef THREADSAFESHAREDDATA_C_H
#define THREADSAFESHAREDDATA_C_H

#include "ThreadSafeSharedData.h"

#ifdef __cplusplus
extern "C" {
#endif


// Function declarations for C interface
bool writeEmcCommandBufferData_C(RCS_CMD_MSG *data);
int readEmcCommandBufferData_C(RCS_CMD_MSG* data);
int getEmcCommandBufferDataCount_C();
bool isEmptyEmcCommanBufferData_C();
bool isFullEmcCommanBufferData_C();
void clearEmcCommandBufferData_C();

bool writeParserOutBufferData_C(MotionDataSend * data);
int readParserOutBufferData_C(MotionDataSend* data);
int getParserOutBufferDataCount_C();
bool isEmptyParserOutBufferData_C();
bool isFullParserOutBufferData_C();
void clearParserOutBufferData_C();
#ifdef __cplusplus
}
#endif

#endif // THREADSAFESHAREDDATA_C_H
