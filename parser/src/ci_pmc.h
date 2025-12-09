#ifndef CI_PMC_h
#define CI_PMC_h
#ifdef __cplusplus
extern "C" {
#endif

#include "motiondata.h"

//other+
typedef struct
{
    pmc_data motiondata;
    int lineNumCount;
    int lineNumber;
    int dataindex;      // -3:结束 -2:译码报警 -1:空包 >=1：实包
    int tier;           // 当前执行程序层级 0:无程序 n:n个程序
    int _type;           // 消息类型
}MotionDataSend;


#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif
#endif // CI_PMC_h
