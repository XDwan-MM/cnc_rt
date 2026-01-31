#ifndef ALARMDATA_H
#define ALARMDATA_H

#include <time.h>

#define ALARM_COUNT_MAX 100
#define ALARM_HIS_COUNT_MAX 1000

// ---定义报警级别 ---
typedef enum {
    ALARM_ERROR,    // 错误
    ALARM_WARNING,  // 警告
    ALARM_LEVEL_ALL,
} AlarmLevel;

/*!	报警错误类型 */
typedef enum
{
    ALARM_SY = 0,	/*!< 	系统报警（System） */
    ALARM_CH,		/*!< 	通道报警（Channel） */
    ALARM_AX,		/*!< 	轴报警（Axis） */
    ALARM_SV,		/*!< 	伺服报警（Servo） */
    ALARM_PC,		/*!< 	PLC报警（PLC） */
    ALARM_DV,		/*!< 	设备报警（Dev） */
    ALARM_PS,		/*!< 	语法报警（Program Syntax） */
    ALARM_UP,		/*!< 	用户PLC报警（User PLC） */
    ALARM_HM,		/*!< 	HMI报警（HMI） */
    ALARM_ETHCAT_SV,/*!< 	EtherCAT饲服报警 */
    ALARM_TYPE_ALL
} AlarmType;

// 统一的报警数据结构
typedef struct {
    int id;             // 报警号
    int type;           // 报警类型
    int level;          // 报警级别
    time_t start;       // 报警发生时间
} AlarmDataDef;

typedef struct {
    AlarmDataDef datas[ALARM_COUNT_MAX];
    int count;
} AlarmDatas;

// 报警历史数据
typedef struct {
    int id;
    int type;
    int level;
    time_t start;
    time_t end;
} AlarmHisDataDef;

typedef struct {
    AlarmHisDataDef datas[ALARM_HIS_COUNT_MAX];  // 存储元素的固定大小数组
    int front;                                  // 队头下标（指向队头元素）
    int rear;                                   // 队尾下标（指向队尾元素的下一个位置）
} AlarmHisDatas;

typedef struct {
    AlarmDatas alms;
    AlarmHisDatas hiss;
} AlarmDataBuffer;



#endif // ALARMDATA_H
