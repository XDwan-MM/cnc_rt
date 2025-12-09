#ifndef TASKDATA_H
#define TASKDATA_H

// EMC_TASK 模式的枚举类型
enum EMC_OPERATION_MODE {
    JOG = 1,   // 点动模式
    RAPID = 2, //快速模式
    MPG = 3,   // 手轮模式
    ZRN = 4,   // 回原点模式
    EDIT = 5,  // 编辑模式
    MEM = 6,   // 自动模式
    MDI = 7,   // MDI (手动数据输入) 模式
    DNC = 8,   // 远程连接模式
};

//判断操作模式与运动控制器之间的配合关系。


//EMC_OPERATION_MODE OperatingMode; //操作模式

#endif // TASKDATA_H
