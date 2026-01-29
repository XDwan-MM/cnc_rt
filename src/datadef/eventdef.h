/******************************************************************** 
 * @file        event_queue.h
 * @brief       事件队列
 * @version     V1.00
 * @date        2026/01/28
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef __EVENT_DEF_H__
#define __EVENT_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    ncEvtPoweroff = 0,          // 系统断电
    ncEvtXPosPress,             // 按下X正向移动
    ncEvtXNegPress,             // 按下X负向移动
    ncEvtYPosPress,             // 按下Y正向移动
    ncEvtYNegPress,             // 按下Y负向移动
    ncEvtZPosPress,             // 按下Z正向移动
    ncEvtZNegPress,             // 按下Z负向移动
    ncEvtXPosRelease,           // 释放X正向移动
    ncEvtXNegRelease,           // 释放X负向移动
    ncEvtYPosRelease,           // 释放Y正向移动
    ncEvtYNegRelease,           // 释放Y负向移动
    ncEvtZPosRelease,           // 释放Z正向移动
    ncEvtZNegRelease,           // 释放Z负向移动
    ncEvtHoldPress,             // 按下进给保持
    ncEvtJogPress,              // 按下手动模式
    ncEvtRapidPress,            // 按下快速模式
    ncEvtMemPress,              // 按下记忆模式
    ncEvtRstPress,              // 按下复位
    ncEvtCycPress,              // 按下循环启动
    ncEvtEStopPress,            // 按下急停
}SYS_EVENT_CODE;    

// typedef enum{
//     ncEvtPoweroff = 0,          // 系统断电
//     ncEvtSaveData,              // 保存系统数据
//     ncEvtSysExit,               // 系统退出
//     ncEvtUserStart,             // 用户自定义事,保留100个
//     ncEvtHoldPress,             // 进给保持
//     ncEvtXPosPress,             // X正向移动按下
//     ncEvtXNegPress,             // X负向移动按下
//     ncEvtYPosPress,             // Y正向移动按下
//     ncEvtYNegPress,             // Y负向移动按下
//     ncEvtZPosPress,             // Z正向移动按下
//     ncEvtZNegPress,             // Z负向移动按下
//     ncEvtRapidOverrideChg,      // 快速移动倍率改变
//     ncEvtFeedOverrideChg,       // 进给倍率改变
//     ncEvtSpindleOverrideChg,    // 主轴倍率改变
//     ncEvtHardRstFin = 104,      // 硬复位完成	
//     ncEvtSaveRegB,              // 保存B寄存器	
//     ncEvtUserReqChn,            // 请求切换通道
//     ncEvtUserReqMsk,            // 请求屏蔽通道
//     ncEvtReserve0,              // 用户保留PLC事件(DMTG使用)
//     ncEvtReserve1,              // 用户保留PLC事件(DMTG使用)
//     ncEvtReserve2,              // 用户保留PLC事件(DMTG使用)
//     ncEvtReserve3,              // 用户保留PLC事件(DMTG使用)
//     ncEvtChOffset,              // 及时修改坐标偏置(G54等与刀偏)
//     ncEvtCleanCurTime,          // PLC通知NC清单次切削时间
//     ncEvtCycPress,              // 按下循环启动
//     ncEvtWorkMeasDone,          // 工件测量点位到达
//     ncEvtIOPoweroff,            // 按下IO断电事件
//     ncEvtResetPlcKey,           // PLC启动的复位(PLC激发的事件代替面板Reset按键)
//     ncEvtLoadPrg,               // 导入由VAR_BLK_LOAD_PROG_NO指定的程序号
//     ncEvtProgRestart,           // 重运行
//     ncEvtToolProgFlowEnd,       // 自动对刀完成【玻璃机】
//     ncEvtMcpPress,              // 有mcp按键按下
//     ncEvtRstMsg,                // 复位消息(多通道时，PLC选择完复位的通道后发送的事件)
//     ncEvtRigChg,                // 钥匙锁切换权限消息
//     ncEvtRfidReadTag,           // RFID读电子标签数据到CNC
//     ncEvtRfidWriteTag,          // RFID写CNC数据到电子标签
//     ncEvtToolChangeFin0,        // 通道0换刀完成
//     ncEvtToolChangeFin1,        // 通道1换刀完成
//     ncEvtToolChangeFin2,        // 通道2换刀完成
//     ncEvtToolChangeFin3,        // 通道3换刀完成
// }SYS_EVENT_CODE;    


// typedef enum{
    // ncEvtPrgStar = 0,    //程序启动
    // ncEvtPrgEnd,         //程序结束
    // ncEvtPrgHold,        //Hold完成	
    // ncEvtPrgBreak,       //break完成
    // ncEvtG92Fin,         //G92完成
    // ncEvtRstFin,         //上电复位完成
    // ncEvtRwndFin,        //重运行完成
    // ncEvtMdiRdy,         //MDI准备好
    // ncEvtMdiExit,        //MDI退出
    // ncEvtMdiAck,         //MDI行解释完成
    // ncEvtRunStart,       //程序运行
    // ncEvtRunRowAck,      //任意行请求应答
    // ncEvtRunRowRdy,      //任意行准备好
    // ncEvtBpSaved,        //断点保存完成
    // ncEvtBpResumed,      //断点恢复完成
    // ncEvtIntvHold,       //执行到M92等待用户干预
    // ncEvtEstop,          //外部急停
    // ncEvtLoadOK,         //程序加载完成
    // ncEvtSyntax1,        //第一类语法错【修改后可接着运行】
    // ncEvtSyntax2,        //第二类语法错【修改后从头运行】
    // ncEvtGcodeSave,      //程序中的数据保存指令
    // ncEvtLoadData,       //程序中的数据加载指令
    // ncEvtChgTool,        //G代码修改了刀具数据
    // ncEvtChgCrds,        //G代码修改了坐标系数据
    // ncEvtChgAxes,        //通道轴组发生了改变
    // ncEvtChgVar,         //G代码修改变量
    // ncEvtNckNote,        //通道提示
    // ncEvtNckAlarm,       //通道报警
    // ncEvtStopAck,        //sys_stop_prog完成
    // ncEvtChgTimeVar,     //G代码修改时间变量
    // ncEvtChgParm,        //G代码修改参数
    // ncEvtVerifyFinish,   //G代码修改加工参数，包括用户变量、坐标系、刀具
    // ncEvtFastVerifyFinish,
    // ncEvtCallUserSubProg,
    // ncEvtG134FastVerifyFin,
    // ncEvtRandomRowFinish,        //任意行扫描完成 
    // ncEvtChgMachineParm,
    // ncEVtRandomRowRequestProg,   //任意行扫描模式请求程序管理器准备好程序
    // ncEvtCallG41,
    // ncEvtCallG43,
    // ncEvtCallG54,
    // ncEvtBpSaveFile,             //保存断点文件事件
    // ncEvtCallUserChoiceFile,     //内核通知界面操作文件事件
    // ncEvtBpSaveFail,             //保存断点失败事件，禁止保存断点
    // ncEvtFaultIrq,               //故障中断
    // ncEvtUserAlarm,
    // ncEvtCallUserSubProgFin,
    // ncEvtEstopBpAck,             //急停断点扫描请求应答
    // ncEvtEstopBpRdy,             //急停断点扫描准备好
    // ncEvtEstopBpFinish,          //急停断点扫描完成
    // ncEvtEstopBpRequestProg,     //急停断点扫描请求程序管理器准备好程序
    // ncEvtEstopBpFail,            //急停断点扫描请求失败
    // ncEvtRedecodeFin,            //重解释启动成功
    // ncEvtJogBpResumed,           //手动干预断点恢复完成
    // ncEvtCycTotalL,
    // ncEvtPackFin,                //数据打包完成
    // ncEvtAlarmChg,               //报警产生或消除
    // ncEvtFileChg,                //文件修改
    // ncEvtNotOnBP,                //机床不在断点位置事件
    // ncEvtConnect,                //nc连接	
    // ncEvtDisConnect,             //nc断开连接
    // ncEvtFileSend,               //传送文件完毕
    // ncEvtStopCheck,              //准停检查
// }CHAN_EVENT_CODE; 


// typedef enum{
    // ncEvtMaxEncPos = 0,     // 轴编码器初始位置过大
    // ncEvtMaxACC,            // 轴加速度过大
// }AXIS_EVENT_CODE;


// typedef enum{


// }NET_EVENT_CODE;

#ifdef __cplusplus
}
#endif
#endif