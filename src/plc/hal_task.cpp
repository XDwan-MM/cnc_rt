#include "hal_task.h"
#include "hal_io.h"
#include "time.h"
#include "datatype.h"
#include "cyclescanning.h"
#include "mt_data.h"
#include "ladapi.h"
#include "global.h"

MT_PLC_IN plc_in={0};
PLC_MT_OUT plc_out={0};
LadData*Ladderdata=nullptr;

struct InfosGene
{
    uint64 period;
    int64  durationofLastScan;
} timeeee;

static int64 GetCurrentTime()
{
//    struct timespec ts;
//    clock_gettime(CLOCK_REALTIME,&ts);
//    return ts.tv_sec;
}

static void HalReadPhysicalInputs()
{
   uint32 input=0;
   uint16 data=0;
   uint16 buffer_mcp[11];
   //io
   Hal_Io_Read_Di(&hal_io_module[0],&input);
   Hal_Io_Update(&hal_io_module[0],1,0);
   plc_in.IO_In1Reg.all =input;

   //mcp
   for (int i = 0; i < 11; ++i)
   {
      GM_TxPdoEntry_Read(MCPDI[i],&data,2);
      buffer_mcp[i]=data;
      //printf("**buffer_mcp**%d******%d**\n",i,buffer_mcp[i]);
   }
   plc_in.pd_data.keyboardIn1.all=buffer_mcp[0];
   plc_in.pd_data.keyboardIn2.all=buffer_mcp[1];
   plc_in.pd_data.keyboardIn3.all=buffer_mcp[2];
   plc_in.pd_data.keyboardIn4.all=buffer_mcp[3];
   plc_in.pd_data.keyboardIn5.all=buffer_mcp[4];
   plc_in.pd_data.keyboardIn6.all=buffer_mcp[5];
   plc_in.pd_data.keyboardIn7.all=buffer_mcp[6];
   plc_in.pd_data.keyboardIn8.all=buffer_mcp[7];
   plc_in.pd_data.keyboardInb.all=buffer_mcp[8];
   plc_in.pd_data.keyboardIn9.all=buffer_mcp[9];
     //   plc_in.pd_data.keyboardIn8.all=buffer_mcp[10];
}

static void HalWritePhysicalOutputs()
{
   uint32 output=1;
   uint16 data[7]={0};
   //********io*********
   output=plc_out.IO_Out1Reg.all;  
   Hal_Io_Write_Do(&hal_io_module[0],output);
   Hal_Io_Update(&hal_io_module[0],0,1);

   //********mcp**********
   data[0]=plc_out.panel_show.keyboardOut1.all;
   data[1]=plc_out.panel_show.keyboardOut2.all;
   data[2]=plc_out.panel_show.keyboardOut3.all;
   data[3]=plc_out.panel_show.keyboardOut4.all;
   data[4]=plc_out.panel_show.keyboardOut5.all;
   data[5]=plc_out.panel_show.keyboardOut6.all;
   data[6]=plc_out.panel_show.keyboardOut7.all;
//   printf("****REGY10.0******%d**\n",plc_out.panel_show.keyboardOut4.all);
   for (int i = 0; i < 7; ++i) {
       GM_RxPdoEntry_Write(MCPDO[i],data[i]);
//         printf("***MCPDO[i]*%d******%d**\n",i,data[i]);
   }
}

void* hal_task()
{
    // uint64 t0,t1,milliseconds=0;
    // static uint64 leftover=0;
    // leftover+=timeeee.period;

    //后续此处加一个控制开关,不需要每次都拷贝（one_time）
    static bool ret=false;//指令数据获取成功标志位，拷贝1_time
//     qDebug()<<"****梯图信息,指令数据测试****"<<getLadDataPtr()->instrs[19].opcode;

    if(getLadDataPtr()->instrs[0].opcode == 0) printf("ERROR: plc hal task error");

    getLadDataPtr()->info.isRunning=1;
    if(getLadDataPtr()->info.isRunning!=0&&!ret)
    {
       static bool flag=1;
       if(flag==true)
       {
           Ladderdata=getLadDataPtr();//拷贝共享内存数据
           flag=false;
       }
        // qDebug()<<"****梯图信息,指令数据索引****"<<getLadDataPtr()->instrs[19].regIdx;
        int index=getLadDataPtr()->instrs[3].regIdx;
//         qDebug()<<"****梯图信息,指令数据变量****"<<char(getLadDataPtr()->regs[index].type)
//                                               <<getLadDataPtr()->regs[index].idx
//                                               <<int(getLadDataPtr()->regs[index].bit);
//         ret=false;
         //    Hal_Io_Update(&hal_io_module[0],true,false);
         // if(milliseconds<=1)
         // {
         //     t0=GetCurrentTime();
         //     if(timeeee.period)
         //     {
                 HalReadPhysicalInputs();  //*********物理层输入********
                 CycleScanning();          //*********循环扫描**********
                 HalWritePhysicalOutputs();//*********物理层输出*********
         //     }
         //     t1=GetCurrentTime();
         //     timeeee.durationofLastScan=t1-t0;//******单周期执行时间*******
         //     milliseconds=timeeee.durationofLastScan*1000;
         // }
         //     Hal_Io_Update(&hal_io_module[0],false,true);
    }
}

