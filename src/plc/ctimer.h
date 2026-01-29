/*****************************************************************
 * Copyright @ GreeElectric Co.,Ltd.2025-2035.All right reserved
 * File name :     cyclescanning.h
 * Description:    1.简单叙述本文的内容，程序完成的主要功能；
 *                 2.与其他模块或函数的接口；
 *                 3.其他内容的说明
 *
 * ***************************************************************
 *作者：     黄松清
 *日期：     2025.9.25
 *版本：     V2.0
 * ***************************************************************/
#ifndef CTIMER_H
#define CTIMER_H
#include <sys/time.h>
//*********时钟函数类**********

class CTimer
{
public:
    //**********CTimer();**************
    inline CTimer()
    {
        gettimeofday(&startTime,nullptr);
    }
    inline void reset()
    {
        gettimeofday(&startTime,nullptr);
    }
    inline double end()
    {
        gettimeofday(&endTime,nullptr);
        //        double elapsedTime = (endTime.tv_sec - startTime.tv_sec)*1000*1000+
        //       (endTime.tv_usec - startTime.tv_usec);//秒，微秒系统函数（最终为us）
        double elapsedTime = (endTime.tv_sec - startTime.tv_sec);
        //********************毫秒*************************
        return  elapsedTime;//**********执行时间差值***********
    }
private:
    timeval startTime,endTime;
};

#endif // CTIMER_H
