#ifndef EMCCFG_H
#define EMCCFG_H

#ifdef __cplusplus
extern "C" {
#endif


#define DEFAULT_MDI_NC_FILE_PATH "MDI.nc" //MDI.nc文件路径



/* cycle time for emctask, in seconds */
#define DEFAULT_EMC_TASK_CYCLE_TIME 0.100


/* default interp len */
#define DEFAULT_EMC_TASK_INTERP_MAX_LEN 1000 //译码器最大处理长度

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif

#endif // EMCCFG_H
