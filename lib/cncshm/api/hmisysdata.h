#ifndef HMISYSDATA_H
#define HMISYSDATA_H

#define NCFILEPATH_CHAR_LEN     (256)   // nc程序路径长度
#define MDICONTENT_CHAR_LEN     (2048)  // MDI程序内容长度
#define PITCH_COUNT_MAX        (2000)  // 螺补数组个数
#define AXIS_COUNT_MAX          (3)     // 轴个数

/*! 坐标系类型 */
typedef enum {
    CRDS_TYPE_REL = 0,   // 相对
    CRDS_TYPE_EXT = 1,   // EXT
    CRDS_TYPE_G5X = 2,   // G54~G59
    CRDS_TYPE_COUNT = CRDS_TYPE_G5X + 6,
} CRDS_TYPE;

typedef struct
{
    char ncFilePath[NCFILEPATH_CHAR_LEN];   // 加载文件路径
    char mdiContent[MDICONTENT_CHAR_LEN];   // Mdi内容
    float pitchData[PITCH_COUNT_MAX];     // 螺距补偿
    double coordData[CRDS_TYPE_COUNT][AXIS_COUNT_MAX];  // 坐标数据
} HmiSysData;

#endif // HMISYSDATA_H
