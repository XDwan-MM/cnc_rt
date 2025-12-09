#ifndef JSON_PARSE_H
#define JSON_PARSE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************** 
 * @file        json_read.h
 * @brief       json解码
 * @version     V1.00
 * @date        2025/10/15
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/


#include "./datadef/common.h"
#include "./datadef/plan_queue.h"

/**
 * @brief 解析 param.json 文件并填充 params 结构体
 * @param filepath param.json 文件的路径
 * @param params 指向 MotionParams 结构体的指针，用于存储解析结果
 * @return int 成功返回 0，失败返回 -1
 * @author 许晓婷
 * @version 1.0
 * @date 2025/10/15
 */
int parse_param_file(const char *filepath, PARAMS *params);

/**
 * @brief 解析 tp.json 文件并生成 TrajectorySegment 数组
 * @param filepath tp.json 文件的路径
 * @param tc_array 指向 TrajectorySegment 指针的指针，函数将为数组分配内存
 * @return int 成功返回段的数量，失败返回 -1。调用者负责释放 tc_array 指向的内存
 * @author 许晓婷
 * @version 1.0
 * @date 2025/10/15
 */
int parse_tp_file(const char *filepath, PLAN_QUEUE* pl_queue);

/**
 * @brief 解析 dcd.json 文件并生成解释译码数组
 * @param filepath 文件的路径
 * @param pos_array 指向  指针的指针，函数将为数组分配内存
 * @return int 成功返回段的数量，失败返回 -1。
 * @author 许晓婷
 * @version 1.0
 * @date 2025/10/16
 */
int parse_dcd_file(const char *filepath, GCODE_QUEUE *gcode_queue);

#ifdef __cplusplus
}
#endif

#endif // JSON_PARSE_H