/******************************************************************** 
 * @file        varapi数据结构.h
 * @brief       
 * @version     V1.01
 * @date        2025/12/31
 * @author      许晓婷
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
********************************************************************/

#ifndef VARDATATYPE_H
#define VARDATATYPE_H

#include "gncdatadef.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 变量API错误码
#define	VAR_SUCCESS                 (0)		// 成功
#define	VAR_TYPE_ERR 	            (-1)	// 函数与变量类型不匹配 
#define VAR_AXISNO_ERR			    (-2)	// 轴号超过限制
#define VAR_CHANNO_ERR			    (-3)	// 通道号超过限制
#define VAR_AXISINDEX_ERR		    (-4)	// 轴变量索引号超过限制
#define VAR_CHANINDEX_ERR		    (-5)	// 通道变量索引号超过限制
#define VAR_SYSINDEX_ERR		    (-6)	// 系统变量索引号超过限制
#define VAR_BITNO_ERR			    (-7)	// 变量位号索引号超过限制
#define VAR_POINT_NULL_ERR		    (-8)	// 变量值指针为空
#define VAR_SHM_INIT_FAILED         (-9)    // 共享内存未初始化

// var 变量
#define VAR_CHAN_BASE 	(0)             // 通道数据的变量接口起始编号
#define VAR_SYS_BASE 	(40000)	        // 系统数据的变量接口起始编号
#define VAR_AXES_BASE 	(60000)	        // 轴数据的变量接口起始编号
#define VAR_TOOL_BASE 	(70000)	        // 刀具数据的变量起始编号
#define VAR_CHAN_NUM 	(3000)          // 0 ~ 2999
#define VAR_SYS_NUM 	(10000)         // 40000 ~ 59999
#define VAR_AXIS_NUM 	(100)           // 60000 ~ 69999
#define VAR_TOOL_NUM 	(30000)         // 70000 ~ 99999
#define MODAL_GROUP 80

// var 类型
#define VAR_TYPE_AXIS		(0)			// 轴类型
#define VAR_TYPE_CHANNEL	(1)			// 通道类型
#define VAR_TYPE_SYSTEM		(2)			// 系统类型
#define VAR_TYPE_TOTAL		(3)			// 所有类型

// type
#define INT_TYPE		(0)				// 整数类型 (int i)
#define DOUBLE_TYPE		(1)				// 浮点数类型 (double f)
#define UINT_TYPE		(2)				// 无符号整数类型 (unsigned int n)
#define CHAR_TYPE		(3)				// 字符串类型 (char s[8])
#define PSTR_TYPE		(4)				// 字符串指针类型 (char* pstr)

// 模式
#define	CHAN_MODE_RESET	(0)
#define	CHAN_MODE_AUTO	(1)
#define	CHAN_MODE_JOG	(2)
#define	CHAN_MODE_STEP	(3)
#define	CHAN_MODE_MPG	(4)
#define	CHAN_MODE_HOME	(5)
#define	CHAN_MODE_PMC	(6)
//#ifdef _MCPMDI // MCP面板切换MDI方式
#define CHAN_MODE_MDI	(7)
//#else
#define CHAN_MODE_SBL	(7)

/*!  模式定义 */
typedef enum
{
    MODE_TYPE_EDIT = 0,   /*! 编辑 */
    MODE_TYPE_MEN,        /*! 记忆 */
    MODE_TYPE_MDI,        /*! 手动输入 */
    MODE_TYPE_DNC,        /*! 连线 */
    MODE_TYPE_MPG,        /*! 手轮 */
    MODE_TYPE_JOG,        /*! 寸动 */
    MODE_TYPE_RAPID,      /*! 快速 */
    MODE_TYPE_HOME,       /*! 原点复归 */
    MODE_TYPE_ModeTypeCount
} ModeType;

typedef struct{
    int type;
    union{
        int i;
        double f;
        unsigned int n;
        char s[8];
        char* pstr;
    }value; 
}VarUnion;

#ifdef __cplusplus
}
#endif
#endif // VARDATATYPE_H
