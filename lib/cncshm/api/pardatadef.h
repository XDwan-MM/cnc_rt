#ifndef PARDATADEF_H
#define PARDATADEF_H

#define PARAM_STRING_LEN (64)       /*! string类型长度 */
#define PARAM_BYTE_LEN (8)          /*! type类型长度 */
#define PARAM_STR_LEN (8)           /*! str类型长度 */

/*!
 * @name 参数管理器宏定义 参数类别最大的数量
 */
#define PARAMAN_FILE_NCU_MAX	(2000)
#define PARAMAN_FILE_MAC_MAX	(2000)
#define PARAMAN_FILE_CHAN_MAX	(2000)
#define PARAMAN_FILE_AXIS_MAX	(2000)
#define PARAMAN_FILE_ACMP_MAX	(1000)
#define PARAMAN_FILE_CFG_MAX	(1000)
#define PARAMAN_FILE_TABLE_MAX	(100000)

#define PARA_PROP_COUNT (11)

/*!
 * @name 参数管理各表的子表的最大数量
 */
#define PARAMAN_SUB_CHAN_MAX	(4)
#define PARAMAN_SUB_AXIS_MAX	(6)
#define PARAMAN_SUB_ACMP_MAX	(6)
#define PARAMAN_SUB_CFG_MAX     (6)

/*!
 * @name 参数 ID编号
 * @{
 */
#define NCU_PARAM_ID_BASE	(0)
#define MAC_PARAM_ID_BASE	(10000)
#define CHAN_PARAM_ID_BASE	(40000)
#define AXIS_PARAM_ID_BASE	(100000)
#define COMP_PARAM_ID_BASE	(300000)
#define DEV_PARAM_ID_BASE	(500000)
#define TABLE_PARAM_ID_BASE (700000)  /*! 数据表参数 */

#define NCU_PARAM_ID_NUM	(2000)
#define MAC_PARAM_ID_NUM	(2000)
#define CHAN_PARAM_ID_NUM	(1000)
#define AXIS_PARAM_ID_NUM	(1000)
#define COMP_PARAM_ID_NUM	(1000)
#define DEV_PARAM_ID_NUM	(1000)
#define TABLE_PARAM_ID_NUM  (100000)  /*! 分配给数据表参数的ID数 */
/*!@}*/

/*!
 * @name 参数管理器宏定义 参数类别
 */
#define PARAMAN_FILE_NCU	(0)	/*! NC参数 */
#define PARAMAN_FILE_MAC	(1)	/*! 机床用户参数 */
#define PARAMAN_FILE_CHAN	(2)	/*! 通道参数 */
#define PARAMAN_FILE_AXIS	(3)	/*! 坐标轴参数 */
#define PARAMAN_FILE_ACMP	(4)	/*! 误差补偿参数 */
#define PARAMAN_FILE_CFG	(5)	/*! 设备接口参数 */
#define PARAMAN_FILE_TABLE	(6)	/*! 数据表参数 */
#define PARAMAN_FILE_BOARD	(7)	/*! 主站参数 */

#define PARAMAN_MAX_FILE_LIB		(7)		/*! 参数结构文件最大分类数 */
#define PARAMAN_MAX_PARM_PER_LIB	(1000)	/*! 各类参数最大条目数 */
#define PARAMAN_MAX_PARM_EXTEND     (1000)	/*! 分支扩展参数最大条目数 */
#define PARAMAN_LIB_TITLE_SIZE		(16)	/*! 分类名字符串最大长度 */
#define PARAMAN_REC_TITLE_SIZE		(16)	/*! 子类名字符串最大长度 */
#define PARAMAN_ITEM_NAME_SIZE		(64)	/*! 参数条目字符串最大长度 */


/*!  参数属性类型 */
enum ParaPropType
{
    PARA_PROP_VALUE = 0,    /*!<  参数值 参数结构文件定义 */
    PARA_PROP_MAXVALUE,     /*!<  最大值 参数结构文件定义 */
    PARA_PROP_MINVALUE,     /*!<  最小值 参数结构文件定义 */
    PARA_PROP_DFVALUE,      /*!<  缺省值 参数结构文件定义 */
    PARA_PROP_NAME,         /*!<  名字  STRING */
    PARA_PROP_ACCESS,       /*!<  权限  INT */
    PARA_PROP_ACT,          /*!<  生效方式 INT */
    PARA_PROP_STORE,        /*!<  存储类型  INT */
    PARA_PROP_ID,           /*!<  参数编号 INT */
    PARA_PROP_SIZE,         /*!<  大小 INT */
    PARA_PROP_RELATIONID,   /*!<  扩展参数 */
};

/*!  参数生效方式枚举定义 */
enum ParaActType
{
    PARA_ACT_SAVE, /*!<  保存生效 */
    PARA_ACT_NOW,  /*!<  立即生效 */
    PARA_ACT_RST,  /*!<  复位生效 */
    PARA_ACT_PWR,  /*!<  重启生效 */
    PARA_ACT_HIDE  /*!<  隐藏未启用 */
};

/*!
 * @name 数据类型
 * @{
 */
#define DTYPE_NULL		(0)	/*! 空类型 */
#define DTYPE_INTEGER	(1)	/*! 整型 */
#define DTYPE_INT DTYPE_INTEGER
#define DTYPE_FLOAT		(2)	/*! 实型 */
#define DTYPE_EXPR		(3)	/*! 表达式 */
#define DTYPE_VAR		(4)	/*! 简单变量 */
#define DTYPE_STRING	(5)	/*! 字符串 */
#define DTYPE_UINT		(6) /*! 无符号整形 */
#define DTYPE_BOOL		(7) /*! 布尔型 */
#define DTYPE_FUNC		(8) /*! 函数表达式 */
#define DTYPE_ARR		(9) /*! 数组表达式 */
#define DTYPE_HEX4		(10) /*! 十六进制格式 */
#define DTYPE_BYTE		(11) /*! 十六进制格式 */
#define DTYPE_INTEXP    (12) /*! 表达式整形 */
#define DTYPE_INTOPEXP  (13) /*! 整型运算表达式 */
#define DTYPE_FLOATOPEXP    (14) /*! 浮点型运算表达式 */
#define DTYPE_DOT	DTYPE_FLOAT
#define DTYPE_LONGSTRING	(15)	/*! 长字符串 */



#define TOTAL_AXES_NUM		(SYS_AXES_NUM+SYS_SPDL_NUM) /*! 系统最大逻辑轴数 */
#define SYS_PART_NUM		SYS_NCOBJ_NUM	/*! 系统设备接口数 */
#define CH_GRAPH_EVERY_VIEW_PAR_RESV (3) /*! NC参数中图形参数,每个视角的保留参数数量 */


#endif // PARDATADEF_H
