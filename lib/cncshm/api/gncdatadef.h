/*!
 * @file        gncdatadef.h
 * @brief       基本数据定义:参数、刀具、采样
 * @version     V1.00
 * @date        2025/12/30
 * @author      ghmi Team
 * @copyright   珠海格力电器股份有限公司装备动力技术研究院三所
 */

#ifndef __GNC_DATADEF_H__
#define __GNC_DATADEF_H__

/******** 格力装备动力技术研究院 许晓婷 2025/12/31 新增 ***********/
#define _GNC_CV_5080_
#ifdef _GNC_CV_5080_
#define SYS_CHAN_NUM		(4)		/*! 系统最大通道数 */
#define SYS_AXES_NUM		(6)     /*! 系统最大进给轴数 */
#define SYS_NCBRD_CFG_NUM   (6)		/*! 主控制设备数 */
#define SYS_SPDL_NUM		(1)		/*! 系统最大主轴数 */
#define SYS_NCOBJ_NUM		(80)	/*! 从设备控制对象（部件）数 */
#define CHAN_AXES_NUM		(4)		/*! 通道最大轴数  */
#define CHAN_SPDL_NUM		(1)		/*! 通道最大主轴数  */
#define MAX_SMC_AXES_NUM	(16)	/*! 最多运控轴数  */
#endif

#ifdef _GNC_10_
#define SYS_AXES_NUM 		(3)    	// 三进给轴
#define SYS_SPDL_NUM 		(1)    	// 一主轴
#define SYS_CHAN_NUM 		(1)    	// 单通道
#define SYS_NCOBJ_NUM		(8)		// 从设备控制对象（部件）数
#define CHAN_AXES_NUM		(3)		// 通道最大轴数
#define CHAN_SPDL_NUM		(1)		// 通道最大主轴数
#define MAX_SMC_AXES_NUM	(4)		// 最多运控轴数/
#define SYS_NCBRD_CFG_NUM   (6)		/*! 主控制设备数 */
#endif

#ifdef _GNC_20_
#define SYS_AXES_NUM 		(4)     // 四进给轴
#define SYS_SPDL_NUM 		(2)     // 两主轴
#define SYS_CHAN_NUM 		(2)     // 双通道
#define SYS_NCOBJ_NUM		(80)	// 从设备控制对象（部件）数
#define CHAN_AXES_NUM		(3)		// 通道最大轴数
#define CHAN_SPDL_NUM		(1)		// 通道最大主轴数
#define MAX_SMC_AXES_NUM	(4)		// 最多运控轴数
#endif

#ifdef _GNC_30_
#define SYS_AXES_NUM 		(5)     // 五进给轴
#define SYS_SPDL_NUM 		(2)     // 两主轴
#define SYS_CHAN_NUM 		(2)     // 双通道
#define SYS_NCOBJ_NUM		(80)	// 从设备控制对象（部件）数
#define CHAN_AXES_NUM		(3)		// 通道最大轴数
#define CHAN_SPDL_NUM		(1)		// 通道最大主轴数
#define MAX_SMC_AXES_NUM	(4)		// 最多运控轴数
#endif

#ifdef _GNC_80_
#define SYS_CHAN_NUM		(4)		/*! 系统最大通道数 */
#define SYS_AXES_NUM		(24)	/*! 系统最大进给轴数 */
#define SYS_SPDL_NUM		(8)		/*! 系统最大主轴数 */
#define SYS_NCBRD_NUM		(4)		/*! 主控制设备数 */
#define SYS_NCOBJ_NUM		(80)	/*! 从设备控制对象（部件）数 */
#define CHAN_AXES_NUM		(9)		/*! 通道最大轴数  */
#define CHAN_SPDL_NUM		(4)		/*! 通道最大主轴数  */
#define MAX_SMC_AXES_NUM	(16)	/*! 最多运控轴数  */
#endif

/******** 格力装备动力技术研究院 许晓婷 2025/12/31 新增 ***********/

// #ifdef _HNC_80_
// 	#define SYS_CHAN_NUM	8
// 	#define SYS_AXES_NUM	64
// 	#define SYS_SPDL_NUM	16 /*! 主轴数 */
// 	#define SYS_NCBRD_NUM	8 /*! 主控制设备数 */
// 	#define SYS_NCOBJ_NUM	120 /*! 从设备控制对象（部件）数【轴数+RS232+LAN+USB+MCP+NCKB+I/O】*/
// 	#define CHAN_AXES_NUM	9
// 	#define CHAN_SPDL_NUM   4	/*! 通道最多主轴数 */
// 	#define MAX_SMC_AXES_NUM 32	/*! 最多运控轴数 */
// #else

// 	#define SYS_CHAN_NUM		(4)		/*! 系统最大通道数 */
// 	#define SYS_AXES_NUM		(24)	/*! 系统最大进给轴数 */
// 	#define SYS_SPDL_NUM		(8)		/*! 系统最大主轴数 */
// 	#define SYS_NCBRD_NUM		(4)		/*! 主控制设备数 */
// 	#define SYS_NCOBJ_NUM		(80)	/*! 从设备控制对象（部件）数 */
// 	#define CHAN_AXES_NUM		(9)		/*! 通道最大轴数  */
// 	#define CHAN_SPDL_NUM		(4)		/*! 通道最大主轴数  */
// 	#define MAX_SMC_AXES_NUM	(16)	/*! 最多运控轴数  */
// #endif

#define TOTAL_AXES_NUM		(SYS_AXES_NUM+SYS_SPDL_NUM) /*! 系统最大逻辑轴数 */
#define SYS_PART_NUM		SYS_NCOBJ_NUM	/*! 系统设备接口数 */
#define CH_GRAPH_EVERY_VIEW_PAR_RESV (3) /*! NC参数中图形参数,每个视角的保留参数数量 */

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
 * @{
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
/*!@}*/

//#define BASE_LEN_RESOL (100000.0)
//#define RESOL_FACTOR (_hnc_get_ncu_int(PAR_NCU_LEN_RESOL)/BASE_LEN_RESOL)


/*!
 * @name 参数相对基地址的偏移
 * @{
 */

/*! NC参数 */
enum
{
    PAR_NCU_TYPE,			/*!< NCU控制器类型 */
    PAR_NCU_CYCLE,			/*!< 插补周期 */
    PAR_NCU_PLC2_CMDN,		/*!< PLC2周期执行语句数 */

    PAR_NCU_ANG_RESOL = 5,	/*!< 角度计算分辨率 */
    PAR_NCU_LEN_RESOL,		/*!< 长度计算分辨率 */
    PAR_NCU_TIME_RESOL,		/*!< 时间编程分辨率 */
    PAR_NCU_VEL_RESOL,		/*!< 线速度编程分辨率 */
    PAR_NCU_SPDL_RESOL,		/*!< 角速度编程分辨率 */
    PAR_NCU_ARC_PROFILE,	/*!< 圆弧插补轮廓允许误差 */
    PAR_NCU_MAX_RAD_ERR,	/*!< 圆弧编程端点半径允许偏差 */
    PAR_NCU_G43_SW_MODE,	/*!< 刀具轴选择方式[0,固定z向;1,G17/18/19切换;2,G43指令轴切换] */
    PAR_NCU_G41_G00_G01,	/*!< G00插补使能 */
    PAR_NCU_G53_LEN_BACK,	/*!< G53之后自动恢复刀具长度补偿[0,不恢复 1 恢复] */
    PAR_NCU_CRDS_NUM,	    /*!< 允许联动轴数 */
    PAR_NCU_LAN_EN,			/*!< 局域网使能 */
    PAR_NCU_POWER_SAFE,		/*!< 断电保护使能 */
    PAR_NCU_TIME_EN,		/*!< 系统时间显示使能 */
    PAR_NCU_PSW_CHECK,		/*!< 权限检查使能 */
    PAR_NCU_ALARM_POP,		/*!< 报警窗口自动显示使能 */
    PAR_NCU_KBPLC_EN,       /*!< 键盘PLC使能 */
    PAR_NCU_PREVIEW_EN,  	/*!< 图形预览使能	*/
    PAR_NCU_FSPD_DISP,		/*!< F进给速度显示方式 */
    PAR_NCU_GLNO_DISP,		/*!< G代码行号显示方式 */
    PAR_NCU_INCH_DISP,	    /*!< 公制/英制选择 */
    PAR_NCU_DISP_DIGITS,	/*!< 位置小数点后显示位数 */
    PAR_NCU_FEED_DIGITS,	/*!< 速度小数点后显示位数 */
    PAR_NCU_SPINDLE_DIGITS,	/*!< 转速小数点后显示位数 */
    PAR_NCU_LANGUAGE,		/*!< 语言选择 */
    PAR_NCU_LCD_TIME,		/*!< 进入屏保等待时间 */
    PAR_NCU_DISK_TYPE,		/*!< 外置程序存储类型 */
    PAR_NCU_REFRE_INTERV,   /*!< 界面刷新间隔时间 */
    PAR_NCU_SAVE_TYPE,      /*!< 是否外接UPS */
    PAR_NCU_OPERATE_NOTE,   /*!< 操作提示使能[0位：重运行 1位：Tool->相对实际 2位：Tool->当前位置] */
    PAR_NCU_SERVER_NAME,
    PAR_NCU_SERVER_IP1,
    PAR_NCU_SERVER_IP2,
    PAR_NCU_SERVER_IP3,
    PAR_NCU_SERVER_IP4,
    PAR_NCU_SERVER_PORT,    /*!< 服务器端口号 */
    PAR_NCU_SERVER_LOGIN,   /*!< 服务器访问用户名1 */
    PAR_NCU_SERVER_PASSWD,  /*!< 服务器访问密码 */
    PAR_NCU_NET_DISK_DISCON_TIMEOUT,  /*!< 网络断开判断阈值ms */
    PAR_NCU_NET_TYPE = 44,       /*!< 网盘映射类型 */
    PAR_NCU_IP1,            /*!< IP地址段1 */
    PAR_NCU_IP2,            /*!< IP地址段2 */
    PAR_NCU_IP3,            /*!< IP地址段3 */
    PAR_NCU_IP4,            /*!< IP地址段4 */
    PAR_NCU_PORT,           /*!< 本地端口号 */
    PAR_NCU_NET_START,		/*!< 是否开启网络 */
    PAR_NCU_SERIAL_TYPE,	/*!< 串口类型 */
    PAR_NCU_SERIAL_NO = 52,	/*!< 串口号 */
    PAR_NCU_DATA_LEN,       /*!< 收发数据长度 */
    PAR_NCU_STOP_BIT,       /*!< 停止位 */
    PAR_NCU_VERIFY_BIT,     /*!< 校验位 */
    PAR_NCU_BAUD_RATE,      /*!< 波特率 */
    PAR_NCU_IP_TYPE,        /*!< 静态IP/动态IP */
    PAR_NCU_ARCS_INTERSECT, /*!< 圆弧交点允差 */
    PAR_NCU_ARCS_ONE_CYCLE, /*!< 两段圆弧的圆心距离小于这个值时，认为两圆同心 */
    PAR_NCU_TOOL_NUM = 60,	/*!< 最大刀具数 */
    PAR_NCU_TOFF_DIGIT,     /*!< 刀补有效位数 */
    PAR_NCU_MAGZ_NUM,		/*!< 最大刀库数 */
    PAR_NCU_TOOL_LOCATION,	/*!< 最大刀座数 */
    PAR_NCU_TABRA_ADD_EN,   /*!< 刀具磨损累加使能 */
    PAR_NCU_TDIA_SHOW_EN,   /*!< 车刀直径显示使能 */

    //PAR_NCU_HALF_CYCLE_TOL, /*!< 半圆圆心理论与实际的偏差允许值(mm) */
    //PAR_NCU_SMOOTH_SIZE,    /*!< 指令平滑缓冲区大小 */ //wuxuanhui 2019/4/9 此参数作废
    //PAR_NCU_CYCLE_OPT,		/*!< 复合循环路径选项【0x00FF: 0 常规  1 退刀段效率优先 2 FANUC兼容  &0xFF00 = =  0x0000 : 45度退刀 0x0100: 径向退刀 */
    /*!< &0x0200 = =  0x0200时最后一刀直接退到循环起点，凹槽中有台阶时不要用此选项】 （此参数弃用）*/
    PAR_NCU_HOLD_DECODE_EN = 69,	/*!< 进给保持后重新解释使能 */
    PAR_NCU_G28_LEN_BACK,	/*!< G28后是否自动恢复刀长补 */
    PAR_NCU_SPEEDUP_EN,     /*!< 解释器周期最大解释段数 */
    PAR_NCU_CUTTIME_EN,		/*!< 加工时间显示使能 */

    PAR_NCU_FOLLOW_TIME,	/*!< 跟踪误差滞后周期 */

    PAR_NCU_SELF_EN = 74,		/*!< 健康保障模块使能,		1:健康保障模块生效，0:关闭   pancl */
    PAR_NCU_SELF_CHE_EN = 75,  /*!< 健康保障监测刀库使能,	1:监测刀库，0:不监测 */
    PAR_NCU_EDIT_AUTO_SAVE,		/*!< G代码编辑后，返回自动保存*/
    PAR_NCU_FAST_VERIFY_STOP_TIME = 77 , /*!< 图形校验最大运行时间(秒) */
    PAR_NCU_LOG_IN_CHECK, /*!< 是否开启开机密码 */
    PAR_NCU_NET_CONNECT_TYPE, /*!< 联网设置 */

    PAR_NCU_LOG_SAVE_TYPE = 80, /*!< 日志文件保存类型 */

    PAR_NCU_INTERNET_IP1,	/*!< 网络平台服务器IP */
    PAR_NCU_INTERNET_IP2,
    PAR_NCU_INTERNET_IP3,
    PAR_NCU_INTERNET_IP4,
    PAR_NCU_INTERNET_PORT,	/*!< 网络平台服务器端口 */
    PAR_NCU_GATEWAY1,		/*!< 本机网关 */
    PAR_NCU_GATEWAY2,
    PAR_NCU_GATEWAY3,
    PAR_NCU_GATEWAY4,
    PAR_NCU_CLOUD_TRIGGER,	//云数控上传开关

    PAR_NCU_SUBNET_MASK1 = 91,	// 子网掩码
    PAR_NCU_SUBNET_MASK2,
    PAR_NCU_SUBNET_MASK3,
    PAR_NCU_SUBNET_MASK4,

    PAR_NCU_CLOUD_COMM_TYPE,	//云数控通讯方式 0：http 1：MQTT

    PAR_NCU_SERVER_LOGIN_2 = 97, /*!< 服务器访问用户名2 */
    PAR_NCU_TOOL_WEAR_RANGE,/*!< 车床刀具磨损允许输入范围(mm) */
    PAR_NCU_TURN_SIMU_CLOSE_WORK_SHOW, /*!< 车床图形关闭毛坯显示使能 */

    PAR_NCU_HMI = 100,					/*!< 界面设置参数基地址 */
    PAR_NCU_ISSU_EDITION = PAR_NCU_HMI,	/*!< 发布版本号 */
    PAR_NCU_TEST_EDITION,				/*!< 测试版本号 */
    PAR_NCU_SHOW_LIST,					/*!< 示值列，40 */
    PAR_NCU_POS_INDEX = PAR_NCU_SHOW_LIST + 7,           /*!< 加工界面显示坐标序号，与WgMainDisp头文件定义的枚举相同*/
    PAR_NCU_DISPCFG_NO_H = 110,  /*!< 显示配置界面 操作工权限    显示配置标志高32位(备用) */
    PAR_NCU_DISPCFG_NO_L,        /*!< 显示配置界面 操作工权限    显示配置标志低32位(备用) */
    PAR_NCU_DISPCFG_USER_H,      /*!< 显示配置界面 车间管理员权限 显示配置标志高32位 */
    PAR_NCU_DISPCFG_USER_L,      /*!< 显示配置界面 车间管理员权限 显示配置标志低32位 */
    PAR_NCU_DISPCFG_MAC_H,       /*!< 显示配置界面 机床厂家权限   显示配置标志高32位 */
    PAR_NCU_DISPCFG_MAC_L,       /*!< 显示配置界面 机床厂家权限   显示配置标志低32位 */
    PAR_NCU_DISPCFG_NC_H,        /*!< 显示配置界面 数控厂家权限   显示配置标志高32位 */
    PAR_NCU_DISPCFG_NC_L,        /*!< 显示配置界面 数控厂家权限   显示配置标志低32位 */
    PAR_NCU_DISPCFG_RD_H,        /*!< 显示配置界面 系统管理员权限 显示配置标志高32位, 存放低一级权限允许显示的参数 */
    PAR_NCU_DISPCFG_RD_L,        /*!< 显示配置界面 系统管理员权限 显示配置标志低32位, 存放低一级权限允许显示的参数 */

    PAR_NCU_HMI_REFRESH_TIME = 120,  /*!< 界面刷新周期 */

    PAR_NCU_INTP_TYPE = 200,		/*!< 插补类型 */
    PAR_NCU_1_CYC,			/*!< 1阶平滑周期 */
    PAR_NCU_2_CYC,			/*!< 2阶平滑周期 */
    PAR_NCU_RECTIFY_TYPE,	/*!< 指令平滑类型 */

    PAR_NCU_FEEDCHECK_SAMPLING_NUM = 250,   /*!<速度校验器离散计算点数 */

    PAR_NCU_PROG_PATH = 300,	/*!< 加工代码程序路径 */
    PAR_NCU_PLC_PATH,		/*!< PLC程序路径 */
    PAR_NCU_PLC_NAME,		/*!< PLC程序名 */
    PAR_NCU_DRV_PATH,		/*!< 驱动程序路径 */
    PAR_NCU_DRV_NAME,		/*!< 驱动程序名 */
    PAR_NCU_PARA_PATH,		/*!< 参数文件路径 */
    PAR_NCU_PARA_NAME,		/*!< 参数文件名 */
    PAR_NCU_SIMU_PATH,		/*!< 仿真配置文件路径 */
    PAR_NCU_SIMU_NAME,		/*!< 仿真配置文件名 */
    PAR_NCU_DLGP_PATH,		/*!< 对话编程配置文件路径 */
    PAR_NCU_DLGP_NAME,		/*!< 对话编程配置文件名 */
    PAR_NCU_VIDEO_DEV,      /*!< 视频外设驱动 */

    PAR_NCU_SUB_DBG = 320,	/*!< 宏程序单段使能【WIN】 */
    PAR_NCU_USER_LOGIN = 321,	/*!< 是否开启用户登录? */
    PAR_NCU_GRIND_MODE = 348,   /*!< 上电初始化界面 */
    PAR_NCU_TRIANGEL_FUNC_CTRL = 349,      /*!< 三角函数控制 */
    PAR_NCU_G16_OPT = 350,	/*!< G16的极点定义模式选择 0：FANUC模式 1：HNC-8模式 */
    PAR_NCU_GEDIT_FRAME = 351,	/*!< 编辑界面框架选择 0：HNC-8模式 1：宁江专机模式 */
    PAR_NCU_FTP_MODE = 352,	/*!< FTP连接方式 0：HNC-8模式 1：沈飞模式 */
    PAR_NCU_5AXIS_ENABLE = 353,/*!< 是否开启五轴功能应用 0:不开启 1:开启 */
    PAR_NCU_HMI_TYPE,		/*!< HMI类型，0：标准8型；1：沈飞专用； */
    PAR_NCU_CHECK_SYSDISK_FREE,	/*!< 系统盘剩余空间不足提示检测阈值 */
    PAR_NCU_MILL_FUN_TYPE,	/*!< 铣床功能类型 */
    PAR_NCU_API_LOG_LEVEL,    /*!< api日志记录级别 */
    PAR_NCU_EXIT_MDI_CLEAR,		/*!< 退出mdi时是否清空mdi内容 */
    PAR_NCU_DEFAULT_RIGHT_LEVEL = 359, /*!< 上电默认用户权限 */

    PAR_NCU_INSERT_SPACE_CLOSE = 361,	/*!< 程序显示分词功能关闭 */
    PAR_NCU_POS_SHOW_2COL,	/*!< 主界面坐标显示2列 */
    PAR_NCU_PROG_SMALL_CHARACTER,	/*!< 程序显示小字符 */
    PAR_NCU_RANDOM_RESTART,	/*!< 开启一键偏置功能 */

    PAR_NCU_MAC_TYPE = 368, /*!< 机床类型 */
    PAR_NCU_TOOL_MAG_TYPE, /*!< 刀库类型 */
    PAR_NCU_COMP_ENABLE, /*!< 智能化功能开关 */
    PAR_NCU_MDI_TYPE, /*!< 切换MDI方式 */
    PAR_NCU_TOOL_HEAD_NUM = 372, /*!< 刀沿数目 */
    PAR_NCU_SAMPLE_ST_M = 373, /*!< 伺服调整采样开始M代码 */
    PAR_NCU_SAMPLE_END_M = 374, /*!< 伺服调整采样结束M代码 */
    PAR_NCU_WORKMEAS_AUTO,  /*!< 工件测量自动测量开启 */
    PAR_NCU_HMISHOW = 376,  /*hmi显示参数*/
    PAR_NCU_TEMPERATURE_SENSOR,			/*是否有温度传感器*/
    PAR_NCU_POS_INDEX_2 = 378,       /*!< 车床主页面显示行1 */
    PAR_NCU_POS_INDEX_3 = 379,		 /*!< 车床主页面显示行2 */

    PAR_NCU_COM2_SERIAL_TYPE = 380,	/*!< 串口应用2:串口类型 */
    PAR_NCU_COM2_SERIAL_NO,			/*!< 串口应用2:串口号 */
    PAR_NCU_COM2_DATA_LEN,       	/*!< 串口应用2:收发数据长度 */
    PAR_NCU_COM2_STOP_BIT,       	/*!< 串口应用2:停止位 */
    PAR_NCU_COM2_VERIFY_BIT,     	/*!< 串口应用2:校验位 */
    PAR_NCU_COM2_BAUD_RATE,      	/*!< 串口应用2:波特率 */
    PAR_NCU_COM3_SERIAL_TYPE = 386,	/*!< 串口应用3:串口类型 */
    PAR_NCU_COM3_SERIAL_NO,			/*!< 串口应用3:串口号 */
    PAR_NCU_COM3_DATA_LEN,       	/*!< 串口应用3:收发数据长度 */
    PAR_NCU_COM3_STOP_BIT,       	/*!< 串口应用3:停止位 */
    PAR_NCU_COM3_VERIFY_BIT,     	/*!< 串口应用3:校验位 */
    PAR_NCU_COM3_BAUD_RATE,      	/*!< 串口应用3:波特率 */
    PAR_NCU_COM4_SERIAL_TYPE = 392,	/*!< 串口应用4:串口类型 */
    PAR_NCU_COM4_SERIAL_NO,			/*!< 串口应用4:串口号 */
    PAR_NCU_COM4_DATA_LEN,       	/*!< 串口应用4:收发数据长度 */
    PAR_NCU_COM4_STOP_BIT,       	/*!< 串口应用4:停止位 */
    PAR_NCU_COM4_VERIFY_BIT,     	/*!< 串口应用4:校验位 */
    PAR_NCU_COM4_BAUD_RATE,      	/*!< 串口应用4:波特率 */
    PAR_NCU_COM5_SERIAL_TYPE = 398,	/*!< 串口应用5:串口类型 */
    PAR_NCU_COM5_SERIAL_NO,			/*!< 串口应用5:串口号 */
    PAR_NCU_COM5_DATA_LEN,       	/*!< 串口应用5:收发数据长度 */
    PAR_NCU_COM5_STOP_BIT,       	/*!< 串口应用5:停止位 */
    PAR_NCU_COM5_VERIFY_BIT,     	/*!< 串口应用5:校验位 */
    PAR_NCU_COM5_BAUD_RATE,      	/*!< 串口应用5:波特率 */

    PAR_NCU_CHECK_NEW_PACKAGE = 404, /*!< 是否在启动时检查新版本 */
    PAR_NCU_CLOUD_NETCARD_NO = 405, /*!< 云通讯网络网卡序号 */
    PAR_NCU_MDI_ENSURE = 406, /*!< MDI输入是否需要确认 */
    PAR_NCU_HOLD_EDIT_ENSURE = 407, /*!< 车床单段后重解释功能 */
    PAR_NCU_FS_SHOWTYPE = 408,     /*!< FS显示模式 */

    PAR_NCU_BUS_ALARM_MASK = 409,  /*!< 总线报警掩码 */

    PAR_NCU_INPUT_UM_TYPE = 410,/*!< UM方式输入 */
    PAR_NCU_RST_CANCLE_DLG = 411,   /*!< 复位取消弹框 */
    PAR_NCU_SWITCH_POS_DISPLAY,/*!< 2.40占位 坐标菜单切换主界面坐标显示 */

    PAR_NCU_ETH1_IP1 = 413,            /*!< 网口1IP地址段 */
    PAR_NCU_ETH1_IP2,
    PAR_NCU_ETH1_IP3,
    PAR_NCU_ETH1_IP4,
    PAR_NCU_ETH1_GATEWAY1,		/*!< 网口1默认网关 */
    PAR_NCU_ETH1_GATEWAY2,
    PAR_NCU_ETH1_GATEWAY3,
    PAR_NCU_ETH1_GATEWAY4,
    PAR_NCU_ETH1_SUBNET_MASK1,	/*!< 网口1子网掩码 */
    PAR_NCU_ETH1_SUBNET_MASK2,
    PAR_NCU_ETH1_SUBNET_MASK3,
    PAR_NCU_ETH1_SUBNET_MASK4,
    PAR_NCU_ETH1_PORT,           /*!< 网口1端口号 */
    PAR_NCU_ETH1_IP_TYPE,        /*!< 网口1静态IP/动态IP */
    PAR_NCU_ULTRASOUND = 427,    /*!< 超声波功能 */
    PAR_NCU_TYPE_OF_TOUCH_POS = 428, /*!< 测量点位置的类型：指定通道#1360输出普通IO锁存位置或是高速IO锁存位置 */
    PAR_NCU_OPEN_DCM = 429, /*!< 开启防碰撞功能 */
    PAR_NCU_INC_OFFSET_ENABLE,		 /*!< 系统指令增量缓存使能 */
    PAR_NCU_RESOL_FILT_THRESHOLD,	 /*!< 指令速度分辨率级滤波阈值 */

    PAR_NCU_CLOSE_REAL_GET_HEAT_COMP=432,  /*<关闭热变形量实时计算*/
    PAR_NCU_CLOSE_HEAT_COMP,             /*<关闭热误差补偿功能*/
    PAR_NCU_AUTO_LOGOUT_ACCESS_TIME, /*!< 自动注销权限时间 */

    PAR_NCU_BUS_RESTART_MASK,	/*!< 435  总线重连功能是否开启 */
    PAR_NCU_G_PRINT_TYPE,		 /*!< G代码打印类型,0:关闭功能,1:只进行打印文件输出,2:文件输出&弹框限时显示,3:文件输出&不限时弹框 */
    PAR_NCU_G_PRINT_TIME,        /*!< G代码打印类型为2时,弹框留存时间(s) */
    PAR_NCU_FOREIGN_LANGUAGE_SHOW = 438,		/*!< 系统支持语言（默认支持中文） */
    PAR_NCU_DIAMETER_DISPLAY = 439,	/*!< 坐标模块显示直径标识 */
    PAR_NCU_TURRET_DISPLAY,         /*!< 车床刀偏界面显示主轴背刀 */
    PAR_NCU_FOCUS_CURTOOL,          /*!< 车床刀偏界面进入刀偏表光标自动定位当前刀偏号 */
    PAR_NCU_OFFSET_CONFIRM,         /*!< 车床刀偏界面刀偏磨损输入值确认 */
    PAR_NCU_MAINWG_SHOW_TYPE = 443, /*!< 加工界面显示:VIEW_POS(位置视图),VIEW_ACTWCS(绝对坐标), VIEW_ACTRCS(相对坐标)*/
    PAR_NCU_MIXPOS_COL_1,           /*!< 车床综合视图列1 */
    PAR_NCU_MIXPOS_COL_2,           /*!< 车床综合视图列2 */
    PAR_NCU_MIXPOS_COL_3,           /*!< 车床综合视图列3 */
    PAR_NCU_PARAM_CHECK,			/*!< 是否进行参数一致性检查*/
    PAR_NCU_EDITSHOW,				/*!< 编辑程序显示参数 */

    PAR_NCU_LOADCURRENT_WARNING = 449,  /*!<负载电流警告 */
    PAR_NCU_LOADCURRENT_ALARM = 450,    /*!<负载电流报警 */
    PAR_NCU_TURNTOOL_INPUT_TYPE = 451,    /*!< 车床刀偏磨损输入方式 0：华中方式 1：广数方式（2.40.09占位） */

    PAR_NCU_TOTAL = 2000		/*!< NC参数总数 */
};



/*! 机床用户参数 */
enum
{
    PAR_MAC_CHAN_NUM,	/*!< 通道数：1~SYS_CHAN_NUM */
    PAR_MAC_CHAN_TYPE,	/*!< 机床通道加工类型【车、铣床、车铣复合】 占用8个参数*/
    PAR_MAC_CHAN_FLAG = PAR_MAC_CHAN_TYPE+8,	/*!< 通道选择标志 */
    PAR_MAC_CHAN_AX_FLAG = PAR_MAC_CHAN_FLAG+8,	/*!< 通道的轴显示标志，每个通道占用两个参数，共8*2 = 16个参数 */
    PAR_MAC_SHOW_AXES = 41,		/*!< 是否动态显示坐标轴 */
    PAR_MAC_CALIB_TYPE,			/*!< 刀具测量仪类型 */
    PAR_MAC_ALARM_PAUSE,		/*!< 开机自检警告暂停功能 */
    PAR_MAC_CC_CW_SPD_TYPE,		/*!< 半径补偿圆弧速度处理策略 */
    PAR_MAC_CC_WEAR_TYPE,       /*!< 半径补偿=半径减/加磨损 */
    PAR_MAC_CC_CHECK_CTRL,      /*!< 半径补偿干涉控制，0：干涉报警 1：自动处理干涉 */
    PAR_MAC_CC_CHECK_NUM,         /*!< 半径补偿干涉检查段数 */

    PAR_MAC_HARD_RESET_DELAY = 48,	/*!< 硬复位延时 */
    PAR_MAC_AXES_NUM,			/*!< 机床总轴数 */
    PAR_MAC_SMX_AXIS_NUM,		/*!< 运动控制通道轴数（耦合从轴+PMC轴） */
    PAR_MAC_SMX_AXIS_IDX,		/*!< PMC及耦合从轴编号，预留32个 */

    PAR_MAC_CYC_TYPE=83,      /*!< 钻攻中心固定循环类型 */
    PAR_MAC_T_CTRL_TYPE=89,         /*!< T指令控制方式 */
    PAR_MAC_CHIP_BREAK_ACC,		/*!<断屑加速度检查*/
    PAR_MAC_USR_NEW_USR_VAR=91,      /*!< 兼容FANUC,三菱，#500~999断电保存。0，关闭断电保存功能；1，打开 */
    PAR_MAC_SP_POS_HOLD,		/*!< C轴为速度模式时不刷新坐标 */
    PAR_MAC_PRE_RUN_MASK,             /*!< 按循环启动是否运行预加载程序 */
    PAR_MAC_HOME_CHECK,			/*!< 轴回零完成后进行位置检测，位置偏差较大时提示 */
    PAR_MAC_TRUSS_CH,           /*!< 2.40占位 桁架通道号 */
    PAR_MAC_PWR_TIME_RECORD = 96,          /*!<断电时间记录诊断功能 */
    PAR_MAC_G02_LOSE_PARM = 98, 			/*!< G02/G03缺参数时是否转成G01 */

    PAR_MAC_BIG_TOOL_SWITCH,	/*!< 大小刀功能开关 */

    PAR_MAC_SPDL_TYPE=100,           //排钻机床主轴类型
    PAR_MAC_PAIZUAN_START_NO = 101,	//排钻起始刀具号
    PAR_MAC_PAIZUAN_TOOL_NUM = 102, //排钻刀具总数
    PAR_MAC_NEW_FUCTION_TEST = 103,   /*!< 车削中心新加功能调试参数 */
    PAR_MAC_NEW_FUCTION_TEST2,   /*!< 新加功能调试参数 */
    PAR_MAC_TOOLLIFE_TACTICS = 105,	/*!< 刀具寿命报警策略 */
    PAR_MAC_TOOL = 106,	/*!< 刀具处理参数预留 */
    PAR_MAC_HOME_ONLY_ONE = 107,    /*!< 2.40占位 正负正回零只在第一次找Z脉冲 */
    PAR_MAC_CYCLE_MAX_RECTIFY_NUM = 108,    /*!< 固定循环最大平滑周期数 */
    PAR_MAC_FZONE_IN_MASK=110,	/*!< 机床保护区内部禁止掩码 */
    PAR_MAC_FZONE_EX_MASK,		/*!< 机床保护区外部禁止掩码 */
    PAR_MAC_FZONE_BND,		/*!< 机床保护区边界-x +x -y +y -z +z 6*6=36 */

    PAR_MAC_TOOL_INTER_ENABLE=148,
    PAR_MAC_TOOL_INTER_X_LEN,//刀具干涉原点距离
    PAR_MAC_TOOL_INTER_X_DIR,//刀具干涉两轴移动方向
    PAR_MAC_TOOL_INTER_Y_LEN,
    PAR_MAC_TOOL_INTER_Y_DIR,
    PAR_MAC_TOOL_INTER_Z_LEN,
    PAR_MAC_TOOL_INTER_Z_DIR,

    PAR_GCODE_FANUC_G70_RETURN = 159,   /*!<返回循环起点的方式>*/

    PAR_G95_F_DISP =160,         /*!<转进给F速度显示*/
    PAR_GCODE_FANUC_ERROR =161,  /*!<复合循环误差范围*/
    PAR_GCODE_FANUC_HIGH = 162,   /*!<复合循环粗加工凹槽不考虑最低点*/
    PAR_MAC_2CH_SYNC_TYPE = 163,   /*!<双通道同步模式*/
    PAR_GCODE_FANUC =164,		/*!< FANUC类型指令支持 */
    PAR_MAC_HOME_DWELL = 165,	/*!< 回参考点延时时间，单位：ms */
    PAR_MAC_PLCACK_CYCLE,		/*!< PLC应答最长时间 */
    PAR_MAC_SYNC_M_CODE_ACK_CYCLE,		/*!< 同步M代码等待应答最长时间 */

    PAR_MAC_G64_CORNER_CHK = 169,		/*!< G64拐角准停校验检查使能 */

    PAR_MAC_MCODE_TO_G = 170,	/*!< M代码属性表 */

    PAR_MAC_3D_CC_MAX_ERR = 218,   /* 3D半径补偿最大允许误差 */
    PAR_MAC_3D_CC_CHECK_SWITCH,      /*3D半径补偿干涉检查开关*/
    PAR_MAC_CHAN_GMODE_SHOW = 220,	/*!< 模态G指令显示定制，每个工位占用3个参数，共8*3 = 24个参数 */

    PAR_MAC_SYS_OCCUPIED_PROG_PATH = 244,	/*!< 系统占用程序存储路径 */

    PAR_MAC_MEAS_SPD = 250,	/*!< 测量速度 */
    PAR_MAC_MEAS_DIST,		/*!< 测量最小行程 */
    PAR_MAC_DECODE_CTRL_SEARCH_LINE = 252, /*!< G代码流程控制搜索行数 */
    PAR_MAC_VERY_ONOFF = 253,		/*!< 校验开关 */
    PAR_MAC_GCODEMAN_EDIT_ONOFF = 254,		/*!< 程序管理激活编辑 */
    PAR_MAC_HOME_BLOCK,             /*!< 机床是否安装回零挡块 */
    PAR_MAC_USR_VAR_WAIT,           /*!< 用户自定义宏变量是否执行G08等待 */
    //PAR_MAC_TINY_LEN_CLEAR,         /*!< 极小长度过滤阈值 *///(参数作废)
    //PAR_MAC_TINY_INC_CLEAR = 258,			/*!< 极小增量过滤阈值 *///(参数作废)
    PAR_MAC_HANDLE_BACK_SUBPROG_FBD =259,  /*!<手轮回退子程序禁止使能 */
    PAR_MAC_IPSYNC_FUN = 260,	/*!< 插补同步函数注册 */
    PAR_MAC_G32_HOLD_DX,		/*!< 螺纹加工中止的退刀距离 */
    PAR_MAC_G32_HOLD_ANG,		/*!< 螺纹加工中止的退刀角度 */

    PAR_MAC_SPECIAL = 270,		/*!< 专机预留参数起始地址 */

    PAR_MAC_CHECK_ENCRYPT = 298,	/*!< 是否检查文件加密属性 */
    PAR_MAC_PROG_SKEY = 299,		/*!< G代码文件密钥 */

    PAR_MAC_USER = 300,	/*!< 用户参数基地址 */

    PAR_MAC_USER_PROG_NO_BASE = 500,	/*!<系统已占用程序号参数基地址 */

    PAR_MAC_TOTAL = 2000	/*!< 机床用户参数总数 */
};


/*! 通道参数 */
enum
{
    PAR_CH_NAME = 0,	/*!< 名称 */
    PAR_CH_XINDEX,		/*!< X轴编号 */
    PAR_CH_YINDEX,		/*!< Y轴编号 */
    PAR_CH_ZINDEX,		/*!< Z轴编号 */
    PAR_CH_AINDEX,		/*!< A轴编号 */
    PAR_CH_BINDEX,		/*!< B轴编号 */
    PAR_CH_CINDEX,		/*!< C轴编号 */
    PAR_CH_UINDEX,		/*!< U轴编号 */
    PAR_CH_VINDEX,		/*!< V轴编号 */
    PAR_CH_WINDEX,		/*!< W轴编号 */
    PAR_CH_SPDL0,		/*!< 主轴0编号 */
    PAR_CH_SPDL1,		/*!< 主轴1编号 */
    PAR_CH_SPDL2,		/*!< 主轴2编号 */
    PAR_CH_SPDL3,		/*!< 主轴3编号 */
    PAR_CH_X_NAME,		/*!< X轴名 */
    PAR_CH_Y_NAME,		/*!< Y轴名 */
    PAR_CH_Z_NAME,		/*!< Z轴名 */
    PAR_CH_A_NAME,		/*!< A轴名 */
    PAR_CH_B_NAME,		/*!< B轴名 */
    PAR_CH_C_NAME,		/*!< C轴名 */
    PAR_CH_U_NAME,		/*!< U轴名 */
    PAR_CH_V_NAME,		/*!< V轴名 */
    PAR_CH_W_NAME,		/*!< W轴名 */
    PAR_CH_S0_NAME,		/*!< 主轴0名 */
    PAR_CH_S1_NAME,		/*!< 主轴1名 */
    PAR_CH_S2_NAME,		/*!< 主轴2名 */
    PAR_CH_S3_NAME,		/*!< 主轴3名 */
    PAR_CH_SVEL_SHOW,	/*!< 主轴转速显示方式 */
    PAR_CH_S_SHOW,		/*!< 主轴显示定制 */
    PAR_CH_STOP_DELAY,	/*!<急停后增量延时周期 */

    PAR_CH_DEFAULT_F = 30,	/*!< 通道的缺省进给速度 */
    PAR_CH_DRYRUN_SPD,		/*!< 空运行进给速度 */
    PAR_CH_DIAPROG,			/*!< 直径编程使能 */
    PAR_CH_UVW_INC_EN,		/*!< UVW增量编程使能 */
    PAR_CH_CHAMFER_EN,		/*!< 倒角使能 */
    PAR_CH_ANGLEP_EN,		/*!< 角度编程使能*/
    PAR_CH_CYCLE_OPTION,	/*!< 复合循环选项屏蔽字[位]：0x0001 粗加工圆弧转直线 0x0002：凹槽轴向余量报警屏蔽 0x0004: 无精加工 */
    PAR_CH_MPG_ACC_RATE,	/*!< 手摇加速度系数 */
    PAR_CH_MPG_JK_RATE,		/*!< 手摇捷度系数 */
    PAR_CH_MPG_WORK_RATE,   /*!< 手摇加工系数 */

    PAR_CH_MAC_FRAME = 40,	/*!< 机床结构类型【0, 一般直角系机床 1, 通用五轴机床；2+其它机床】 */
    PAR_CH_MACTURN,			/*!< 车床横式立式类型【0, 横式 1, 立式】 */
    PAR_CH_SHOW_AXES,		/*!< 坐标系界面是否动态显示坐标轴 */
    PAR_CH_G96_MAX_S_LIMIT,      /*!< 恒限速最高转速钳制条件 【0 和修调有关 1 和修调无关】 */
    PAR_CH_G95_DEFAULT_F=44,   /*!<通道的缺省转进给速度 */
    PAR_CH_STD_RAD0,		/*!<最小邻域半径L0 */
    PAR_CH_RATIO_SINGLE_POINT,/*!<单点降速角度比例因子 */
    PAR_CH_RATIO_POINT_ANGLE,/*!<转角比判据最小转角比 */
    PAR_CH_RATIO_RELATIVE_ANGLE,/*!<相对长线段判据最小转角比 */
    PAR_CH_MODLE_JUDGE_CONDITION,/*!<判据组合模式【16进制，末位为判据模式(0 1 2)，首位为曲率半径计算模式(0 1)】 */
    PAR_CH_MAX_OVERRIDE=50,	/*!< 最大进给修调倍数【1.0-2.0】 */
    PAR_CH_ARC_SPEED_DOWN_R,
    PAR_CH_ARC_SPEED_DOWN_V,
    HMI_5AXIS_IJK_SETTING_ON,/*< 开启五轴IJK设置 */
    PAR_CH_ARC_SPEED_DOWN_R2,
    PAR_CH_ARC_SPEED_DOWN_V2,
    PAR_CH_ARC_SPEED_DOWN_R3,
    PAR_CH_ARC_SPEED_DOWN_V3 = 57,
    PAR_CYCYLE_BACK_INTP_PROG=59, /*!<循环启动自动回断点程序号 */
    PAR_CH_FOLLOW_ROTATE_RAD = 60, /*!<工具跟随的摆动半径 */
    PAR_CH_FOLLOW_CHORD_LEN, /*!<弦线跟随的弦长 */
    PAR_CH_FOLLOW_TOOL_AX, /*!<工具旋转的旋转轴号 */
    PAR_CH_FOLLOW_TABLE_AX, /*!<工作台旋转的旋转轴号 */
    PAR_CH_FOLLOW_DX, /*!<工具跟随旋转中心x偏移 */
    PAR_CH_FOLLOW_DY, /*!<工具跟随旋转中心y偏移 */
    PAR_CH_G04_TIME_SHOW,	/*< 显示G04剩余时间 */

    PAR_CH_VPLAN_CRFT_EN = 67,  /*!<第二加工代码工艺参数合并使能 */
    PAR_CH_VPLAN_SPL_EN = 68,  /*!<第二加工代码样条合并使能 */
    PAR_CH_VPLAN_MODE = 69,	/*!< 速度规划模式0-9:曲面模式  10+高速模式【激光、木工】  */

    PAR_CH_MICR_MAX_LEN,	/*!< 微线段上限长度 */ /*!< wxh，从这里开始，最多20个参数，Q0/Q1/Q2/Q3参数必须保持一致 */
    PAR_CH_FINE_STOP_ANG,	/*!< 准停转角阈值 */
    PAR_CH_INTER_TANGT_ANG,	/*!< 内切转角阈值 */
    PAR_CH_PATH_TOLERANCE,	/*!< 轨迹轮廓允差 */
    PAR_CH_CONER_DEC_FACTOR,/*!< 拐角降速比例因子% */
    PAR_CH_HSPL_MIN_LEN,	/*!< 微线段下限长度 */
    PAR_CH_HSPL_MAX_ANG,	/*!< 样条过渡夹角 */
    PAR_CH_HSPL_MAX_RAT,	/*!< 样条平滑的相邻段最大长度比 */
    PAR_CH_HSPL_MAX_LEN,	/*!< 样条平滑的最大线段长度 */
    PAR_CH_ARC2LINE,		/*!< 圆弧离散为直线选择 79 */

    PAR_CH_LOOKAHEAD_NUM,	/*!< 速度规划前瞻段数 */
    PAR_CH_VEL_RANGE_COEF,	/*!< 速度波动范围系数 */
    PAR_CH_RECTIFY_NUM,		/*!< 指令速度平滑周期数 */
    PAR_CH_JERK_TIMES,		/*!< 合成捷度时间常数 */
    PAR_CH_MAX_ECEN_ACC,	/*!< 向心加速度 */
    PAR_CH_MAX_TANG_ACC,	/*!<切向加速度 */
    PAR_CH_FEED_ACC_COEF,	/*!< 加速度系数 */
    PAR_CH_FEED_JK_COEF,		/*!< 捷度系数 */
    PAR_CH_EN_AUTO_ACC,		/*!< 自适应向心加速度使能[根据轴加速度计算] */
    PAR_CH_FLAT_ANGLE,		/*!<平角阈值系数 */

    PAR_CH_CYL_RAX = 90,	/*!< 圆柱插补旋转轴号【缺省5、C轴】 */
    PAR_CH_CYL_LAX,			/*!< 圆柱插补直线【轴向】轴号【缺省2、Z轴】 */
    PAR_CH_CYL_PAX,			/*!< 圆柱插补平行【周向、纬线】轴号【缺省1、Y轴】 */
    PAR_CH_TOOL_RETURN_AXNO,
    PAR_CH_POWEROFF_OF_MAC_TYPE = 94,/*!< 断电机床类型 */

    PAR_CH_POLAR_LAX = 95,	/*!< 极坐标插补的直线轴轴号 */
    PAR_CH_POLAR_RAX,		/*!< 极坐标插补的旋转轴轴号 */
    PAR_CH_POLAR_VAX,		/*!< 极坐标插补的假想轴轴号 */
    PAR_CH_POLAR_CX,		/*!< 极坐标插补的旋转中心直线轴坐标 */
    PAR_CH_POLAR_CY,		/*!< 极坐标插补假想轴偏心量 */
    PAR_CH_POLAR_POLETYPE,  /*!< 极点处理模式 */
    PAR_CH_SP_TOOL_NUM,		/*! <主轴上刀具数 */
    PAR_CH_CHANGE_AXIS_NO = 102,/*!<动态切换轴掩码 */
    PAR_CH_THREAD_AX_RATIO,   /*!<螺纹进刀最大比例 */
    PAR_CH_G95_DEFAULT = 104,/*!< 系统上电时G95/G94模态设置 */
    PAR_CH_G96_MIN = 105,    /*!< G96默认最小速度 */
    //PAR_CH_THREAD_TOL = 105,	/*!< 螺纹起点允许偏差 */
    //PAR_CH_THREAD_WAY,			/*!< 螺纹加工方式 */
    PAR_CH_UVW_IMG_AXIS_MASK =106, /*!<G94.2下圆弧功能专用：UVW虚拟轴掩码 */
    PAR_CH_G61_DEFAULT = 107,		/*!< 系统上电时G61/G64模态设置 */
    PAR_CH_G00_DEFAULT,		/*!< 系统上电时G00/G01模态设置 */
    PAR_CH_G90_DEFAULT,		/*!< 系统上电时G90/G91模态设置 */
    PAR_CH_G28_ZTRAP_EN,	/*!< G28搜索Z脉冲使能 */
    PAR_CH_G28_POS_EN,		/*!< G28到中间点模式 【1 G00定位 0 G01移动】 */
    PAR_CH_G28_ONE_SHOT,	/*!< G29 */
    PAR_CH_SKIP_MODE,		/*!< 任意行模式[0,非扫描 1，扫描，恢复单行轴移动模态 2扫描，恢复全部轴移动模态] */
    PAR_CH_AXIS_RETURN_ORDER, /*!<任意行轴返回顺序 */
    PAR_CH_G00_JERK_TYPE=117,			/*!< G00捷度类型 */
    PAR_CH_MPG_JERK_TYPE,			/*!< 手摇捷度类型 */
    PAR_CH_ARC2LINE_SWITCH,   /*!<圆弧离散为直线使能  */
    PAR_CH_ARC2LINE_CHORD_LEN,/*!<圆弧离散为直线弓高误差  */
    PAR_CH_ARC2LINE_MAX_LEN, /*!<圆弧离散为直线最大步长  */
    PAR_CH_ARC2LINE_MAX_ANGLE = 122, /*!<圆弧离散为直线最大转角  */
    PAR_CH_SAMPLE_PNT_HSPLINE, /*!< hermite采样点数 */
    PAR_CH_SAMPLE_PNT_NURBS, /*!< nurbs采样点数 */
    //PAR_CH_M_GROUP1,             /*!< M代码分组1 */
    // PAR_CH_G95_DEFAULT,		/*!< 系统上电时G95/G94模态设置 */

    PAR_CH_MAG_START_NO = 125,	/*!< 	起始刀库号 */
    PAR_CH_MAG_NUM,				/*!< 	刀库数目 */
    PAR_CH_TOOL_START_NO,		/*!< 	起始刀具号 */
    PAR_CH_TOOL_NUM,			/*!< 	刀具数目 */
    PAR_CH_TOOL_OFF_START_NO,	/*!<	起始刀补号 */
    PAR_CH_LIFE_ON,				/*!< 	刀具寿命功能开启 */

    PAR_CH_TOFF_ON,				/*!<限位与保护区时，刀补开启 */
    PAR_CH_TOFF_LIMIT,			/*!<刀补开启时，Z轴刀尖与负限位的距离 */
    PAR_CH_LIFE_LOSE_NO,        /*!<  T指令寿命管理忽略号	,例如T106，系统将其认为是T(106 - 100) */
    PAR_CH_SYNC_RESET,			/*!<复位是否清除同步 */
    PAR_CH_MILL_LEN,			/*!<铣床刀具组长度补偿 */
    PAR_CH_MILL_ARC,			/*!<铣床刀具组半径补偿 */
    PAR_CH_G00_CHECK_TYPE,      /*!<G00类型为2时，准停检查方式*/
    PAR_CH_G00_CHECK_LIMIT,		/*!<G00类型为2时，指令检查范围*/
    PAR_CH_REDECODE_ENABLE = 139,     /*!<重解释使能  */

    PAR_CH_ROBOT_PARA=200, /*!<200~299机器人参数区 */
    //PAR_CH_WTZONE_NUM = 200,	// 工件及刀具保护区总个数0~10
    //PAR_CH_WTZONE_TYPE,			// 工件及刀具保护区类型
    //PAR_CH_WTZONE_FLAG,			// 工件及刀具保护区属性
    //PAR_CH_WTZONE_BND,			// 工件及刀具保护区边界

    PAR_CH_UNIFORM_EN =227,            /*!<系统匀化开关 */ //在线匀化相关四个参数 gx 2021/7/12
    //默认匀顺参数
    PAR_CH_UNIFORM_FILTER_SIGMA,     /*!<系统匀化平滑率 */
    PAR_CH_UNIFORM_MAX_ANGEL,     /*!<系统匀化最大外拐角 */
    PAR_CH_UNIFORM_SAMPLE_LEN,    /*!<系统匀化采样长度 */
    PAR_CH_UNIFORM_PROFILE_TOLERANCE,   /*!<系统匀化轮廓允差 */
    PAR_CH_UNIFORM_MAX_LENGTH,    /*!<系统匀化上限长度 */
    PAR_CH_UNIFORM_MICR_LENGTH,     /*!<系统匀化噪点长度 */
    //hyj
    PAR_CH_UNIFORM_MAX_IJK_ANGEL,	/*!<匀化程序段判断刀轴矢量夹角阈值*/
    PAR_CH_UNIFORM_IJK_TOLERANCE,	/*!<刀轴矢量允差*/

    PAR_CH_G39_TOL_LEN = 252,      /*!<三点定位轨迹误差 */
    PAR_CH_G39_MAX_ANGLE,	       /*!<三点定位角度幅值 */
    PAR_CH_G39_POLE_TRANSIT_TYPE,  /*!<三点定位极点过渡模式 */

    PAR_CH_HIDE_CORD_AXIS = 264, 	/*!< 隐藏坐标系界面轴内容 2.40.05.04占位*/
    PAR_CH_GRAPH_SIMU_SEL = 265, 	/*!< 视角选择，每个通道4个bit(铣) */
    PAR_CH_GRAPH_VIEWXYZ_X_ANGLE = 266, 					/*!< 视角xyz,转动角度X(铣) */
    PAR_CH_GRAPH_VIEWXYZ_Y_ANGLE,  						/*!< 视角xyz,转动角度Y(铣) */
    PAR_CH_GRAPH_VIEWXYZ_Z_ANGLE,  						/*!< 视角xyz,转动角度Z(铣) */
    PAR_CH_GRAPH_VIEWXYZ_ZOOM,								/*!< 视角xyz,显示比例(铣) */
    PAR_CH_GRAPH_VIEWXYZ_X_CENTER,							/*!< 视角xyz,X中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXYZ_Y_CENTER, 						/*!< 视角xyz,Y中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXYZ_Z_CENTER, 						/*!< 视角xyz,Z中心坐标(铣) */

    PAR_CH_GRAPH_VIEWXY_ZOOM = PAR_CH_GRAPH_VIEWXYZ_Z_CENTER + CH_GRAPH_EVERY_VIEW_PAR_RESV,	/*!< 视角xy,显示比例(铣) */
    PAR_CH_GRAPH_VIEWXY_X_CENTER,									/*!< 视角xy,X中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXY_Y_CENTER, 									/*!< 视角xy,Y中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXY_Z_CENTER, 									/*!< 视角xy,Z中心坐标(铣) */

    PAR_CH_GRAPH_VIEWYZ_ZOOM = PAR_CH_GRAPH_VIEWXY_Z_CENTER + CH_GRAPH_EVERY_VIEW_PAR_RESV,	/*!< 视角yz,显示比例(铣) */
    PAR_CH_GRAPH_VIEWYZ_X_CENTER,									/*!< 视角yz,X中心坐标(铣) */
    PAR_CH_GRAPH_VIEWYZ_Y_CENTER, 									/*!< 视角yz,Y中心坐标(铣) */
    PAR_CH_GRAPH_VIEWYZ_Z_CENTER, 									/*!< 视角yz,Z中心坐标(铣) */

    PAR_CH_GRAPH_VIEWXZ_ZOOM = PAR_CH_GRAPH_VIEWYZ_Z_CENTER + CH_GRAPH_EVERY_VIEW_PAR_RESV,	/*!< 视角xz,显示比例(铣) */
    PAR_CH_GRAPH_VIEWXZ_X_CENTER,									/*!< 视角xz,X中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXZ_Y_CENTER, 									/*!< 视角xz,Y中心坐标(铣) */
    PAR_CH_GRAPH_VIEWXZ_Z_CENTER, 									/*!< 视角xz,Z中心坐标(铣) */

    PAR_CH_GRAPH_TURN_ZOOM_WORK = PAR_CH_GRAPH_VIEWXZ_Z_CENTER + CH_GRAPH_EVERY_VIEW_PAR_RESV,	/*!< 最佳显示比例(车) */
    PAR_CH_GRAPH_TURN_ZOOM_USER,  									/*!< 用户比例缩放系数缩放(车) */
    PAR_CH_GRAPH_TURN_LO,											/*!< 毛坯长度(车) */
    PAR_CH_GRAPH_TURN_DO, 											/*!< 毛坯直径(车) */
    PAR_CH_GRAPH_TURN_DI, 											/*!< 毛坯内侧直径(车) */
    PAR_CH_GRAPH_TURN_OFFL,  										/*!< 对刀点偏移(车) */

    PAR_CH_USER_AD_OFF = 300, /*!< 用户模拟量输入点偏移量。x寄存器，单位字节 */
    PAR_CH_USER_DA_OFF = 301, /*!< 用户模拟量输出点偏移量。y寄存器，单位字节 */

    //PAR_CH_RESONA_DAMP_AMP = 300, // 主轴转速避振波幅【百分比 0.05】
    //PAR_CH_RESONA_DAMP_PRD = 301, // 主轴转速避振周期【秒】

    PAR_CH_TAX_ENABLE = 310,	/*!< 倾斜轴控制使能 */
    PAR_CH_TAX_ORTH_AX_INDEX,	/*!< 正交轴轴号 */
    PAR_CH_TAX_TILT_AX_INDEX,	/*!< 倾斜轴轴号 */
    PAR_CH_TAX_TILT_ANGLE,		/*!< 倾斜角度 */
    PAR_CH_UMICRO_LVL,		/*!< 高速高精加工等级 */
    PAR_CH_JOG_ACC_RATE = 315,	/*!< 手动加速度系数 */

    PAR_CH_G41_TRANS_PROG_ID = 330,       /*!< G41转换程序号 330*/
    PAR_CH_G43_TRANS_PROG_ID,       /*!< G43转换程序号 */
    PAR_CH_G54_TRANS_PROG_ID,       /*!< G54转换程序号 */
    PAR_CH_M00_TRANS_PROG_ID,
    PAR_CH_SPECIAL_FEEDRATE_OVERRIDE,   /*!< 转进给G01速度特殊进给修调值 */

    PAR_CH_EG_MAIN_AXIS = 340,	/*!<主动轴主轴号 */
    PAR_CH_EG_SUB_AXIS,			/*!<从动轴主轴号 */
    PAR_CH_EG_MIAN_RATIO,       /*!<主动轴比例 */
    PAR_CH_EG_SUB_RATIO,        /*!<从动轴比例 */
    PAR_CH_EG_TYPE,				/*!<同步类型，1指令跟随，0实际跟随 */
    PAR_CH_EG_PHASE_ENABLE,     /*!<同步相位开启 */
    PAR_CH_EG_PHASE,		    /*!<同步相位差 */

    PAR_CH_EG2_MAIN_AXIS = 347,	/*!<主动轴主轴号 */
    PAR_CH_EG2_SUB_AXIS,			/*!<从动轴主轴号 */
    PAR_CH_EG2_MIAN_RATIO,       /*!<主动轴比例 */
    PAR_CH_EG2_SUB_RATIO,        /*!<从动轴比例 */
    PAR_CH_EG2_TYPE,				/*!<同步类型，1指令跟随，0实际跟随 */
    PAR_CH_EG2_PHASE_ENABLE,     /*!<同步相位开启 */
    PAR_CH_EG2_PHASE,		    /*!<同步相位差 */

    PAR_CH_EG3_MAIN_AXIS = 354,	/*!<主动轴主轴号 */
    PAR_CH_EG3_SUB_AXIS,			/*!<从动轴主轴号 */
    PAR_CH_EG3_MIAN_RATIO,       /*!<主动轴比例 */
    PAR_CH_EG3_SUB_RATIO,        /*!<从动轴比例 */
    PAR_CH_EG3_TYPE,				/*!<同步类型，1指令跟随，0实际跟随 */
    PAR_CH_EG3_PHASE_ENABLE,     /*!<同步相位开启 */
    PAR_CH_EG3_PHASE,		    /*!<同步相位差 */

    PAR_CH_CARRY_MAIN_AXIS = 361,			/*!<乘载主动主轴号 */
    PAR_CH_CARRY_SUB_AXIS,			/*!<乘载从动主轴号 */
    PAR_CH_CARRY_RATIO,			    /*!<乘载比例 */

    PAR_CH_FREQTAP_SPDL_ACCE_COEF = 364,  /*!<变频主轴刚性攻丝主轴加速公式系数 */
    PAR_CH_FREQTAP_SPDL_REDC_COEF,  /*!<变频主轴刚性攻丝主轴减速公式系数 */
    PAR_CH_FREQTAP_SPDL_DELAY,      /*!<变频主轴刚性攻丝主轴延时时间 */
    PAR_CH_FREQTAP_VEL_COEF,		/*!<变频主轴刚性攻丝速度补偿系数 */
    PAR_CH_FREQTAP_ACC_DIS,			/*!<变频主轴刚性攻丝加速度补偿值（微米） */

    PAR_CH_FREQTAP2_SPDL_ACCE_COEF = 369,  /*!<变频主轴刚性攻丝主轴加速公式系数 */
    PAR_CH_FREQTAP2_SPDL_REDC_COEF,  /*!<变频主轴刚性攻丝主轴减速公式系数 */
    PAR_CH_FREQTAP2_SPDL_DELAY,      /*!<变频主轴刚性攻丝主轴延时时间 */
    PAR_CH_FREQTAP2_VEL_COEF,		/*!<变频主轴刚性攻丝速度补偿系数 */
    PAR_CH_FREQTAP2_ACC_DIS,			/*!<变频主轴刚性攻丝加速度补偿值（微米） */

    PAR_CH_FREQTAP3_SPDL_ACCE_COEF = 374,  /*!<变频主轴刚性攻丝主轴加速公式系数 */
    PAR_CH_FREQTAP3_SPDL_REDC_COEF,  /*!<变频主轴刚性攻丝主轴减速公式系数 */
    PAR_CH_FREQTAP3_SPDL_DELAY,      /*!<变频主轴刚性攻丝主轴延时时间 */
    PAR_CH_FREQTAP3_VEL_COEF,		/*!<变频主轴刚性攻丝速度补偿系数 */
    PAR_CH_FREQTAP3_ACC_DIS,			/*!<变频主轴刚性攻丝加速度补偿值（微米） */

    PAR_CH_FREQTAP4_SPDL_ACCE_COEF = 379,  /*!<变频主轴刚性攻丝主轴加速公式系数 */
    PAR_CH_FREQTAP4_SPDL_REDC_COEF,  /*!<变频主轴刚性攻丝主轴减速公式系数 */
    PAR_CH_FREQTAP4_SPDL_DELAY,      /*!<变频主轴刚性攻丝主轴延时时间 */
    PAR_CH_FREQTAP4_VEL_COEF,		/*!<变频主轴刚性攻丝速度补偿系数 */
    PAR_CH_FREQTAP4_ACC_DIS,			/*! 383 <变频主轴刚性攻丝加速度补偿值（微米） */

    PAR_CH_HEAT_TYPE = 384,		/*!< 五轴热误差补偿类型 */
    PAR_CH_HEAT_WARP_START,		/*!< 五轴热误差表起始温度 */
    PAR_CH_HEAT_WARP_NUM,		/*!< 五轴热误差表温度点数 */
    PAR_CH_HEAT_WARP_STEP,		/*!< 五轴热误差表温度间隔 */
    PAR_CH_HEAT_WARP_SENSOR,	/*!< 五轴热误差表传感器编号 */
    PAR_CH_HEAT_WARP_TABLE,		/*!< 五轴热误差表起始参数号 */

    PAR_CH_IIPDIS_MODE = 390,        /*!<五轴恒进给插补行程计算方式 */
    PAR_CH_CONSVEL_FEED_ACC_COEF,    /*!<五轴恒进给加速度平滑比例因子*/
    PAR_CH_CONSTANT_VEL_RATE,        /*!<五轴恒进给恒进给旋转轴比例系数 */
    PAR_CH_CIP_IIPDIS_MODE,          /*!<五轴空间圆弧插补行程计算方式 */
    PAR_CH_ACC_LIMIT,			 /*!<五轴加速度约束开关 */
    /*!
     * @name 五轴参数，占用400~600，共200个。其他模块禁止使用。zouj 2020/1/25
     * @{
     */
    PAR_CH_RTCPARA_OFF=50,		/*!<RTCP参数偏移值 */

    PAR_CH_TOOL_INIT_DIR_X=400,	/*!<刀具初始方向(X) */
    PAR_CH_TOOL_INIT_DIR_Y,		/*!<刀具初始方向(Y) */
    PAR_CH_TOOL_INIT_DIR_Z,		/*!<刀具初始方向(Z) */
    PAR_CH_TOOL_G54_METHOD,     /*!<RTCP对刀方式 */
    PAR_CH_WCOMP_ENABLE,        /*!<W轴补偿 */
    PAR_CH_ANG_OUTPUT_MODE=405,	/*!<旋转轴角度输出判定方式 */
    PAR_CH_ANG_OUTPUT_ORDER,	/*!<旋转轴角度输出判定顺序 */
    PAR_CH_POLE_TOLERANCE,		/*!<极点角度范围 */
    PAR_CH_SWIVEL_DIVISION,		/*!<摆头分度使能 */
    PAR_CH_SWIVEL_DOUBLE_TOOLCOMP, /*!<直角头双向刀长补 */
    PAR_CH_RTCP_SWIVEL_TYPE=410,	/*!<摆头结构类型 */
    PAR_CH_RTCP_SWIVEL_RAX1_DIR_X,	/*!<摆头第1旋转轴方向(X) */
    PAR_CH_RTCP_SWIVEL_RAX1_DIR_Y,	/*!<摆头第1旋转轴方向(Y) */
    PAR_CH_RTCP_SWIVEL_RAX1_DIR_Z,	/*!<摆头第1旋转轴方向(Z) */
    PAR_CH_RTCP_SWIVEL_RAX2_DIR_X,	/*!<摆头第2旋转轴方向(X) */
    PAR_CH_RTCP_SWIVEL_RAX2_DIR_Y,	/*!<摆头第2旋转轴方向(Y) */
    PAR_CH_RTCP_SWIVEL_RAX2_DIR_Z,	/*!<摆头第2旋转轴方向(Z) */
    PAR_CH_RTCP_SWIVEL_RAX1_OFF_X,	/*!<摆头第1旋转轴偏移矢量(X) */
    PAR_CH_RTCP_SWIVEL_RAX1_OFF_Y,	/*!<摆头第1旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_SWIVEL_RAX1_OFF_Z,	/*!<摆头第1旋转轴偏移矢量(Z) */
    PAR_CH_RTCP_SWIVEL_RAX2_OFF_X,	/*!<摆头第2旋转轴偏移矢量(X) */
    PAR_CH_RTCP_SWIVEL_RAX2_OFF_Y,	/*!<摆头第2旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_SWIVEL_RAX2_OFF_Z,	/*!<摆头第2旋转轴偏移矢量(Z) */

    PAR_CH_RTCP_TABLE_TYPE=425,		/*!<转台结构类型 */
    PAR_CH_RTCP_TABLE_RAX1_DIR_X,	/*!<转台第1旋转轴方向(X) */
    PAR_CH_RTCP_TABLE_RAX1_DIR_Y,	/*!<转台第1旋转轴方向(Y) */
    PAR_CH_RTCP_TABLE_RAX1_DIR_Z,	/*!<转台第1旋转轴方向(Z) */
    PAR_CH_RTCP_TABLE_RAX2_DIR_X,	/*!<转台第2旋转轴方向(X) */
    PAR_CH_RTCP_TABLE_RAX2_DIR_Y,	/*!<转台第2旋转轴方向(Y) */
    PAR_CH_RTCP_TABLE_RAX2_DIR_Z,	/*!<转台第2旋转轴方向(Z) */
    PAR_CH_RTCP_TABLE_RAX1_OFF_X,	/*!<转台第1旋转轴偏移矢量(X) */
    PAR_CH_RTCP_TABLE_RAX1_OFF_Y,	/*!<转台第1旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_TABLE_RAX1_OFF_Z,	/*!<转台第1旋转轴偏移矢量(Z) */
    PAR_CH_RTCP_TABLE_RAX2_OFF_X,	/*!<转台第2旋转轴偏移矢量(X) */
    PAR_CH_RTCP_TABLE_RAX2_OFF_Y,	/*!<转台第2旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_TABLE_RAX2_OFF_Z,	/*!<转台第2旋转轴偏移矢量(Z) */

    PAR_CH_BREAK_TOOL_DIR = 439,    /*!<断点恢复时的刀具方向 */
    PAR_CH_BREAK_SAFE_HIGH,         /*!<断点恢复时刀具方向的安全高度 */

    PAR_CH_RTCP_7AXIS_ENABLE =588,		/*!<7轴RTCP使能 */
    PAR_CH_RTCP_SWIVEL_RAX3_DIR_X,	/*!<摆头第3旋转轴方向(X) */
    PAR_CH_RTCP_SWIVEL_RAX3_DIR_Y,	/*!<摆头第3旋转轴方向(Y) */
    PAR_CH_RTCP_SWIVEL_RAX3_DIR_Z,	/*!<摆头第3旋转轴方向(Z) */
    PAR_CH_RTCP_SWIVEL_RAX3_OFF_X,	/*!<摆头第3旋转轴偏移矢量(X) */
    PAR_CH_RTCP_SWIVEL_RAX3_OFF_Y,	/*!<摆头第3旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_SWIVEL_RAX3_OFF_Z,	/*!<摆头第3旋转轴偏移矢量(Z) */
    PAR_CH_RTCP_TABLE_RAX3_DIR_X,	/*!<转台第3旋转轴方向(X) */
    PAR_CH_RTCP_TABLE_RAX3_DIR_Y,	/*!<转台第3旋转轴方向(Y) */
    PAR_CH_RTCP_TABLE_RAX3_DIR_Z,	/*!<转台第3旋转轴方向(Z) */
    PAR_CH_RTCP_TABLE_RAX3_OFF_X,	/*!<转台第3旋转轴偏移矢量(X) */
    PAR_CH_RTCP_TABLE_RAX3_OFF_Y,	/*!<转台第3旋转轴偏移矢量(Y) */
    PAR_CH_RTCP_TABLE_RAX3_OFF_Z,	/*!<转台第3旋转轴偏移矢量(Z) */
    PAR_CH_TOOL_INIT_SUB_DIR_X,		/*!<刀具初始副方向(X) */ //刀轴副矢量尽量选取与刀轴矢量垂直的方向，以六轴砂带磨为例，取径向为刀轴方向，切向为刀轴副方向
    PAR_CH_TOOL_INIT_SUB_DIR_Y,		/*!<刀具初始副方向(Y) */
    PAR_CH_TOOL_INIT_SUB_DIR_Z,		/*!<刀具初始副方向(Z) */

    PAR_CH_RTCP_10AXIS_ENABLE = 604,		/*!<双五轴立式RTCP使能 */

    PAR_CH_FUNC_SWITCH = 610,/*函数开关，610~~641*/
    PAR_CH_OPTION = 642,/*选项开关，642~~673*/
    /*!@}*/

    PAR_CH_DCM_MANUAL_OPEN = 674,		/*!<开启手动防碰撞功能 */
    PAR_CH_DCM_AUTO_OPEN,			    /*!<开启自动防碰撞功能 */
    PAR_CH_DCM_MODAL_X_ZERO ,           /* 碰撞检测模型X轴零点偏置  */
    PAR_CH_DCM_MODAL_Y_ZERO,            /* 碰撞检测模型Y轴零点偏置  */
    PAR_CH_DCM_MODAL_Z_ZERO,            /* 碰撞检测模型Z轴零点偏置  */

    PAR_CH_TURNTOOL_SHOW_AXIS = 679,        /*! 车床刀偏轴显示配置*/
    PAR_CH_GRAPH_VIEW_QUAD_XYZ = 680,       /*!< 视角xyz,象限(铣) */
    PAR_CH_GRAPH_VIEW_QUAD_XY,              /*!< 视角xy,象限(铣) */
    PAR_CH_GRAPH_VIEW_QUAD_YZ,              /*!< 视角yz,象限(铣) */
    PAR_CH_GRAPH_VIEW_QUAD_XZ,              /*!< 视角xz,象限(铣) */
    PAR_CH_G28_POINT_TO_ZERO_EN,		    /*!< G28从中间点到零点的快移使能 【0 就进给速度定位 1 快移速度定位】 */

    PAR_CH_MOV_SPACE_TYPE,	/*!<主从协同空间平滑类型，  0:不开启主从协同空间平滑, 1:XYZ主运动，其他为从运动 ,2:四轴缠绕机*/
    PAR_CH_MAS_AXIS_TYPE,
    PAR_CH_SLA_AXIS_TYPE,
    PAR_CH_BFC_LONGBLK_THRESHOLD,					/*主从运动空间程序段上限插补周期倍数*/
    PAR_CH_BFC_VELMAX_RATIO,							/*主从运动空间加工速度调整系数*/
    PAR_CH_BFC_BAD_MOVSPACE_CURVE_PEAK_THRESHOLD,		/*主从运动空间加工轨迹上限曲率*/
    PAR_CH_BFC_BAD_MOVSPACE_CURVE_DISTANCE_THRESHOLD,	/*主从运动空间加工轨迹曲率极值点位置偏差上限插补周期倍数*/
    PAR_CH_BFC_BAD_MOVSPACE_CURVE_SMOOTHLEN_THRESHOLD,	/*主从运动空间拟合样条长度下限插补周期倍数*/
    PAR_CH_BFC_VUNIT_LEN__THRESHOLD,					/*主从运动空间速度区间下限插补周期倍数*/
    PAR_CH_BFC_SHORTBLK_THRESHOLD,						/*主从运动空间程序段下限插补周期倍数*/
    PAR_CH_BFC_CROSSBLK_ENDV_ADJUST_FACTOR,				/*主从运动空间大转角程序段速度调整系数*/
    PAR_CH_BFC_LONGBLK_ACCMAX_RATIO,					/*主从运动空间长短程序段衔接加速度调整系数*/

    /* 空闲3个，可用 */

    PAR_CH_VCS_TYPE = 700,				/* 空间误差补偿使能 */
    PAR_CH_VCS_LOOPTYPE,				/* 机床机构链  */
    PAR_CH_VCS_SQ1,						/* 第二轴相对基准轴垂直度  */
    PAR_CH_VCS_SQ2,						/* 第三轴相对基准轴垂直度  */
    PAR_CH_VCS_SQ3,						/* 第三轴相对第二轴垂直度  */

    PAR_CH_TOTAL = 1000
};

/*!
 * @name 坐标轴参数
 * @{
 */
#define SERVO_PARM_START_IDX 500 /*! 伺服参数起始参数号 */
#define SERVO_PARM_TOTAL_NUM 500 /*! 伺服参数个数 */
enum
{
    PAR_AX_NAME = 0,	/*!< 轴名[显示用] */
    PAR_AX_TYPE,		/*!< 轴类型[直线、摆动、回转、主轴] */
    PAR_AX_INDEX,		/*!< 轴编号 暂时预留 */
    PAR_AX_MODN,		/*!< 设备号 暂时预留 */
    PAR_AX_DEV_I = PAR_AX_MODN,
    PAR_AX_PM_MUNIT,	/*!< 电子齿轮比分子(位移量)[每转位移量nm] */
    PAR_AX_PM_PULSE,	/*!< 电子齿轮比分母(脉冲数)[每转指令脉冲数] */
    PAR_AX_PLMT,		/*!< 正软极限 */
    PAR_AX_NLMT,		/*!< 负软极限 */
    PAR_AX_PLMT2,		/*!< 第2正软极限 */
    PAR_AX_NLMT2,		/*!< 第2负软极限 */

    PAR_AX_HOME_WAY = 10,	/*!< 回参考点方式 */
    PAR_AX_HOME_DIR,		/*!< 回参考点方向 */
    PAR_AX_ENC_OFF,			/*!< 编码器反馈偏置量【手动零点、绝对式编码器】 */
    PAR_AX_HOME_OFF,		/*!< 回参考点后的偏移量 */
    PAR_AX_HOME_MASK,		/*!< Z脉冲屏蔽角度 */
    PAR_AX_HOME_HSPD,		/*!< 回参考点高速 */
    PAR_AX_HOME_LSPD,		/*!< 回参考点低速 */
    PAR_AX_HOME_CRDS,		/*!< 参考点坐标值 */
    PAR_AX_HOME_CODSPACE,	/*!< 距离码参考点间距 */
    PAR_AX_HOME_CODOFF,		/*!< 间距编码偏差 */

    PAR_AX_HOME_RANGE = 20,	/*!< 搜Z脉冲最大移动距离 */
    PAR_AX_HOME_CRDS2,		/*!< 第2参考点坐标值 */
    PAR_AX_HOME_CRDS3,		/*!< 第3参考点坐标值 */
    PAR_AX_HOME_CRDS4,		/*!< 第4参考点坐标值 */
    PAR_AX_HOME_CRDS5,		/*!< 第5参考点坐标值 */
    PAR_AX_REF_RANGE,		/*!< 参考点范围偏差 */
    PAR_AX_HOME_CYCLE_OFF,	/*!< 非整传动比回转轴偏差 */
    PAR_AX_ENC2_OFF,		/*!< 第2编码器反馈偏置量【手动零点、绝对式编码器】 */
    PAR_AX_PM2_MUNIT,		/*!< 第2编码器电子齿轮比分子(位移量)[每转位移量nm] */
    PAR_AX_PM2_PULSE,		/*!< 第2编码器电子齿轮比分母(脉冲数)[每转指令脉冲数] */

    PAR_AX_G60_OFF = 30,	/*!< 单向定位(G60)偏移量 */
    PAR_AX_ROT_RAD,			/*!< 转动轴当量半径 */
    PAR_AX_JOG_LOWSPD,		/*!< 慢速点动速度 */
    PAR_AX_JOG_FASTSPD,		/*!< 快速点动速度 */
    PAR_AX_RAPID_SPD,		/*!< 快移速度 */
    PAR_AX_FEED_SPD,		/*!< 最高进给速度 */
    PAR_AX_RAPID_ACC,		/*!< 快移加速度 */
    PAR_AX_RAPID_JK,		/*!< 快移捷度 */
    PAR_AX_FEED_ACC,		/*!< 进给加速度 */
    PAR_AX_FEED_JK,			/*!< 进给捷度 */
    PAR_AX_THREAD_ACC,		/*!< 螺纹加速度 */
    PAR_AX_THREAD_DEC,		/*!< 螺纹减速度 */
    PAR_AX_MPG_UNIT_SPD,	/*!< 手摇单位速度比例系数 */
    PAR_AX_MPG_RESOL,		/*!< 手摇脉冲分辨率 */
    PAR_AX_MPG_STAGE_VEL,	/*!< 手摇分档速度 */
    PAR_AX_MPG_INTE_TIME,	/*!< 手摇缓冲周期 [45] */
    PAR_AX_MPG_SMOOTH_TIME,	/*!< 手摇过冲周期 */
    PAR_AX_MPG_SPD,			/*!< 手摇进给速度 */
    PAR_AX_RAPID_RATE,		/*!< 超速报警系数 */
    PAR_AX_FOLLOW_RATE,		/*!< 轴跟踪误差系数，1000mm/min的跟踪误差 */

    PAR_AX_DEFAULT_S = 50,	/*!< 主轴缺省转速 */
    PAR_SPDL_MAX_SPEED,		/*!< 主轴最大转速 */
    PAR_SPDL_SPD_TOL,		/*!< 主轴转速允许转速波动率 */
    PAR_SPDL_SPD_TIME,		/*!< 主轴转速到达允许最大时间 */
    PAR_SPDL_THREAD_TOL,	/*!< 螺纹加工时的转速允差 */
    PAR_AX_SP_ORI_POS,		/*!< 进给主轴定向角度 */
    PAR_AX_SP_ZERO_TOL,		/*!< 进给主轴零速允差【脉冲】 */
    PAR_AX_MAX_EXT_PINC,	/*!< 外部指令最大周期叠加量 */

    PAR_AX_EXT_LOAD_EN,		/*!< 负载由外部导入 */
    PAR_AX_THREAD_START_TOL, /*!< 螺纹起刀允差 */
    PAR_AX_POS_TOL = 60,	/*!< 定位允差 */
    //	PAR_AX_MAX_LAG,			/*!< 最大跟随误差 */
    PAR_AX_LAG_CMP_EN,		/*!< 龙门轴同步误差补偿使能 */
    PAR_AX_LAG_CMP_COEF,	/*!< 跟随误差补偿调整系数 */
    PAR_AX_LAG_CMP_CNT,		/*!< 动态补偿系数整定周期数 */

    PAR_AX_ATEETH = 65,	/*!< 传动比分子[轴侧齿数] */
    PAR_AX_MTEETH,		/*!< 传动比分母[电机侧齿数] */
    PAR_AX_MT_PPR,		/*!< 电机每转脉冲数 */
    PAR_AX_PITCH,		/*!< 丝杆导程 */
    PAR_AX_RACK_NUM,	/*!< 齿条齿数 */
    PAR_AX_RACK_SPACE,	/*!< 齿条齿间距 */
    PAR_AX_WORM_NUM,	/*!< 蜗杆头数 */
    PAR_AX_WORM_SPACE,	/*!< 蜗杆齿距 */

    PAR_AX_RATING_CUR = 74,  /*!< 额定电流 */
    PAR_AX_POWER_RATE,  /*!< 功率系数 */
    PAR_AX_ENC2_PPR,	/*!< 第2编码器每转脉冲数 */
    PAR_AX_INDEX_TYPE, /*!<分度轴类型：1、鼠牙盘；2，分度轴 */
    PAR_AX_INDEX_POS,/*!<分度起点 */
    PAR_AX_INDEX_DIVIDE, /*!<分度间隔 */
    PAR_AX_INDEX_LOCK_M, /*!<分度加锁M代码 */
    PAR_AX_INDEX_UNLOCK_M, /*!<分度解锁M代码 */
    //PAR_ZAX_LOCK_EN = 80,	/*!< Z轴锁允许使能 */
    //PAR_RAX_ROLL_EN,		/*!< 旋转轴循环使能 */
    PAR_RAX_PATH_MODE,		/*!< 旋转轴路径模式：0、正常；1、短路径；2、一直正向；3、一直负向 */
    PAR_RAX_CYCLE_RANGE,	/*!< 旋转轴循环行程 */
    PAR_RAX_DISP_RANGE,		/*!< 旋转轴显示角度范围 */
    PAR_LAX_PROG_UNIT,		/*!< 直线轴编程指令最小单位 */
    PAR_RAX_PROG_UNIT,		/*!< 旋转轴编程指令最小单位 */
    PAR_OVERLOAD_JUDGE,		/*!< 轴过载判定百分比 */
    PAR_AX_INDEX_POS_TOL,   /*!<分度不移动阈值*/
    PAR_AX_ENC_MODE = 90,	/*!< 编码器工作模式\n
 0x000F：0,用第1编码器反馈 1,用第2编码器反馈  2,无反馈\n
 0x00F0：0,用第1编码器指令 1,用第2编码器指令\n
 0x0100：0,跟随误差由伺服驱动器反馈 1,跟踪误差由系统计算\n
 0x1000：0,默认采用32位脉冲计数 1,采用64位脉冲计数  */
#define AX_ENCODER_MASK	0x00FF
#define AX_NC_CMD_MASK  0x00F0
#define AX_NC_TRACK_ERR	0x0100
#define AX_NC_CYC64_MODE 0x1000

    PAR_AX_EC1_TYPE,		/*!< 1号编码器类型【增量、距离码、绝对】 */
    PAR_AX_EC1_OUTP,		/*!< 反馈电子齿轮比分子[输出脉冲数] */
    PAR_AX_EC1_FBKP,		/*!< 反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_EC1_BIT_N,		/*!< 1号编码器计数位数【绝对式必填】 */
    PAR_AX_EC2_TYPE,		/*!< 2号编码器类型【增量、距离码、绝对】 */
    PAR_AX_EC2_OUTP,		/*!< 反馈电子齿轮比分子[输出脉冲数]  */
    PAR_AX_EC2_FBKP,		/*!< 反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_EC2_BIT_N,		/*!< 2号编码器计数位数【绝对式必填】 */
    PAR_AX_PARAM,           /*!< 轴参数 */

    PAR_AX_SMX_TYPE = 100,	/*!< 运动控制(MC)轴类型  */
    PAR_AX_SMX_LEAD_IDX,
    PAR_AX_COMPEN_LAG = 106,
    PAR_AX_ALARM_LAG,
    PAR_AX_ALARM_VDIFF,
    PAR_AX_ALARM_CDIFF,
    PAR_AX_SMX_PARA,		/*!< MC轴运动系数，16 */

    // wcf 添加
    PAR_AX_IS_SET_HOME_OFFSET = 121,    /*!< 设置编码器反馈偏置量使能 */
    PAR_AX_VEL_LIMIT = 122,             /*!< 轴的最大速度上限(电机最高限速值) */
    PAR_AX_MAX_FERROR = 123,            /*!< 最大跟随误差 */
    PAR_AX_MIN_FERROR = 124,            /*!< 最小跟随误差 */


    PAR_AX_WCS_DISP=126,	/*!< 同步时，从轴工件坐标系显示方式 */
    PAR_AX_SYNC_MIRROR,		/*!< 是否镜像 */
    PAR_AX_COORD_DIR,		/*!< 主从轴坐标系的正方向 */
    PAR_AX_ZERO_OFFSET,		/*!< 同步轴机床零点偏差 */

    PAR_AX_COMP_MAX_COEF = 130,	/*!< 最大误差补偿率 */
    PAR_AX_COMP_MAX_VALUE,		/*!< 最大误差补偿值 */
    PAR_AX_CODOFF_VALUE,		/*!< 轴反馈偏差 */

    PAR_AX_TANG_CRDS = 133,     /*!< 切线控制引导轴坐标系 */
    PAR_AX_TANG_WAIT = 134,     /*!< 切线控制引导轴是否等待 */
    PAR_AX_TANG_NO = 135,		/*!< 切线控制随动轴轴号0，1，2代表A，B，C轴，此参数有150移动到此 */
    PAR_AX_TANG_ANGLE,			/*!< 切线控制偏移角 */
    PAR_AX_TANG_OFFSET,			/*!< 切线跟随偏差值 */
    PAR_AX_HALF_ALL,			/*!< 全闭环、半闭环差值报警阈值 */
    PAR_AX_STOC_ABC,			/*!< 主轴CS切换的轴号 */

    //wangxu
    PAR_AX_ADJUST_COE=140, /*!< 蛙跳调节系数 */
    PAR_AX_FOL_COE, /*!< 随动比例增益 */
    PAR_AX_FOL_ADR, /*!< 随动模拟量地址 */
    PAR_AX_FOL_POS_DOWN, /*!< 随动位置下限 */
    PAR_AX_FOL_SPD, /*!< 随动最大转速 */
    PAR_AX_FOL_OFF, /*!< 随动到位信号误差范围 */
    PAR_AX_JUMP_HIGH, /*!< 蛙跳高度 */
    PAR_AX_FOL_POS_UP, /*!< 随动位置上限 */
    PAR_AX_JUMP_TO_FOL, /*!< 蛙跳切换到随动的高度 */
    PAR_AX_FOL_NONLINER, /*!< 随动电压非线性，需要先标定 */
    PAR_AX_DEM_SPACE, /*!< 标定的间距 */
    PAR_AX_DEM_VOL, /*!< 碰板电压 */

    PAR_AX_SPEED_NEGATION_FLAG = 152,   /*!< 主轴速度模式下的指令取反 */
    PAR_AX_SPDL_MANU_PAR_BASE,          /*!< 主轴手动档位用户参数地址*/
    PAR_AX_SPDL_MANU_SHIFT_NUM,         /*!< 主轴手动档位数*/
    PAR_AX_S_ACK=155,       /*!< S指令需要响应 */
    PAR_AX_OUT_DA,          /*!< 主轴输出模拟量 */
    PAR_AX_MAX_MOTOR,       /*!< 主轴电机最大转速 */
    PAR_AX_SHIFT_NUM,       /*!< 主轴挡位数 */
    PAR_AX_LOW_SPEED1,      /*!< 主轴1档最低转速 */
    PAR_AX_HIGH_SPEED1,     /*!< 主轴1档最高转速 */
    PAR_AX_ATEETH1,      	/*!< 主轴1档传动比分子[轴侧齿数] */
    PAR_AX_MTEETH1,	    	/*!< 主轴1档传动比分母[电机侧齿数] */
    PAR_AX_EC_OUTP1,		/*!< 主轴1档反馈电子齿轮比分子[输出脉冲数] */
    PAR_AX_EC_FBKP1,		/*!< 主轴1档反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_LOW_SPEED2,      /*!< 主轴2档最低转速 */
    PAR_AX_HIGH_SPEED2,     /*!< 主轴2档最高转速 */
    PAR_AX_ATEETH2,      	/*!< 主轴2档传动比分子[轴侧齿数] */
    PAR_AX_MTEETH2,	    	/*!< 主轴2档传动比分母[电机侧齿数] */
    PAR_AX_EC_OUTP2,		/*!< 主轴2档反馈电子齿轮比分子[输出脉冲数] */
    PAR_AX_EC_FBKP2,		/*!< 主轴2档反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_LOW_SPEED3,      /*!< 主轴3档最低转速 */
    PAR_AX_HIGH_SPEED3,     /*!< 主轴3档最高转速 */
    PAR_AX_ATEETH3,      	/*!< 主轴3档传动比分子[轴侧齿数] */
    PAR_AX_MTEETH3,	    	/*!< 主轴3档传动比分母[电机侧齿数] */
    PAR_AX_EC_OUTP3,		/*!< 主轴3档反馈电子齿轮比分子[输出脉冲数] */
    PAR_AX_EC_FBKP3,		/*!< 主轴3档反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_LOW_SPEED4,      /*!< 主轴4档最低转速 */
    PAR_AX_HIGH_SPEED4,     /*!< 主轴4档最高转速 */
    PAR_AX_ATEETH4,      	/*!< 主轴4档传动比分子[轴侧齿数] */
    PAR_AX_MTEETH4,	    	/*!< 主轴4档传动比分母[电机侧齿数] */
    PAR_AX_EC_OUTP4,		/*!< 主轴4档反馈电子齿轮比分子[输出脉冲数] */
    PAR_AX_EC_FBKP4,		/*!< 主轴4档反馈电子齿轮比分母[反馈脉冲数] */
    PAR_AX_SWITCH_SPEED,    /*!< 启用切换点转速 */
    PAR_AX_1S2_SPEED,       /*!< 档位一与档位二切换点转速 */
    PAR_AX_2S3_SPEED,       /*!< 档位二与档位三切换点转速 */
    PAR_AX_3S4_SPEED,       /*!< 档位三与档位四切换点转速 */
    PAR_AX_SHIGT_SPEED,     /*!< 主轴换挡时电机转速 */
    PAR_AX_SHIGT_HOME,      /*!< 主轴换挡后需要重新回零 */

    PAR_FEED_AX_SHIFT_NUM = 189,  /*!< 进给轴挡位数 */
    PAR_FEED_AX_EC_OUTP1,   /*!< 进给轴1档传动比分子*/
    PAR_FEED_AX_EC_FBKP1,	/*!< 进给轴1档传动比分母*/
    PAR_FEED_AX_EC_OPUT2,   /*!< 进给轴2档传动比分子*/
    PAR_FEED_AX_EC_FBKP2,	/*!< 进给轴2档传动比分母*/
    PAR_AX_VIB_STOP = 194, /*!< 震荡是否立刻停止 */
    PAR_AX_VIB_OVERRIDE = 195, /*!< 震荡是否受修调控制 */
    PAR_AX_ACT_PULSE_TOLERANCE = 196,	/*!< 断电反馈脉冲位置允差 */
    PAR_AX_ENC_TOLERANCE = 197,	/*!< 断电位置允差 */
    PAR_AX_OUT_ACTVEL = 198,	/*!< 实际速度超速判断周期 */
    PAR_AX_INTEG_PRD = 199,		/*!< 显示速度积分周期数 */
    PAR_AX_DRIVE_SHAFT_TYPE, /*!< 传动类型 */
    PAR_AX_SLIDE_WAY_TYPE, /*!< 导轨类型 */


    PAR_AX_PLMT3,		/*!< 第3正软极限 */
    PAR_AX_NLMT3,		/*!< 第3负软极限 */
    PAR_AX_PLMT4,		/*!< 第4正软极限 */
    PAR_AX_NLMT4,		/*!< 第4负软极限 */
    PAR_AX_PLMT5,		/*!< 第5正软极限 */
    PAR_AX_NLMT5,		/*!< 第5负软极限 */

    PAR_AX_NONINT_ENABLE,    /*!< 非整数比时多圈计算功能 */
    PAR_AX_NONINT_MT_PPR,    /*!< 非整数比轴多圈脉冲数(脉冲) */

    PAR_AX_HOMECRDS_ENABLE,    /*!< 距离码参考点坐标值使能 */
    PAR_AX_DIS_HOME_CRDS,		/*!< 距离码参考点坐标值 */
    PAR_AX_EG_COMP,				/*!< 电子齿轮补偿周期 */
    PAR_AX_EG_COMP_SMOOTH,				/*!< 电子齿轮补偿滤波周期 */
    PAR_AX_TAPPING_SYNC,    /*!< 开启攻丝同步功能 */
    PAR_AX_TAPPING_SYNC_ERR,    /*!< 开启攻丝偏差限制值（微米） */

    PAR_AX_G00_ACC_TIME,	/*!<G00类型为2时，加速度时间常数*/
    PAR_AX_G00_JK_TIME,		/*!<G00类型为2时，捷度时间常数*/
    PAR_AX_COMP_SMOOTH,		/*!<最大误差平滑周期 */
    PAR_AX_G00_POS_CHECK,/*!< 重叠开启时的到位检查范围 */

    PAR_AX_DIR_HOME=220,/*!< 距离码光栅进行位置检查 */
    PAR_AX_DIR_HOME_LIMIT = 221,/*!< 距离码光栅进行位置检查的报警阈值 */
    PAR_AX_DIR_HOME_VEL = 222,/*!< 距离码光栅速度模式重新建立零点 */

    PAR_AX_VARIABLE_GAIN_COMP_TYPE = 223,/*!< 位置变增益补偿类型：0为速度前馈补偿，1为三角形垂直型补偿，2为三角形非垂直型补偿，3为四边形补偿 */
    PAR_AX_VARIABLE_GAIN_COMP_LEN_PITCH, /*!< 直线轴1圈对应的距离（螺距），单位mm */
    PAR_AX_VARIABLE_GAIN_COMP_LEN_SIDE1, /*!< 边长1：三角形：转台在零点时“转台回转中心”到“滑块”的距离；四边形：转台在零点时“转台回转中心”到“滑块”的垂直距离，单位mm */
    PAR_AX_VARIABLE_GAIN_COMP_LEN_SIDE2, /*!< 边长2：三角形：转台在零点时“转台回转中心”到“电机座回转中心”的距离；四边形：转台在零点时“转台回转中心”到“两摆杆间转动副”的距离，单位mm */
    PAR_AX_VARIABLE_GAIN_COMP_LEN_SIDE3, /*!< 边长3：三角形：转台在零点时“电机座回转中心”到“滑块”的距离；四边形：转台在零点时“两摆杆间转动副”到“滑块”的距离，单位mm */
    PAR_AX_VARIABLE_GAIN_COMP_MOTOR_DIRE,/*!< 当223号参数填2或3时，指定滑块的运动方向：-1为转台正转时滑块中心靠近电机座中心，1为转台正转时滑块中心远离电机座中心 */
    PAR_AX_VARIABLE_GAIN_COMP_DEVI_ANG,	 /*!< 当223号参数填1时，垂直型的偏差角，单位度 */

    PAR_AX_FF_TYPE = 230,			/*!< 前馈控制类型 */
    PAR_AX_VEL_KFF_ON_VLOOP,		/*!< 速度环前馈的速度系数 */
    PAR_AX_ACC_KFF_ON_CLOOP,		/*!< 电流环前馈的加速度系数 */
    PAR_AX_GRAVITY_LENGTH_ON_CLOOP, /*!< 电流环摆动轴初始重力臂mm */
    PAR_AX_GRAVITY_KFF_ON_CLOOP,	/*!< 电流环摆动轴重力补偿系数 */
    PAR_AX_FF_GEAR_RATIO_NUME,		/*!< 前馈控制传动比分子 */
    PAR_AX_FF_GEAR_RATIO_DENO,		/*!< 前馈控制传动比分母 */
    PAR_AX_FF_NOTINT_OFFSET_ON_VLOOP,/*!< 速度环前馈指令非整数周期偏移 */
    PAR_AX_FF_NOTINT_OFFSET_ON_CLOOP,/*!< 电流环前馈指令非整数周期偏移 */
    PAR_AX_FF_RESO_RATIO_ON_VLOOP,	/*!< 速度环前馈量分辨率 */
    PAR_AX_FF_MOTOR_INERTIA,		/*!< 电机转动惯量(10^-3*kg*m2) */
    PAR_AX_FF_MOMENT_INERTIA,		/*!< 电机负载惯量(10^-3*kg*m2) */
    PAR_AX_RATED_CURRENT_ON_CLOOP,	/*!< 电流环前馈的额定电流 */
    PAR_AX_FF_TORQUE_CONSTANT,		/*!< 电机转矩常数（电机额定转矩除以电机额定电流），单位Nm/A */
    PAR_AX_FRICTION_FF_ENABLE,		/*!< 摩擦前馈开关 */
    PAR_AX_FF_FRICTION_S,			/*!< 正向最大静摩擦力矩 */
    PAR_AX_FF_FRICTION_C,			/*!< 正向库仑摩擦力矩 */
    PAR_AX_FF_FRICTION_VS,			/*!< 正向Stribeck速度 */
    PAR_AX_FF_FRICTION_B,			/*!< 正向粘滞摩擦系数 */
    PAR_AX_FF_FRICTION_ALPHA,		/*!< 正向粘滞摩擦速度指数项 */
    PAR_AX_FF_FRICTION_BACK_S,		/*!< 负向最大静摩擦力矩 */
    PAR_AX_FF_FRICTION_BACK_C,		/*!< 负向库仑摩擦力矩 */
    PAR_AX_FF_FRICTION_BACK_VS,		/*!< 负向Stribeck速度 */
    PAR_AX_FF_FRICTION_BACK_B,		/*!< 负向粘滞摩擦系数 */
    PAR_AX_FF_FRICTION_BACK_ALPHA,	/*!< 负向粘滞摩擦速度指数项 */
    PAR_AX_FF_FRICTION_RHO,			/*!< 控制滞回曲线形状参数1 */
    PAR_AX_FF_FRICTION_SIGMA,	    /*!< 控制滞回曲线形状参数2 */
    PAR_AX_FF_FRICTION_N,		    /*!< 控制滞回曲线形状参数3 */
    PAR_AX_FF_MOTOR_TORQUE,			/*!<额定扭矩（Nm） */
    PAR_AX_FF_MIN_VEL,				/*!<前馈最小速度（mm/min） */
    PAR_AX_VEL_FF_SMOOTH_ON_VLOOP,	/*!<速度环前馈的速度平滑周期 */
    PAR_AX_ACC_KFF_ON_VLOOP,		/*!<速度环前馈的加速度系数 */
    PAR_AX_ACC_FF_SMOOTH_ON_VLOOP,	/*!<速度环前馈的加速度平滑周期 */
    PAR_AX_VEL_KFF_ON_CLOOP,		/*!<电流环前馈的速度系数 */
    PAR_AX_VEL_FF_SMOOTH_ON_CLOOP,	/*!<电流环前馈的速度平滑周期 */
    PAR_AX_ACC_FF_SMOOTH_ON_CLOOP,	/*!<电流环前馈的加速度平滑周期 */
    PAR_AX_FF_CONNECT_TYPE,			/*!<前馈控制衔接类型：0为关闭，1为线性插值，2为三次多项式插值 */
    PAR_AX_FF_CONNECT_VEL,			/*!<前馈控制衔接速度（mm/min） */
    PAR_AX_FRICTION_MIN_VEL,		/*!<摩擦前馈最小速度（mm/min） */

    PAR_AX_SPDL_ORIENT_WAY = 274,       /*!< 为主轴时，主轴定向方式 */
    PAR_AX_SPDL_HOME_PULSE,             /*!< 为主轴时，编码器零位所存脉冲 */
    PAR_AX_SPDL_ORIENT_CENTRIPETAL,     /*!< 为主轴时，开启分中定向功能 */
    PAR_AX_JOG_CS_DALAY,                /*!< 手动CS切换最大延时时间*/

    PAR_AX_FF_ENC_PPR = 278,			/*!< 电机每转脉冲数（电机旋转一周的脉冲数）*/

    PAR_ECAT_MULTI_AXIS_MASK = 497,	/*!< EtherCat多合一轴掩码 */
    PAR_ECAT_MOTOR_RATING_CUR_RATE = 498,	/*!< EtherCat额定电流系数 */
    PAR_ECAT_MOTOR_RATING_CUR = 499,	/*!< EtherCat额定电流 */

    /*!
     * @name 伺服参数（进给轴，预留500个）
     * @{
     */
    PAR_SV_POSITION_GAIN = SERVO_PARM_START_IDX,	/*!< 位置比例增益 */
    PAR_SV_POS_FF_GAIN,				/*!< 位置前馈增益 */
    PAR_SV_SPEED_GAIN,				/*!< 速度比例增益 */
    PAR_SV_SPEED_KI,				/*!< 速度积分时间常数 */
    PAR_SV_SPEED_FB_FILTER,			/*!< 速度反馈滤波因子 */
    PAR_SV_MACHINE_VEL_ADD_TIME_CONST = (SERVO_PARM_START_IDX+6),     /*!< 加速时间常数 */
    PAR_SV_WORKING_MODE = (SERVO_PARM_START_IDX+23),               /*!< 驱动器工作模式 */
    PAR_SV_NUM_OF_POLE_PAIRS,             /*!< 磁极对数 */
    PAR_SV_MOTOR_ENCODE_TYPE,				/*!< 编码器反馈类型 */
    PAR_SV_ZERO_OFFSET,						/*!< 伺服编码器零位偏移参数索引 */
    PAR_SV_CUR_CONTROL_PROPORTION_GAIN,   /*!< 电流控制比例增益 */
    PAR_SV_CUR_CONTROL_INTEGRAL_TIME,                                 /*!< 电流控制积分时间 */

    PAR_SV_TORQUE_FILTER_INSTRUCT_TIME = (SERVO_PARM_START_IDX+32),   /*!< 转矩指令滤波时间 */

    PAR_SV_SERVO_VERSION = (SERVO_PARM_START_IDX + 34),				  /*!< 进给轴伺服版本号 */   /*!< 控制字由135变更为34 （2019-5-24） 自整定功能控制字*/

    PAR_SV_MOTOR_TYPE = (SERVO_PARM_START_IDX+43), 	/*!< 驱动器电机类型代码 */
    PAR_SV_TORQUE_RATIO_OF_INERIAS = (SERVO_PARM_START_IDX+56),       /*!< 转矩惯量比值 */
    PAR_SV_LOAD_RATIO_OF_INERIAS,                   /*!< 负载惯量比 */
    PAR_SV_FIRST_BAND_TRAP_FREQUENCY = (SERVO_PARM_START_IDX+76),     /*!< 第1陷波器频率 */
    PAR_SV_FIRST_BAND_TRAP_WIDTH,                   /*!< 第1陷波器宽度 */
    PAR_SV_FIRST_BAND_TRAP_DEPTH,                   /*!< 第1陷波器深度 */
    PAR_SV_SECOND_BAND_TRAP_FREQUENCY,              /*!< 第2陷波器频率 */
    PAR_SV_SECOND_BAND_TRAP_WIDTH,                  /*!< 第2陷波器宽度 */
    PAR_SV_SECOND_BAND_TRAP_DEPTH,                  /*!< 第2陷波器深度 */
    PAR_SV_BAND_TRAP_APPLY_MODE,                    /*!< 陷波器应用模式 */

    PAR_SV_INERTIA_MODE = (SERVO_PARM_START_IDX + 83),      /*!< 惯量辨识模式*/
    PAR_SV_INERTIA_ACC,                                     /*!< 惯量辨识最大速度*/
    PAR_SV_INERTIA_TIME,                                    /*!< 惯量辨识加速时间*/
    PAR_SV_MOTOR_RATING_CUR,								/*!< 电机额定电流 */
    PAR_SV_MOTOR_RATING_SPEED,								/*!< 电机额定转速 */

    PAR_SV_MOTOR_OPEN_LOOP_CUR = (SERVO_PARM_START_IDX + 92),	/*!< 电机开环运行电流 */

    PAR_SV_SYNC_FUNC = (SERVO_PARM_START_IDX + 108),        /*!< 同步功能控制字1 */

    PAR_SV_MOTOR_CIRCULAR_GRATING = (SERVO_PARM_START_IDX + 120),	/*!< 圆光栅位数 */

    PAR_SV_INERTIA_STATE = (SERVO_PARM_START_IDX + 123),    /*!< 136 根据雷力新的通讯协议，状态字由136改为123（2019-5-24）; 惯量辨识状态字*/
    PAR_SV_INERTIA_AMP,                                     /*!< 惯量辨识幅值*/
    PAR_SV_INERTIA_FREQUENCY,                               /*!< 惯量辨识频率*/
    PAR_SV_NEW_VERSION_MOTOR_TYPE,	/*!< 新版本驱动器电机类型代码 */
    PAR_SV_SOURCE_OF_PARAMAN,										/*!< 参数来源选择 */
    PAR_SV_STIFFNESS_LEVEL,											/*!< 刚度等级 */

    PAR_SV_CALIBARIONT_CTRL = (SERVO_PARM_START_IDX + 135), /*!< 伺服校零控制参数索引 */
    PAR_SV_CALIBARIONT_STATE,                               /*!< 伺服校零状态参数索引 */

    PAR_SV_NEW_TORQUE_RATIO_OF_INERIAS = (SERVO_PARM_START_IDX + 141),       /*!< 新版本转矩惯量比值 */
    PAR_SV_NEW_LOAD_RATIO_OF_INERIAS,                   /*!< 新版本负载惯量比 */

    PAR_SV_SERVO_VERSION_ECAT = (SERVO_PARM_START_IDX + 144),   /*!< 自整定功能控制字ECAT*/

    PAR_SV_INERTIA_STATE_ECAT = (SERVO_PARM_START_IDX + 147),   /*!< 惯量辨识频率*/
    PAR_SV_SERVO_PARAM_COUNT,	/*!< 伺服参数个数(新版本有效) */

    /*!@}*/

    /*!
     * @name 伺服参数（主轴，预留500个）
     * @{
     */
    PAR_SP_POSITION_GAIN = SERVO_PARM_START_IDX,	/*!< 位置控制位置比例增益 */

    PAR_SP_MOTOR_SPEED_MAX = (SERVO_PARM_START_IDX + 17),               /*!< 电机最高速度 */

    PAR_SP_WORKING_MODE = (SERVO_PARM_START_IDX + 23),               /*!< 驱动器工作模式 */
    PAR_SP_NUM_OF_POLE_PAIRS,             /*!< 主轴磁极对数 */
    PAR_SP_MOTOR_ENCODE_TYPE,				/*!< 主轴编码器反馈类型 */
    PAR_SP_ZERO_OFFSET,						/*!< 主轴伺服编码器零位偏移参数索引 */


    PAR_SP_SERVO_CTRL1 = (SERVO_PARM_START_IDX + 31), /*!< 状态控制字1 */
    PAR_SP_SERVO_CTRL2, /*!< 状态控制字2 */

    PAR_SP_SERVO_VERSION = (SERVO_PARM_START_IDX + 41),	/*!< 主轴伺服版本号 */

    PAR_SP_MOTOR_RATING_CUR = (SERVO_PARM_START_IDX+53),	/*!< IM电机额定电流 */

    PAR_SP_PM_MOTOR_RATING_CUR = (SERVO_PARM_START_IDX + 56),								/*!< 主轴电机额定电流 */
    PAR_SP_MOTOR_RATING_SPEED,								/*!< 主轴电机额定转速 */
    PAR_SP_MOTOR_TYPE = (SERVO_PARM_START_IDX+59),		/*!< 驱动器电机类型代码 */
    /*!@}*/

    PAR_SP_MOTOR_OPEN_LOOP_CUR = (SERVO_PARM_START_IDX + 73),	/*!< 主轴电机开环运行电流 */

    PAR_SP_CALIBARIONT_CTRL = (SERVO_PARM_START_IDX + 114), /*!< 主轴伺服校零控制参数索引 */
    PAR_SP_CALIBARIONT_STATE,                               /*!< 主轴伺服校零状态参数索引 */

    PAR_ENCODER_CHECK = (SERVO_PARM_START_IDX+119),  /*!< 正余弦采样编码器选择 */

    PAR_RASTER_DETEC_COMP_ENABLE = (SERVO_PARM_START_IDX+140),		/*!< 光栅检测补偿使能 */
    PAR_FIRST_ENCODER_SIN_AMPLITUDE_OFF = (SERVO_PARM_START_IDX+144),		/*!< 第一编码器正弦幅值偏移 */
    PAR_FIRST_ENCODER_COS_AMPLITUDE_OFF = (SERVO_PARM_START_IDX+145),		/*!< 第一编码器余弦幅值偏移 */
    PAR_FIRST_ENCODER_SIN_COS_MISMATCH = (SERVO_PARM_START_IDX+146),		/*!< 第一编码器正余弦不匹配 */
    PAR_SECOND_ENCODER_SIN_AMPLITUDE_OFF = (SERVO_PARM_START_IDX+147),		/*!< 第二编码器正弦幅值偏移 */
    PAR_SECOND_ENCODER_COS_AMPLITUDE_OFF = (SERVO_PARM_START_IDX+148),		/*!< 第二编码器余弦幅值偏移 */
    PAR_SECOND_ENCODER_SIN_COS_MISMATCH = (SERVO_PARM_START_IDX+149),		/*!< 第二编码器正余弦不匹配 */
    /*!@}*/

    PAR_AX_TOTAL = 1000
};
/*!@}*/



/*! 误差补偿参数 */
enum
{
    PAR_CMP_BL_ENABLE = 0,		/*!< 反向间隙补偿类型 */
    PAR_CMP_BL_VALUE,			/*!< 反向间隙补偿值 */
    PAR_CMP_BL_RATE,			/*!< 反向间隙补偿率 */
    PAR_CMP_BL_VALUE2,			/*!< 第2反向间隙补偿值（快移反向间隙补偿值） */
    PAR_CMP_BL_MPG_VALUE,			/*!< 手摇反向间隙补偿值 */

    PAR_CMP_HEAT_TYPE = 5,		/*!< 热误差补偿类型 */
    PAR_CMP_HEAT_REFN,			/*!< 热误差补偿参考点坐标 */
    PAR_CMP_HEAT_WARP_START,	/*!< 热误差偏置表起始温度 */
    PAR_CMP_HEAT_WARP_NUM,		/*!< 热误差偏置表温度点数 */
    PAR_CMP_HEAT_WARP_STEP,		/*!< 热误差偏置表温度间隔 */
    PAR_CMP_HEAT_WARP_SENSOR,	/*!< 热误差偏置表传感器编号 */
    PAR_CMP_HEAT_WARP_TABLE,	/*!< 热误差偏置表起始参数号 */
    PAR_CMP_HEAT_SLOPE_START,	/*!< 热误差斜率表起始温度 */
    PAR_CMP_HEAT_SLOPE_NUM,		/*!< 热误差斜率表温度点数 */
    PAR_CMP_HEAT_SLOPE_STEP,	/*!< 热误差斜率表温度间隔 */
    PAR_CMP_HEAT_SLOPE_SENSOR,	/*!< 热误差斜率表传感器编号 */
    PAR_CMP_HEAT_SLOPE_TABLE,	/*!< 热误差斜率表起始参数号 */
    PAR_CMP_HEAT_RATE,			/*!< 热误差补偿率 */
    PAR_CMP_HEAT_DOUBLE_TIME,   // 热误差双曲线过渡时间
    PAR_CMP_HEAT_MODE,           /*!< 热误差补偿方式 */

    PAR_CMP_PITCH_TYPE = 20,	/*!< 螺距误差补偿类型 */
    PAR_CMP_PITCH_START,		/*!< 螺距误差补偿起点坐标 */
    PAR_CMP_PITCH_NUM,			/*!< 螺距误差补偿点数 */
    PAR_CMP_PITCH_STEP,			/*!< 螺距误差补偿点间距 */
    PAR_CMP_PITCH_MODULO,		/*!< 螺距误差取模补偿使能 */
    PAR_CMP_PITCH_FACTOR,		/*!< 螺距误差补偿倍率 */
    PAR_CMP_PITCH_TABLE,		/*!< 螺距误差补偿表起始参数号 */

    PAR_CMP_SQU1_ENABLE = 30,	/*!< 第1项垂直度补偿使能 */
    PAR_CMP_SQU1_INPUT_AX,	    /*!< 第1项垂直度补偿基准轴号 */
    PAR_CMP_SQU1_REFN,	        /*!< 第1项垂直度补偿基准点坐标 */
    PAR_CMP_SQU1_ANG,	        /*!< 第1项垂直度补偿角度 */

    PAR_CMP_SQU2_ENABLE = 40,	/*!< 第2项垂直度补偿使能 */
    PAR_CMP_SQU2_INPUT_AX,	    /*!< 第2项垂直度补偿基准轴号 */
    PAR_CMP_SQU2_REFN,	        /*!< 第2项垂直度补偿基准点坐标 */
    PAR_CMP_SQU2_ANG,	        /*!< 第2项垂直度补偿角度 */

    PAR_CMP_STRA1_INPUT_AX = 50,	/*!< 第1项直线度补偿基准轴号 */
    PAR_CMP_STRA1_TYPE,	        /*!< 第1项直线度补偿类型 */
    PAR_CMP_STRA1_START,	    /*!< 第1项直线度补偿起点坐标 */
    PAR_CMP_STRA1_NUM,	        /*!< 第1项直线度补偿点数 */
    PAR_CMP_STRA1_STEP,	        /*!< 第1项直线度补偿点间距 */
    PAR_CMP_STRA1_MODULO,	    /*!< 第1项直线度取模补偿使能 */
    PAR_CMP_STRA1_FACTOR,	    /*!< 第1项直线度补偿倍率 */
    PAR_CMP_STRA1_TABLE,	    /*!< 第1项直线度补偿表起始参数号 */

    PAR_CMP_STRA2_INPUT_AX = 65,	/*!< 第2项直线度补偿基准轴号 */
    PAR_CMP_STRA2_TYPE,	        /*!< 第2项直线度补偿类型 */
    PAR_CMP_STRA2_START,	    /*!< 第2项直线度补偿起点坐标 */
    PAR_CMP_STRA2_NUM,	        /*!< 第2项直线度补偿点数 */
    PAR_CMP_STRA2_STEP,	        /*!< 第2项直线度补偿点间距 */
    PAR_CMP_STRA2_MODULO,	    /*!< 第2项直线度取模补偿使能 */
    PAR_CMP_STRA2_FACTOR,	    /*!< 第2项直线度补偿倍率 */
    PAR_CMP_STRA2_TABLE,    	/*!< 第2项直线度补偿表起始参数号 */

    PAR_CMP_ANG1_INPUT_AX = 80,	/*!< 第1项角度补偿基准轴号 */
    PAR_CMP_ANG1_ASSO_AX,	    /*!< 第1项角度补偿关联轴号 */
    PAR_CMP_ANG1_REFN,          /*!< 第1项角度补偿参考点坐标 */
    PAR_CMP_ANG1_TYPE,	        /*!< 第1项角度补偿类型 */
    PAR_CMP_ANG1_START,	        /*!< 第1项角度补偿起点坐标 */
    PAR_CMP_ANG1_NUM,	        /*!< 第1项角度补偿点数 */
    PAR_CMP_ANG1_STEP,	        /*!< 第1项角度补偿点间距 */
    PAR_CMP_ANG1_MODULO,	    /*!< 第1项角度取模补偿使能 */
    PAR_CMP_ANG1_FACTOR,	    /*!< 第1项角度补偿倍率 */
    PAR_CMP_ANG1_TABLE,	        /*!< 第1项角度补偿表起始参数号 */

    PAR_CMP_ANG2_INPUT_AX = 95,	/*!< 第2项角度补偿基准轴号 */
    PAR_CMP_ANG2_ASSO_AX,	    /*!< 第2项角度补偿关联轴号 */
    PAR_CMP_ANG2_REFN,          /*!< 第2项角度补偿参考点坐标 */
    PAR_CMP_ANG2_TYPE,	        /*!< 第2项角度补偿类型 */
    PAR_CMP_ANG2_START,	        /*!< 第2项角度补偿起点坐标 */
    PAR_CMP_ANG2_NUM,	        /*!< 第2项角度补偿点数 */
    PAR_CMP_ANG2_STEP,	        /*!< 第2项角度补偿点间距 */
    PAR_CMP_ANG2_MODULO,	    /*!< 第2项角度取模补偿使能 */
    PAR_CMP_ANG2_FACTOR,	    /*!< 第2项角度补偿倍率 */
    PAR_CMP_ANG2_TABLE,	        /*!< 第2项角度补偿表起始参数号 */

    PAR_CMP_ANG3_INPUT_AX = 110,	/*!< 第3项角度补偿基准轴号 */
    PAR_CMP_ANG3_ASSO_AX,	    /*!< 第3项角度补偿关联轴号 */
    PAR_CMP_ANG3_REFN,          /*!< 第3项角度补偿参考点坐标 */
    PAR_CMP_ANG3_TYPE,	        /*!< 第3项角度补偿类型 */
    PAR_CMP_ANG3_START,	        /*!< 第3项角度补偿起点坐标 */
    PAR_CMP_ANG3_NUM,	        /*!< 第3项角度补偿点数 */
    PAR_CMP_ANG3_STEP,	        /*!< 第3项角度补偿点间距 */
    PAR_CMP_ANG3_MODULO,	    /*!< 第3项角度取模补偿使能 */
    PAR_CMP_ANG3_FACTOR,	    /*!< 第3项角度补偿倍率 */
    PAR_CMP_ANG3_TABLE,	        /*!< 第3项角度补偿表起始参数号 */

    PAR_CMP_QUAD_ENABLE = 120,	/*!< 过象限突跳补偿类型 */
    PAR_CMP_QUAD_TRQ_VAL,		/*!< 静摩擦补偿扭矩值，取值范围：-10000~10000 */
    PAR_CMP_QUAD_ACC_T,			/*!< 过象限突跳补偿延时时间，单位：ms */
    PAR_CMP_QUAD_DEC_T,			/*!< 过象限突跳补偿减速时间，单位：ms */
    PAR_CMP_QUAD_NEG_TRQ_VAL,		/*!< 正方向（向负向运动）扭矩值，取值范围：-10000~10000 */
    PAR_CMP_QUAD_NEG_ACC_T,			/*!< 突跳正方向（向负向运动）加速时间，单位：ms */
    PAR_CMP_QUAD_NEG_DEC_T,			/*!< 突跳正方向（向负向运动）减速时间，单位：ms */
    PAR_CMP_QUAD_TRQ_MAX,		/*!< 静摩擦补偿扭矩值补偿值，取值范围：-10000~10000 */
    PAR_CMP_QUAD_FIRST_RATE,
    PAR_CMP_QUAD_SECOND_RATE,
    PAR_CMP_QUAD_THIRD_RATE,
    PAR_CMP_QUAD_TABLE,

    PAR_CMP_MULHT_TYPE = 135,		/*!< 多元线性补偿类型 */
    PAR_CMP_MULHT_REFN,				/*!< 多元线性补偿参考点坐标 */
    PAR_CMP_MULHT_BASE_WARP,		/*!< 主轴偏置模型常量 */
    PAR_CMP_MULHT_WARP_SEN_NUM,		/*!< 主轴偏置模型传感器接入个数 */
    PAR_CMP_MULHT_WARP_SEN_LIST,    /*!< 主轴偏置模型传感器编号序列 */
    PAR_CMP_MULHT_WARP_COEF_TABLE,  /*!< 主轴偏置模型系数表起始参数号 */
    PAR_CMP_MULHT_BASE_SLOPE,       /*!< 丝杆斜率模型常量 */
    PAR_CMP_MULHT_SLOPE_SEN_NUM,    /*!< 丝杆斜率模型传感器接入个数 */
    PAR_CMP_MULHT_SLOPE_SEN_LIST,   /*!< 丝杆斜率模型传感器编号序列 */
    PAR_CMP_MULHT_SLOPE_COEF_TABLE, /*!< 丝杆斜率模型系数表起始参数号 */

    PAR_CMP_RATE_ENABLE = 150,		/*!<机床反向的补偿率，在反向间隙与双向螺补时生效 */
    PAR_CMP_RATE_LOW,				/*!<低速时的补偿率，半径100，F1000时的补偿率 */
    PAR_CMP_RATE_HIGH,				/*!<高速时的补偿率，半径100，F5000时的补偿率 */
    PAR_CMP_RATE_MAX,				/*!<最大补偿率 */

    PAR_CMP_FEEDFOR_ENABLE,			/*!<前馈补偿 */
    PAR_CMP_FEEDFOR_RATE,			/*!<前馈补偿补偿系数 */
    PAR_CMP_FEEDFOR_CYCLE,			/*!<前馈补偿提前周期 */

    PAR_CMP_TT_TYPE=157,
    Par_CMP_TT_RATE,
    PAR_CMP_TT_MAX,
    PAR_CMP_TT_UP_TIME,
    PAR_CMP_TT_DELAY,
    PAR_CMP_TT_DOWN_TIME,

    PAR_CMP_NOSENSORHEAT_TYPE,			/*!< 无温度传感器误差补偿类型 */
    PAR_CMP_NOSENSORHEAT_ONE,			/*!< 无温度传感器误差补偿系数1 */
    PAR_CMP_NOSENSORHEAT_TWO,			/*!< 无温度传感器误差补偿系数2 */
    PAR_CMP_NOSENSORHEAT_THREE,			/*!< 无温度传感器误差补偿系数3 */
    PAR_CMP_NOSENSORHEAT_FOUR,			/*!< 无温度传感器误差补偿系数4 */
    PAR_CMP_NOSENSORHEAT_SPDL,			/*!< 无温度传感器误差补偿关联主轴 */
    PAR_CMP_NOSENSORHEAT_SPDL_ONE,      /*!< 无温度传感器误差补偿主轴系数1 */
    PAR_CMP_NOSENSORHEAT_SPDL_TWO,      /*!< 无温度传感器误差补偿主轴系数2 */
    PAR_CMP_NOSENSORHEAT_SPDL_THREE,    /*!< 无温度传感器误差补偿主轴系数3 */
    PAR_CMP_NOSENSORHEAT_REFN,			/*!< 无温度传感器误差补偿膨胀零点 */
    PAR_CMP_NOSENSORHEAT_DST,			/*!< 无温度传感器误差补偿目标点 */

    PAR_CMP_PHYSICALTHERMAL_START,      /*!< 热机理热误差模型起点坐标（mm或度） */
    PAR_CMP_PHYSICALTHERMAL_NUM,      /*!< 热机理热误差补偿点数 */
    PAR_CMP_PHYSICALTHERMAL_STEP,      /*!< 热机理热误差补偿点间距（mm或度） */
    PAR_CMP_PHYSICALTHERMAL_TABLE,      /*!< 热机理热误差补偿表起始参数号 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER1,      /*!< 热机理模型参数1 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER2,      /*!< 热机理模型参数2 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER3,      /*!< 热机理模型参数3 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER4,      /*!< 热机理模型参数4 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER5,      /*!< 热机理模型参数3 */
    PAR_CMP_PHYSICALTHERMAL_PARAMETER6,      /*!< 热机理模型参数4 */

    PAR_CMP_SPECIAL = 160,	/*!< 专机预留 */ /*!<扩展参数起始地址 */
    PAR_CMP_Z_AXIS_HEAT_HEAT_WARP_START = 185,/*!< Z轴热误差偏置表起始温度 */
    PAR_CMP_Z_AXIS_HEAT_HEAT_WARP_NUM,		 /*!< Z轴热误差偏置表温度点数 */
    PAR_CMP_Z_AXIS_HEAT_HEAT_WARP_STEP,	     /*!< Z轴热误差偏置表温度间隔 */
    PAR_CMP_Z_AXIS_HEAT_HEAT_WARP_SENSOR,	     /*!< Z轴热误差偏置表传感器编号 */
    PAR_CMP_Z_AXIS_HEAT_HEAT_WARP_TABLE,	     /*!< Z轴热误差偏置表起始参数号 */

    PAR_CMP_HEAT_TYPE_2=192,   		/*!< 热误差第二组补偿类型 */
    PAR_CMP_HEAT_REFN_2,			/*!< 热误差第二组补偿参考点坐标 */
    PAR_CMP_HEAT_WARP_START_2,	    /*!< 热误差第二组偏置表起始温度 */
    PAR_CMP_HEAT_WARP_NUM_2,		/*!< 热误差第二组偏置表温度点数 */
    PAR_CMP_HEAT_WARP_STEP_2,		/*!< 热误差第二组偏置表温度间隔 */
    PAR_CMP_HEAT_WARP_SENSOR_2,	    /*!< 热误差第二组偏置表传感器编号 */
    PAR_CMP_HEAT_WARP_TABLE_2,	    /*!< 热误差第二组偏置表起始参数号 */
    PAR_CMP_HEAT_SLOPE_START_2,	    /*!< 热误差第二组斜率表起始温度 */
    PAR_CMP_HEAT_SLOPE_NUM_2,		/*!< 热误差第二组斜率表温度点数 */
    PAR_CMP_HEAT_SLOPE_STEP_2,	    /*!< 热误差第二组斜率表温度间隔 */
    PAR_CMP_HEAT_SLOPE_SENSOR_2,	/*!< 热误差第二组斜率表传感器编号 */
    PAR_CMP_HEAT_SLOPE_TABLE_2,	    /*!< 热误差第二组斜率表起始参数号 */
    PAR_CMP_HEAT_RATE_2,			/*!< 热误差第二组补偿率 */
    PAR_CMP_HEAT_DOUBLE_TIME_2,     /*!< 热误差第二组双曲线过渡时间 */

    /* 空闲4个，可用 */

    PAR_CMP_VCS_START = 210,	/*!< 空间误差项补偿起点坐标 */
    PAR_CMP_VCS_NUM,			/*!< 空间误差项补偿点数 */
    PAR_CMP_VCS_STEP,			/*!< 空间误差项补偿点间距 */
    PAR_CMP_VCS_MODULO,			/*!< 空间误差项取模补偿使能 */
    PAR_CMP_VCS_FACTOR,			/*!< 空间误差项补偿倍率 */
    PAR_CMP_VCS_TABLE,			/*!< 空间误差项补偿表起始参数号 */

    PAR_CMP_VCS_max_Ps,			/*最大幅值约束*/
    PAR_CMP_VCS_max_Sp,			/*最大速度约束*/
    PAR_CMP_VCS_max_Acc,		/*最大加速度约束*/
    PAR_CMP_VCS_max_Sw,			/*最大捷度约束*/

    PAR_CMP_SURR_SPIN,           /*!< 工作台或者刀具是否安装 */
    PAR_CMP_SURR_BASE_WARP_TABLE,		/*!< 主轴偏置代理模型常量 */
    PAR_CMP_SURR_WARP_SEN_NUM,		/*!< 主轴偏置代理模型传感器接入个数 */
    PAR_CMP_SURR_WARP_COEF_TABLE,  /*!< 主轴偏置代理模型系数表起始参数号 */

    PAR_CMP_TOTAL = 500		/*!< 轴补偿参数总个数 */
};

/*! 设备接口参数 */
enum
{
    PAR_DEV_NAME,				/*!< 设备名称 */
    PAR_DEV_INDEX,				/*!< 设备的系统序号，在系统全部设备中的序号 */
    PAR_DEV_TYPE,				/*!< 设备类型 */
    PAR_DEV_GRP_IDX,			/*!< 在同组设备中的序号 */
    PAR_DEV_ID,					/*!< 设备ID[生产唯一] */
    PAR_DEV_VENDOR,				/*!< 生产商 */
    PAR_DEV_SNCODE,				/*!< 注册码 */
    PAR_DEV_READONLY_NUM = 8,	/*!< 保留 */
    PAR_DEV_MODE,				/*!< 设备数据字长 */
    PAR_DEV_GNL_NUM				/*!< [设备通用参数的个数] */
};


/*! 设备--主站设备 */
enum
{
    /*!
     * @name 系统固化参数
     * @{
     */
    PAR_DEV_BRD_FPGA_VER = PAR_DEV_GNL_NUM,	/*!< FPGA固件程序版本号 */
    PAR_DEV_BRD_CARD_VER, /*!< 主站卡版本号 */
    /*!@}*/

    PAR_DEV_BRD_SYS_OBJ_NUM = (PAR_DEV_GNL_NUM+10),	/*!< 本地控制对象个数，保留+追加 */
    PAR_DEV_BRD_NET_OBJ_NUM,	/*!< 总线从站控制对象个数 */
    PAR_DEV_BRD_OBJ_NUM,		/*!< 控制对象总个数，本地+总线从站 */

    /*!
     * @name 用户配置参数
     * @{
     */
    PAR_DEV_BRD_BUS_CYCLE = (PAR_DEV_GNL_NUM+40),	/*!< 总线通讯周期 */
    PAR_DEV_BRD_BUS_REQ_TIMES,	/*!< 总线通讯请求次数 */
    PAR_DEV_BRD_BUS_TOPO,		/*!< 拓扑结构（保留） */
    /*!@}*/

    PAR_DEV_BRD_SP_ADD_NUM = (PAR_DEV_GNL_NUM+50),	/*!< 追加模拟量主轴数 */

    PAR_DEV_BRD_RESV_TYPE = (PAR_DEV_GNL_NUM+60)	/*!< 本地保留设备类型，预留10个，可扩展 */

};

/*! 设备--AX */
enum
{
    PAR_DEV_AX_MODE = PAR_DEV_GNL_NUM,	/*!< 工作模式 */
    PAR_DEV_AX_IDX,			/*!< 设备对应的逻辑轴号 */
    PAR_DEV_AX_ENCOD_DIR,	/*!< 编码器反馈取反标志 */
    PAR_DEV_AX_CMD_TYPE,	/*!< 主轴DA输出类型 */
    PAR_DEV_AX_CYC_EN,      /*!< 反馈位置循环使能 */
    PAR_DEV_AX_MT_PPR,		/*!< 反馈位置循环脉冲数 */
    PAR_DEV_AX_ENCOD_TYPE,  /*!< 编码器类型 */
    PAR_DEV_AX_RESERVE1,	/*!< 保留1 */
    PAR_DEV_AX_RESERVE2,	/*!< 保留2 */
    PAR_DEV_AX_RESERVE3     /*!< 保留3 */
};

/*! 设备--MPG */
enum
{
    PAR_DEV_MPG_TYPE = PAR_DEV_GNL_NUM,	/*!< MPG类型 */
    PAR_DEV_MPG_IDX,					/*!< MPG编号 */
    PAR_DEV_MPG_IN,						/*!< 档位输入点组号 */
    PAR_DEV_MPG_DIR_FLAG,				/*!< 各轴方向取反标志 */
    PAR_DEV_MPG_MULT_FACTOR,			/*!< 倍率放大系数 */
    PAR_DEV_MPG_PAR_NUM					/*!< MPG实际配置参数数目 */
};

/*! 设备--NCKB */
enum
{
    PAR_DEV_NCKB_TYPE = PAR_DEV_GNL_NUM,	/*!< NCKB键盘类型 */
    PAR_DEV_NCKB_IDX,		/*!< NCKB编号 */
    PAR_DEV_NCKB_CYCLE,		/*!< NCKB扫描周期 */
    PAR_DEV_NCKB_PAR_NUM	/*!< NCKB实际配置参数数目 */
};

/*! 设备--IO_LOC/IO_NET */
enum
{
    PAR_DEV_IO_WATCHDOG = PAR_DEV_GNL_NUM,	/*!< 是否包含看门狗（保留） */
    PAR_DEV_IO_AXIS_CTRL,	/*!< 是否是轴控制板（保留） */
    PAR_DEV_IN_X_BASE,		/*!< 输入点X寄存器起始组号(PAR_DEV_GNL_NUM+2) */
    PAR_DEV_IN_X_GRPN,		/*!< 输入点组数 */
    PAR_DEV_OUT_Y_BASE,		/*!< 输出点Y寄存器起始组号 */
    PAR_DEV_OUT_Y_GRPN,		/*!< 输出点组数 */
    PAR_DEV_ENCOD1_TYPE,	/*!< 编码器1类型 */
    PAR_DEV_ENCOD1_PPR,		/*!< 编码器1每转脉冲数 */
    PAR_DEV_ENCOD2_TYPE,	/*!< 编码器2类型 */
    PAR_DEV_ENCOD2_PPR,		/*!< 编码器2每转脉冲数 */
};



/*! 设备--MCP_LOC/MCP_NET */
enum
{
    PAR_DEV_MCP_TYPE = PAR_DEV_GNL_NUM,	/*!< MCP类型：1-A / 2-B /3-C */
    PAR_DEV_MCP_MPG_IDX,	/*!< 手摇编号 */
    PAR_DEV_MCP_X_BASE,     /*!< 输入点X寄存器起始组号(PAR_DEV_GNL_NUM+2) */
    PAR_DEV_MCP_X_GRPN,     /*!< 输入点占用X寄存器组数 */
    PAR_DEV_MCP_Y_BASE,     /*!< 输出点寄存器起址 */
    PAR_DEV_MCP_Y_GRPN,     /*!< 输出点占用Y寄存器组数 */
    PAR_DEV_MCP_MPG_DIR,    /*!< 手摇方向取反标志 */
    PAR_DEV_MCP_MPG_MULT,   /*!< 手摇倍率放大系数 */
    PAR_DEV_MCP_CODE_TYPE,  /*!< 波段开关编码类型 */
    PAR_DEV_MCP_SPDL_NUM    /*!< 追加模拟量主轴数（temp） */
};

/*! 设备--SERIAL */
enum
{
    PAR_SERIAL_BIT_LEN = PAR_DEV_GNL_NUM,   /*!< 收发数据位长度 */
    PAR_SERIAL_STOP,		/*!< 停止位 */
    PAR_SERIAL_PARITY,		/*!< 奇偶校验位 */
    PAR_SERIAL_BAUDRATE,	/*!< 波特率 */
    PAR_SERIAL_PAR_NUM		/*!< SERIAL实际配置参数数目 */
};

/*! 设备--LAN */
enum
{
    PAR_LAN_IP0 = PAR_DEV_GNL_NUM, /*!< IP0 */
    // PAR_LAN_IP1,	// IP1
    // PAR_LAN_IP2,	// IP2
    // PAR_LAN_IP3,	// IP3
    PAR_LAN_GATE0,	/*!< GATE0 */
    // PAR_LAN_GATE1,	// GATE1
    // PAR_LAN_GATE2,	// GATE2
    // PAR_LAN_GATE3,	// GATE3
    PAR_LAN_MASK0,	    /*!< MASK0 */
    // PAR_LAN_MASK1,	// MASK1
    // PAR_LAN_MASK2,	// MASK2
    // PAR_LAN_MASK3,	// MASK3
    PAR_LAN_PAR_NUM     /*!< LAN实际配置参数数目 */
};

/*! 设备--WCOM无线通讯接口 */
enum
{
    PAR_WCOM_TYPE = PAR_DEV_GNL_NUM, /*!< WCOM类型 */
    PAR_WCOM_RESERVE,	/*!< 保留 */
    PAR_WCOM_PAR_NUM	/*!< WCOM实际配置参数数目 */
};

/*! 设备--GATHER采集卡 */
enum
{
    PAR_GATHER_TYPE = PAR_DEV_GNL_NUM,	/*!< 采集卡类型 1:KZM-6000 */
    PAR_GATHER_SERIAL_IDX,    /*!< 串口设备号 */
    PAR_GATHER_SENSOR_NUM,    /*!< 传感器数 */
    PAR_GATHER_IN_X_BASE,     /*!< 输入点起始组号 */
    PAR_GATHER_PAR_NUM        /*!< GATHER实际配置参数数目 */
};

/*! 数控设备控制对象应用类型定义，\n
    系统根据其类型和接口参数确定控制对象在系统中的输入输出数据地址 */
enum DevNcobjType
{
    DEV_NCOBJ_NULL_LOC = 1000,	/*!< 本地设备--非网络设备 */
    DEV_NCOBJ_SPDL_LOC,
    DEV_NCOBJ_AXIS_LOC,
    DEV_NCOBJ_IN_LOC,
    DEV_NCOBJ_OUT_LOC,
    DEV_NCOBJ_AD_LOC,
    DEV_NCOBJ_DA_LOC,
    DEV_NCOBJ_IOMD_LOC,		/*!< NCUC总线的IO集成模块 */
    DEV_NCOBJ_MCP_LOC,
    DEV_NCOBJ_MPG_LOC,
    DEV_NCOBJ_NCKB_LOC,
    DEV_NCOBJ_SENSOR_LOC,	/*!< 传感器设备 */
    DEV_NCOBJ_SERIAL_LOC,	/*!< 串口设备 */
    DEV_NCOBJ_GATHER_LOC,	/*!< 温度采集卡 */
    DEV_NCOBJ_1210_LOC,	    /*!<2.40占位 HIO1210板卡 */

    DEV_NCOBJ_NULL_NET = 2000,	/*!< 网络设备--ncuc\ethercat\syqnet... */
    DEV_NCOBJ_SPDL_NET,
    DEV_NCOBJ_AXIS_NET,
    DEV_NCOBJ_IN_NET,
    DEV_NCOBJ_OUT_NET,
    DEV_NCOBJ_AD_NET,
    DEV_NCOBJ_DA_NET,
    DEV_NCOBJ_IOMD_NET,		/*!< NCUC总线的IO集成模块 */
    DEV_NCOBJ_MCP_NET,
    DEV_NCOBJ_MPG_NET,
    DEV_NCOBJ_NCKB_NET,
    DEV_NCOBJ_SENSOR_NET,	/*!< 传感器 */
    DEV_NCOBJ_PIDC_NET,		/*!< 位控板 */

    /*!
     * @name 此处扩展新的类型
     * @{
     */
    DEV_NCOBJ_ENCOD_NET		/*!< 编码器 */
    /*!@}*/
};

/*! 刀具参数 \n
 2.1 基本数据范围 */
#define MAX_GEO_PARA	(24)	/*!< 刀具几何参数个数 */
#define MAX_WEAR_PARA	(24)	/*!< 刀具磨损参数个数 */
#define MAX_TOOL_EXPARA	(24)	/*!< 刀具扩展参数个数 */
#define MAX_TOOL_MONITOR (36)	/*!< 刀具监控参数个数 */
#define MAX_TOOL_BASE	(24)	/*!< 刀具一般信息参数个数 */
#define MAX_TOOL_BREAK	(24)	/*!< 刀具断刀检测参数个数 */
#define MAX_TOOL_PARA	(200)	/*!< 刀具基本参数个数 (24+24+24+36+24+24 = 156) */
#define MAGZ_HEAD_SIZE	(16)	/*!< 刀库数据表头大小 */
#define MAX_TOOL_NAME_LEN (20)  /*!< 刀具名称最大长度 */

/*! 刀具参数索引 \n
 INFTOOL_开头的为Bit32，其它为fBit64 */
enum ToolParaIndex
{
    /*!
     * @name 刀具几何相关参数索引
     * @{
     */
    GTOOL_DIR = 0,	/*!< 方向  */
    GTOOL_LEN1,	/*!< 长度1(铣：刀具长度z；车：X偏置) */
    GTOOL_LEN2,	/*!< 长度2(铣：刀具长度x；车：Y偏置) */
    GTOOL_LEN3,	/*!< 长度3(铣：刀具长度y；车：Z偏置) */
    GTOOL_LEN4,	/*!< 长度4 */
    GTOOL_LEN5,	/*!< 长度5 */
    GTOOL_RAD1,	/*!< 半径1(铣：刀具半径；车：刀尖半径) */
    GTOOL_RAD2,	/*!< 半径2 */
    GTOOL_ANG1,	/*!< 角度1 */
    GTOOL_ANG2,	/*!< 角度2 */
    /*!@}*/

    GTOOL_BASE_LEN1,/*!<长度1(铣：刀具基体长度z) */
    GTOOL_BASE_LEN2,/*!<长度2(铣：刀具基体长度x) */
    GTOOL_BASE_LEN3,/*!<长度3(铣：刀具基体长度y) */

    GTOOL_BASE_XDONE,/*!<X试切标志 */
    GTOOL_BASE_YDONE,/*!<Y试切标志 */
    GTOOL_BASE_ZDONE,/*!<Z试切标志 */
    GTOOL_LEN_A,/*!< 长度6(铣：A轴刀具长度；车：A轴偏置) */
    EXTOOL_GTOOL_LEN1_LIMIT, /*!< 刀具偏置：长度(豪迈刀补表) */
    EXTOOL_GTOOL_RAD1_LIMIT, /*!< 刀具偏置：半径(豪迈刀补表) */
    GTOOL_TOTAL,

    /*!
     * @name 刀具磨损相关参数索引
     * @{
     */
    WTOOL_LEN1 = MAX_GEO_PARA, /*!< (铣：长度磨损z；车：Z磨损) */
    WTOOL_LEN2,	/*!< 长度2(铣：长度磨损x) */
    WTOOL_LEN3,	/*!< 长度3(铣：长度磨损y) */
    WTOOL_LEN4,	/*!< 长度4 */
    WTOOL_LEN5,	/*!< 长度5 */
    WTOOL_RAD1,	/*!< 半径1(铣：半径磨损；车：X磨损) */
    WTOOL_RAD2,	/*!< 半径2 */
    WTOOL_ANG1,	/*!< 角度1 */
    WTOOL_ANG2,	/*!< 角度2 */
    WTOOL_TOOL_LEN_SET_X,	/*!< 走心机刀具刀长设定X */
    WTOOL_TOOL_LEN_SET_Y,	/*!< 走心机刀具刀长设定Y */
    WTOOL_TOOL_LEN_SET_Z,	/*!< 走心机刀具刀长设定Z */
    WTOOL_TOOL_LEN_FIX_X,	/*!< 走心机刀具刀长修正X */
    WTOOL_TOOL_LEN_FIX_Y,	/*!< 走心机刀具刀长修正Y */
    WTOOL_TOOL_LEN_FIX_Z,	/*!< 走心机刀具刀长修正Z */
    WTOOL_LEN_A,/*!< (铣：A轴长度磨损；车：A轴磨损) */
    EXTOOL_WTOOL_LEN1_LIMIT, /*!< 磨损允差：长度(豪迈刀补表) */
    EXTOOL_WTOOL_RAD1_LIMIT, /*!< 磨损允差：半径(豪迈刀补表) */
    /*!@}*/

    WTOOL_TOTAL,

    /*!
     * @name 刀具扩展参数--刀具管理参数，各刀具类型通用
     * @{
     */
    TETOOL_SP_TRQ_TARGET = MAX_GEO_PARA+MAX_WEAR_PARA, /*!< 工艺相关参数0～9 */
    TETOOL_SP_TRQ_TOLERANCE,
    TETOOL_AX_TRQ_TARGET,
    TETOOL_AX_TRQ_TOLERANCE,
    TETOOL_PARA4,
    TETOOL_PARA5,
    TETOOL_PARA6,
    TETOOL_PARA7,
    TETOOL_PARA8,
    TETOOL_PARA9,

    METOOL_PARA0, /*!< 测量参数0～9 */
    METOOL_PARA1,
    METOOL_PARA2,
    METOOL_PARA3,
    METOOL_PARA4,
    METOOL_PARA5,
    METOOL_PARA6,
    METOOL_PARA7,
    METOOL_PARA8,
    METOOL_PARA9,
    /*!@}*/

    EXTOOL_TOTAL,

    /*!
     * @name 刀具监控参数
     * @{
     */
    MOTOOL_TYPE = MAX_GEO_PARA+MAX_WEAR_PARA+MAX_TOOL_EXPARA, /*!< 刀具监控类型，按位有效，可选多种监控方式同时监控 0:关闭 0x01:安装次数 0x02:切削时间 0x04:磨损 0x08:切削里程 0x10:切削能耗 0x20:主轴转数 */
    MOTOOL_GROUP,       /*!< 刀具所属分组号 */
    MOTOOL_RESAULT_LIFE,/*!< 综合寿命(0~100) */
    MOTOOL_WAR_PER,		/*!< 预警比例(0~100) */
    MOTOOL_ALM_PER,		/*!< 报警比例(0~100) */

    MOTOOL_MAX_LIFE1,	/*!< 最大寿命 */
    MOTOOL_ALM_LIFE1,	/*!< 预警寿命 */
    MOTOOL_ACT_LIFE1,	/*!< 实际寿命 */
    MOTOOL_ACT_LIFE_LO1,/*!< 实际寿命(保留) */
    MOTOOL_CNT_UNIT1,	/*!< 计数单位 切削里程0:米 1:百米 2:千米，切削能耗0:瓦时 1:千瓦时，主轴转数0:默认 1:千 2:万 3:十万 4:百万 5:千万 6:亿 */
    MOTOOL_WEIGHT1,		/*!< 权重(0~100) */

    MOTOOL_MAX_LIFE2,	/*!< 最大寿命 */
    MOTOOL_ALM_LIFE2,	/*!< 预警寿命 */
    MOTOOL_ACT_LIFE2,	/*!< 实际寿命 */
    MOTOOL_ACT_LIFE_LO2,/*!< 实际寿命(保留) */
    MOTOOL_CNT_UNIT2,	/*!< 计数单位 切削里程0:米 1:百米 2:千米，切削能耗0:瓦时 1:千瓦时，主轴转数0:默认 1:千 2:万 3:十万 4:百万 5:千万 6:亿 */
    MOTOOL_WEIGHT2,		/*!< 权重(0~100) */

    MOTOOL_MAX_LIFE3,	/*!< 最大寿命 */
    MOTOOL_ALM_LIFE3,	/*!< 预警寿命 */
    MOTOOL_ACT_LIFE3,	/*!< 实际寿命 */
    MOTOOL_ACT_LIFE_LO3,/*!< 实际寿命(保留) */
    MOTOOL_CNT_UNIT3,	/*!< 计数单位 切削里程0:米 1:百米 2:千米，切削能耗0:瓦时 1:千瓦时，主轴转数0:默认 1:千 2:万 3:十万 4:百万 5:千万 6:亿 */
    MOTOOL_WEIGHT3,		/*!< 权重(0~100) */

    MOTOOL_MAX_LIFE4,	/*!< 最大寿命 */
    MOTOOL_ALM_LIFE4,	/*!< 预警寿命 */
    MOTOOL_ACT_LIFE4,	/*!< 实际寿命 */
    MOTOOL_ACT_LIFE_LO4,/*!< 实际寿命(保留) */
    MOTOOL_CNT_UNIT4,	/*!< 计数单位 切削里程0:米 1:百米 2:千米，切削能耗0:瓦时 1:千瓦时，主轴转数0:默认 1:千 2:万 3:十万 4:百万 5:千万 6:亿 */
    MOTOOL_WEIGHT4,		/*!< 权重(0~100) */

    MOTOOL_MAX_LIFE5,	/*!< 最大寿命 */
    MOTOOL_ALM_LIFE5,	/*!< 预警寿命 */
    MOTOOL_ACT_LIFE5,	/*!< 实际寿命 */
    MOTOOL_ACT_LIFE_LO5,/*!< 实际寿命(保留) */
    MOTOOL_CNT_UNIT5,	/*!< 计数单位 切削里程0:米 1:百米 2:千米，切削能耗0:瓦时 1:千瓦时，主轴转数0:默认 1:千 2:万 3:十万 4:百万 5:千万 6:亿 */
    MOTOOL_WEIGHT5,		/*!< 权重(0~100) */
    /*! 最多可同时设置监控5种方式，若超过5种则仅监控前五个 */
    /*!@}*/

    MOTOOL_TOTAL,

    /*!
     * @name 刀具一般信息
     * @{
     */
    INFTOOL_ID = MAX_GEO_PARA+MAX_WEAR_PARA+MAX_TOOL_EXPARA+MAX_TOOL_MONITOR, /*!< 刀具索引号 */
    INFTOOL_MAGZ,		/*!< 	刀具所属刀库号 */
    INFTOOL_CH,			/*!< 	刀具所属通道号 */
    INFTOOL_TYPE,		/*!< 	刀具类型 */
    INFTOOL_STATE,		/*!< 	刀具状态字 */
    INFTOOL_G64MODE,	/*!< 	刀具高速高精加工模式 */
    INFTOOL_SIZE,		/*!<    刀具规格*/
    TOOL_INFO_NAME,		/*!<	刀具名称 */
    EXTOOL_S_LIMIT,     /*!< S转速限制 */
    EXTOOL_F_LIMIT,     /*!< F转速限制 */
    EXTOOL_LARGE_LEFT,	/*!< 大刀具干涉左刀位 */
    EXTOOL_LARGE_RIGHT,	/*!< 大刀具干涩右刀位 */
    EXTOOL_LTOOL_PARM, /*!< 车刀,0x01:旋转方向,0x02刀具编辑锁定,0x04背主轴选择 */
    /*!@}*/
    INFTOOL_TOTAL,

    /*!
     * @name 刀具断刀检测
     * @{
     */
    BKTOOL_REG = MAX_GEO_PARA+MAX_WEAR_PARA+MAX_TOOL_EXPARA+MAX_TOOL_MONITOR+MAX_TOOL_BASE, //M201寄存器号和点位
    BKTOOL_TRIGGER, /*!< 检测到断刀触发 0:正常 1:断刀 */
    BKTOOL_BACK, /*!< 反馈 0:等待反馈 1:正确 2:误报 */
    BKTOOL_POWER,//比率阈值
    BKTOOL_FILTER,//滤波强度
    BKTOOL_TOTAL,

    TOOL1_X1 = MAX_GEO_PARA+MAX_WEAR_PARA+MAX_TOOL_EXPARA+MAX_TOOL_MONITOR+MAX_TOOL_BASE+MAX_TOOL_BREAK,  /*!< 刀具形状数据 */
    TOOL1_Y1,
    TOOL1_Z1,
    TOOL1_I1,
    TOOL1_J1,
    TOOL1_K1,
    TOOL1_X2,
    TOOL1_Y2,
    TOOL1_Z2,
    TOOL1_I2,
    TOOL1_J2,
    TOOL1_K2,

    TOOL_PARA_TOTAL /*!< MAX_TOOL_PARA */
};

/*! 刀库表表头数据索引 */
enum MagzHeadIndex
{
    MAGZTAB_HEAD = 0,	/*!< 刀库表起始偏移地址（刀具号段+刀位属性段） */
    MAGZTAB_TOOL_NUM,	/*!< 刀库表中刀具数 */
    MAGZTAB_CUR_TOOL,	/*!< 当前刀具号 */
    MAGZTAB_CUR_POT,	/*!< 当前刀位号 */
    MAGZTAB_REF_TOOL,	/*!< 标刀号 */
    MAGZTAB_CHAN,		/*!< 刀库所属通道号 */
    MAGZTAB_TYPE,		/*!< 刀库类型 */
    SWAP_LEFT_TOOL,		/*!< 机械手左刀位刀具号 */
    SWAP_RIGHT_TOOL,	/*!< 机械手右刀位刀具号 */
    MAGZTAB_RETURN,		/*!< 刀库还刀方式 0:定点 1:随机 2:大刀定点小刀随机 3:大小刀随机*/
    MAGZTAB_WAIT,		/*!< 中间待刀位 */
    MAGZTAB_CUR_TOOL_SEC,/*!< 第二个当前刀具号(车铣复合用) */
    MAGZTAB_START_TOOLNO,/*!< 刀库初始化的起始刀具号(多刀库有效) */
    MAGZTAB_TOOL_LOAD,    /*!< 刀库装卸刀位 */
    /*! 预留3个 */

    MAGZTAB_TOTAL
};

/*! 刀库类型*/
enum _MAG_TYPE_
{
    MAG_TYPE_NON = 0,					//(0:无刀库)
    MAG_TYPE_UMBRELLA,					//(1:斗笠式刀库)
    MAG_TYPE_DISC,						//(2:圆盘刀库)
    MAG_TYPE_CHAIN,						//(3:链式刀库)
    MAG_TYPE_DRILLING,					//(4:钻攻计数刀库)
    MAG_TYPE_PMC,						//(5:钻攻PMC伺服刀库)
    MAG_TYPE_TDS,						//(6:钻攻TDS刀库:第三方刀库)
    MAG_TYPE_ARM,                       //(7:机械手伺服刀库)
    MAG_TYPE_COUNT
};

/*! 还刀策略类型(还刀方式使用还刀策略枚举类型) */
typedef enum MagzReturnType
{
    MAGZ_RETURN_FIXED = 0,			/*!< 定点:主轴上无刀或者可以与预选刀直接交换 */
    MAGZ_RETURN_RNADOM,				/*!< 随机:大小刀功能开启:大刀随机更换同属性的刀;大小刀功能关闭:大小刀随机更换 */
    MAGZ_RETURN_BF_SR,				/*!< 大刀定点小刀随机,大小刀功能开启有效 */
    MAGZ_RETURN_BS_RANDOM			/*!< 大小刀随机，大小刀功能开启有效 */
}ENMagzReturnType;

/*! INFTOOL_STATE刀具状态字*/
#define TOOL_ATTRI		0x0000FFFF		// 刀具属性(低16位)
#define RETURN_TYPE		0x00010000		// 还刀方式(第16位, 0:非定点还刀;1:定点还刀)

/*!
 * @name 刀具参数具体化（兼容以前的刀具宏）
 * @{
 */
/*!
 * @name 铣刀参数索引
 * @{
 */
#define MTOOL_RAD		GTOOL_RAD1	/*!< 刀具半径 */
#define MTOOL_LEN		GTOOL_LEN1	/*!< 刀具长度 */
#define MTOOL_RAD_WEAR	WTOOL_RAD1	/*!< 铣刀:半径磨损补偿（径向） */
#define MTOOL_LEN_WEAR	WTOOL_LEN1	/*!< 铣刀:长度磨损补偿（轴向） */
/*!@}*/

/*!
 * @name 刀具参数具体化（兼容以前的刀具宏） 车刀参数索引
 * @{
 */
#define LTOOL_RAD		GTOOL_RAD1		/*!< 刀尖半径 */
#define LTOOL_DIR		GTOOL_DIR		/*!< 刀尖方向 */
#define LTOOL_RAD_WEAR	WTOOL_RAD1		/*!< 车刀:刀具磨损值（径向）（相对值） */
#define LTOOL_LEN_WEAR	WTOOL_LEN1		/*!< 车刀:刀具磨损值（轴向）（相对值） */
#define LTOOL_XOFF		GTOOL_LEN1		/*!< 车刀：刀具偏置值（径向）（绝对值） =  试切时X值 - 试切直径/2 */
#define LTOOL_YOFF		GTOOL_LEN2
#define LTOOL_ZOFF		GTOOL_LEN3		/*!< 车刀：刀具偏置值（轴向）（绝对值） =  试切时Z值 - 试切长度 */
#define LTOOL_XDONE		GTOOL_BASE_XDONE/*!< X试切标志 */
#define LTOOL_YDONE		GTOOL_BASE_YDONE/*!< Y试切标志 */
#define LTOOL_ZDONE		GTOOL_BASE_ZDONE/*!< Z试切标志 */
#define LTOOL_MEAS_X	METOOL_PARA0	/*!< 刀具测量X长度 */
#define LTOOL_MEAS_Y	METOOL_PARA1	/*!< 刀具测量Y长度 */
#define LTOOL_MEAS_Z	METOOL_PARA2	/*!< 刀具测量Z长度 */
/*!@}*/

/*!
 * @name 刀具参数具体化（兼容以前的刀具宏） 车铣复合参数索引
 * @{
 */
#define MT_TOOL_XOFF		GTOOL_LEN2		/*!< 长度X */
#define MT_TOOL_YOFF		GTOOL_LEN3		/*!< 长度Y */
#define MT_TOOL_ZOFF		GTOOL_LEN1		/*!< 长度Z/刀具长度 */
/*!@}*/
/*!@}*/

#define SPDL_RESOLUTION	(1000)	/*!< 主轴转速分辨率 */
#define MAX_ROW_IN_ONE_SCAN         50   /*!<任意行扫描模式一个周期最大扫描行数 */
#define MAX_DECODE_BLOCK MAX_ROW_IN_ONE_SCAN /*!<快速校验模式一个周期最大扫描行数 */
#define GIVEN_ROW_IDLE				0
#define GIVEN_ROW_WAIT_PROG_OK		1     /*!<等待任意行程序准备好 */
#define GIVEN_ROW_SCANING			2     /*!<任意行扫描中 */
#define GIVEN_ROW_WAIT_SCAN_ACK		3     /*!<任意行等待界面给出应答后再向后继续扫描 */
#define GIVEN_ROW_WAIT_SUBPROG_OK	4     /*!<等待任意行子程序准备好 */
#define GIVEN_ROW_SUBPROG_OK		5     /*!<任意行子程序准备好 */
#define GIVEN_ROW_SUBPROG_EXCUTING	6    /*!<任意行子程序执行中 */
#define GIVEN_ROW_SCAN_ERR			7    /*!<任意行扫描中发现语法错误 */
#define GIVEN_ROW_TYPE1_WAIT_Z_MOVE 8    /*!<任意行扫描模式1，等待Z轴移动指令 */
#define GIVEN_ROW_WAITING_FOR_CYCLING 9  /*!<任意行程序执行完成，等待第二次循环启动 */
/* 急停/断电断点恢复流程状态机，与原任意行扫描方式分开处理 */
#define ESTOP_BP_IDLE				0
#define ESTOP_BP_WAIT_PROG_OK		1     /*!<等待任意行程序准备好 */
#define ESTOP_BP_SCANING			2     /*!<任意行扫描中 */
#define ESTOP_BP_WAIT_SCAN_ACK		3     /*!<任意行等待界面给出应答后再向后继续扫描 */
#define ESTOP_BP_WAIT_SUBPROG_OK	4     /*!<等待任意行子程序准备好 */
#define ESTOP_BP_SUBPROG_OK		5     /*!<任意行子程序准备好 */
#define ESTOP_BP_SUBPROG_EXCUTING	6    /*!<任意行子程序执行中 */
#define ESTOP_BP_SCAN_ERR			7    /*!<任意行扫描中发现语法错误 */
#define ESTOP_BP_TYPE1_WAIT_Z_MOVE 8    /*!<任意行扫描模式1，等待Z轴移动指令 */

/*!
 * @name 采样
 * @{
 */
/*!
 * @name 采样数据宏定义
 * @{
 */
#define SMPL_MAX_CHAN_NUM (64)	/*!< 最大采样通道数。PS：内存上预留64个通道，但目前仅可用32个 */
#define SMPL_CHAN_NUM (32)		/*!< 可用采样通道数 */
#define SMPL_DATA_NUM (10000)	/*!< 每采样通道的采样点数 */
/*!@}*/

/*! 采样类型 */
enum HncSampleType
{
    SAMPL_NULL_TYPE = 0,/*!< 空类型 */
    SAMPL_AXIS_CMD = 1,	/*!< 轴的指令位置 */
    SAMPL_AXIS_ACT,		/*!< 轴的实际位置 */
    SAMPL_FOLLOW_ERR,	/*!< 轴的跟随误差 */
    SAMPL_CMD_INC,		/*!< 轴的指令速度 */
    SAMPL_ACT_VEL,		/*!< 轴的实际速度 */
    SAMPL_ACT_TRQ,		/*!< 轴的负载电流 */
    SAMPL_CMD_POS,		/*!< 指令电机位置 */
    SAMPL_CMD_PULSE,	/*!< 指令脉冲位置 */
    SAMPL_ACT_POS,		/*!< 实际电机位置 */
    SAMPL_ACT_PULSE,	/*!< 实际脉冲位置 */
    SAMPL_TOL_COMP,		/*!< 补偿值 */
    /*!< 12预留给程序行号N */
    SAMPL_ACT_POS_2 = 13,/*!< 全闭环实际位置 */
    SAMPL_ACT_VEL_2,	/*!< 全闭环实际速度 */
    SAMPL_VIR_AXIS_POS_CMD,	/*!< 虚拟轴指令位置 */
    SAMPL_VIR_AXIS_POS_ACT,	/*!< 虚拟轴实际位置 */
    SAMPL_SYS_VAL = 101,	/*!< 系统变量 */
    SAMPL_CHAN_VAL,		/*!< 通道变量 */
    SAMPL_AXIS_VAL,		/*!< 轴变量 */
    SAMPL_X_REG,		/*!< X寄存器 */
    SAMPL_Y_REG,		/*!< Y寄存器 */
    SAMPL_F_AXIS_REG,	/*!< 轴F寄存器 */
    SAMPL_G_AXIS_REG,	/*!< 轴G寄存器 */
    SAMPL_F_CHAN_REG,	/*!< 通道F寄存器 */
    SAMPL_G_CHAN_REG,	/*!< 通道G寄存器 */
    SAMPL_F_SYS_REG,	/*!< 系统F寄存器 */
    SAMPL_G_SYS_REG,	/*!< 系统G寄存器 */
    SAMPL_R_REG,		/*!< R寄存器 */
    SAMPL_B_REG,		/*!< B寄存器 */
    SAMPL_I_REG,		/*!< I寄存器 */
    SAMPL_Q_REG,		/*!< Q寄存器 */
    SAMPL_K_REG,		/*!< K寄存器 */
    SAMPL_W_REG,		/*!< W寄存器 */
    SAMPL_D_REG,		/*!< D寄存器 */
    SAMPL_T_REG,		/*!< T寄存器 */
    SAMPL_C_REG,		/*!< C寄存器 */
    SAMPL_TOTAL
};

/*!@}*/
#define SAMPL_TYPE_VALID(type)	((type > SAMPL_NULL_TYPE && type <= SAMPL_VIR_AXIS_POS_ACT) || (type >= SAMPL_SYS_VAL && type < SAMPL_TOTAL))

#define T_CTRL_CHANGE_DIRECT 0X1
#define T_CTRL_TOOL_MODE     0X2
#define VAR_WRITE_WAIT       0X1
#define VAR_READ_WAIT        0X2

// PAR_NCU_WORKMEAS_AUTO
#define WOKR_MEAS_JOG_MEAS_FLAG 0X00000001  /*!< 工件测量功能,是否开启通过菜单X+\X-\Y+\Y-\Z+\Z-进行手动测量 */
#define WOKR_MEAS_CONFIRM_FLAG  0X00000002 /*!<  点击菜单X+\X-\Y+\Y-\Z+\Z-进行手动测量是否需要提示确认 */
#define WOKR_MEAS_POS_ADD_TOOLLEN_FLAG  0X00000004 /*!< 测量结果是否叠加刀补 */
#define WOKR_MEAS_ANGLE_ROTATE_FLAG  0X00000008 /*!< 角度测量结果不按就近旋转原则 */
#define WOKR_MEAS_3DROT_ASK_FLAG_OFF  0X00000010 /*!< 旋转轴不为0时,是否关闭3DRot开启提醒 */
#define WOKR_MEAS_WM_POS_TYPE         0X00000020 /*!< 工件测量第一列显示坐标类型:0:机床实际,1:工件实际 */

//PAR_MAC_NEW_FUCTION_TEST
#define  HMI_SET_FS                  0X00000001   /*!< HMI修改FS */
#define  COORD_ADD                   0X00000002  /*!< 坐标系叠加 */
#define  G50_CLEAR_TYPE              0x00000004  /*!< G50取消方式 */
#define  FLASH_G5X_RESET             0X00000008  /*!<复位时工件坐标系的设定*/
//#define  G97_NOT_IN_ADVANCE 0x0004/*!< G97指令不超前解释*/
//#define  G71_FINISHING_DIR_ALARM 0x0008 /*!< 精加工方向判断*/
//#define  STREAM_CYCLE_VAR_CLEAR  0x0010 /*!< 不同的固定循环文件模态变量清零(此参数作废yeqi:2019/3/13)*/
#define  G96_KEEP_SPDL                      0X00000010  /*!< 转进给模式切换其他状态，是否保持主轴转速*/
#define  M99_COUNT                          0x00000020  /*!< M99计数功能*/
#define  STREAM_CYCLE_SBL                   0x00000040  /*!< 固定循环单段*/
#define  ZOUXINJI_SPECIAL                   0x00000080  /*!< 走芯机专用功能（T指令调用子程序）*/

#define  TURN_TOOL_ROLL_ENABLE              0x00000100  /*!< 车刀旋转功能开启*/
#define  C_AXIS_INC                         0x00000200  /*!< C轴H增量编程功能开启*/
#define  G95_FOLLOW_ACT						0x00000400	/*!< g95跟随实际反馈*/
#define  TURN_TOOL_RETURN_TYPE              0X00000800  /*!< T指令回零方式*/

#define  SPDL_SPEED_KEEP_POSITIVE           0X00001000  /*!< 主轴指令转速一直保持正值*/
#define  PWCS_ZERO_WHEN_TOOLOFF_CHANGED	    0X00002000	/*!< 	车床刀补/磨损修改工件零点显示立刻更新 */
#define  MDI_AND_PROG_COORD_SYNC            0X00004000  /*!< MDI和程序之前刀补和坐标系模态是否保持*/
#define  MULTI_SPDL_CUTTER_OFFSET           0x00008000  /*!< 排刀偏置补全*/

#define  TURN_TOOL_CALL_GCODE               0x00010000  /*!< T指令调用子程序*/
#define  THREAD_NEW_TYPE		    	    0x00020000	/*!<螺纹车削方式2*/
#define  MDI_AND_PROG_TOOL_KEEP             0X00040000  /*!<MDI和程序之前T坐标系是否保持*/
#define  UVW_ABS_AND_INC                    0X00080000  /*!< UVW增量编程和实际UVW轴共存*/

//#define  CHAMFER_NC                       0x00100000	/*!< 倒角指令方式*/
#define  G96_G00_S_SPEED_IS_MAX             0X00200000  /*!< G00恒限速主轴最大速度立即到达*/
//#define  CHAMFER_NA                       0x00400000	/*!< 角度指令方式*/


//PAR_MAC_NEW_FUCTION_TEST2
//#define  NOISE_BLK_FILTER             0X00000001  /*!< 噪点过滤 */
#define  M99_TO_PRE_RUN                 0X00000002 /*!< 主程序M99回到PRE_RUN.CYC继续执行 */
#define  DEBUG_ROW                      0X00000004 /*!< 程序运行调试使能,	运行蓝色行到固定循环内，单段时固定循环允许单段执行 */
#define  G91_G52_ADD_ZERO               0X00000008 /*!< G91G52时叠加工件零点 */
////20181130linsenadd
#define  MULTI_SPDL_CTRL_EXT            0X00000010  /*!< M3/4/5控制0号主轴；M13/14/15控制1号主轴；M23/24/25控制2号主轴；M33/34/35控制3号主轴； */
#define  WIN32_OUTPUT_IIP               0X00000020  /*!< WIN模拟版下，输出插补文件； */
#define  INTP_PRODUCE_POINT             0X00000040  /*!< WIN模拟版下，产生一个插补点就停止插补，直到数据被取走； */
#define  M99_NO_STOP                    0X00000080  /*!< M99不产生准停段； */

#define  USR_VAR_TYPE_SYN               0X00000100  /*!< 用户自定义变量类型同步; */
#define  G108_NO_SPLIT                  0X00000200  /*!< G108不做行拆分,需求：如果主轴在位置模式，则执行到M3时不自动切换到速度模式，而进行报警处理*/
#define  SCAN_G00_FEEDRATE              0X00000400  /*!< 设1则任意行返回G00速度执行，否则返回G01+040030速度*/
#define  GRINDING_PROCESS               0X00000800 /*!< 磨削工艺开关*/

#define  RANDOM_ROW_TYPE                0X00001000  /*!< 打开时，任意行到指定行；关闭时，任意行到指定行上一行 */
#define  VERIFY_NO_CHECK_LIMIT          0X00002000  /*!< 校验不检查软限位 */
#define  G54PX_MULTI_CHAN               0X00004000  /*!< 打开时，G54.x在多个通道间平均分配 */
#define  TOOL_ALARM_M30                 0X00008000  /*!< 打开时，M30时报警刀具寿命到 */

#define  GETD_AXIS_CHANGE_COORD         0X00010000  /*!< 打开时，抓取轴同时将坐标系也抓取过来 */
#define  SPL_VECT_CHANGE                0X00020000  /*!< 切向强置 */
#define  CHECK_AX_LIMIT_DECODER         0X00040000  /*!< 打开时，在解释器层检查限位 */
#define  JOG_SUBPROG_FORM_JOG_CYC       0X00080000  /*!< 打开时，手动子程序从JOG.CYC中调用 */

#define  M3_S_MISMATCH                  0X00100000  /*!< 专项需求：多主轴情况下，M3指令和S指令不匹配时报警。举例：m3s2=1000 */
//#define  SAVE_BREAKPOINT_AT_ESTOP     0X00200000  /*!<作废 急停时保存断点文件 */
//#define  SAVE_BREAKPOINT_AT_POWEROFF  0X00400000  /*!<作废 断电时保存断点文件 */
#define  ALL_FILE_LOAD                  0X00800000  /*!<装载主程序G代码及所有相关G代码，i代码到内存中 */

#define  SPEED_CONTROL_ADAPTIVE         0X01000000  /*!< 速度控制自适应 */
#define  CTOS_CLR_POS                   0X02000000  /*!< CTOS清除坐标 */
//#define  ARC_RATATE_VEL_MODE	        0X04000000  /*!<旋转轴的插补速度控制模式：默认进给速度F为线性轴速度，旋转轴跟随运动，有该标记是时进给速度F为合成速度模式*/
#define  AUTO_RUN_DCM_ON                0X08000000     /*!< 运行程序时自动防碰撞开启 */

//#define  G68_SPACE_ROTATE             0X10000000  /*!< G68空间旋转 */
#define  SYN_MCODE_NO_ACK_NO_ALARM      0X20000000  /*!< 同步M代码无应答时持续等待应答; */
//#define  EDIT_MEMORY_100M             0X40000000  /*!< 编辑区内存100M*/
#define  GROUP_01_DEFALUT_MODE_IS_00    0X80000000  /*!< 第一组G代码默认模态*/

//PAR_GCODE_FANUC
#define FANUC_A 0X0002  /*!< FANUC A类指令 */

// EXTOOL_LTOOL_PARM
#define TOOL_ROLL_DIR   0X00000001 /*!< 车刀,旋转方向:0x01反转,0x00正转 */
#define IS_TOOL_LOCKED  0X00000002 /*! 车刀刀具的偏置,R,T,旋转方向锁定 */
#define TURRET_LOCATION 0X00000004 /*! 是否为背主轴刀具，此位为1代表是，此位为0代表否*/

//PAR_NCU_CLOSE_HEAT_COMP
#define RUN_CONTROL     1
#define M_CODE_CONTROL  2

//PAR_NCU_HMISHOW
#define TOOL_TYPE_SHOW                  0x0004 /*!< 刀具属性 */
#define TABLE_NO_DEFAULT_MOVE           0x0008 /*!< 关闭表格输入后自动移动焦点 */
#define TURN_SIMU_MIRRORED              0x0020 /*!< 斜床身零件图形显示 */
#define ACT_FEED_SHOW_CMD               0x0040 /*!< F实际显示F指令的值 */
#define TOOLLIFE_HMI_TYPE               0x0080 /*!< 兼容旧版本的刀具寿命管理界面 0：综合刀具寿命管理；1：旧版本刀具寿命管理*/
#define HMI_SHOW_PNG_FIRST              0x0100 /*!< 2.40占位 开机图片默认显示png图片 */
#define HMI_BACKGROUND_LOADING_OFF      0x0200 /*!< 开机加载完成所有界面后启动系统 */
#define HMI_GMODE_CUTINFO_SHOW          0x0800 /*!< 2.40.07占位 加工资讯显示 0：模态信息 1：加工信息 */
#define HMI_SSA_QR_SHOW                 0x1000 /*!< 2.40占位 加工统计、状态统计、报警信息二维码是否显示 */
#define MILL_SIMU_THUMBNAIL_MIRRORED    0x2000 /*!< 2.40占位 仿新代图形开关：是否支持铣床图形框选放大、镜像显示 */
#define TRUSS_VALID_FLAG                0x4000 /*!< 开启桁架功能*/
#define G50_AXIS_SAVE_FLAG              0x8000 /*!< 开启G50坐标值保存功能功能*/
#define IS_5AXIS_FOOL_PROOFING_ON       0x10000 /*!< 是否开启五轴坐标系防呆功能 */
#define HMI_OPEN_CONFIRM_MENU_FUNC 		0x20000 /*!< 是否开启确认取消菜单功能 0：关闭 1：开启 */

// PAR_NCU_FS_SHOWTYPE 2.40占位
#define PAR_NCU_FEED_SHOW_CMD 0x01 /*!< F实际,显示为指令值 */
#define PAR_NCU_FEED_SHOW_ACT 0x02 /*!< F指令,显示为实际值 */
#define PAR_NCU_SDPL_SHOW_CMD 0x04 /*!< S实际,显示为指令值 */
#define PAR_NCU_SPDL_SHOW_ACT 0x08 /*!< S指令,显示为实际值 */

// PAR_NCU_MAINWG_SHOW_TYPE /*!< 加工界面显示:VIEW_POS(位置视图),VIEW_ACTWCS(绝对坐标), VIEW_ACTRCS(相对坐标)*/
#define PAR_NCU_SHOW_POS    0x01 /*!< VIEW_POS(位置视图) */
#define PAR_NCU_SHOW_ACTWSC 0x02 /*!< VIEW_ACTWCS(绝对坐标) */
#define PAR_NCU_SHOW_ACTRCS 0x04 /*!< VIEW_ACTRCS(相对坐标) */
#define PAR_NCU_SHOW_ACTMCS 0x08 /*!< VIEW_ACTMCS(机床实际) */

// PAR_AX_PARAM
#define PAR_AX_ZERO_SET_STATE 0x01 /*!< 零点是否建立,0为已建立*/

// PAR_NCU_FOREIGN_LANGUAGE_SHOW
#define PAR_NCU_SHOW_ENGLISH    0x01 /*!< 系统支持显示英文 */
#define PAR_NCU_SHOW_RUSSIA     0x02 /*!< 系统支持显示俄文 */

// PAR_NCU_COMP_ENABLE
#define PAR_NCU_COMP_WARM_UP     0x00010000 /*!< 一键暖机功能开关 */

#define RUN_MAX_CNTR (999999999) // 加工最大工件数

/*!	报警错误类型 */
//enum AlarmType
//{
//    ALARM_SY = 0,	/*!< 	系统报警（System） */
//    ALARM_CH,		/*!< 	通道报警（Channel） */
//    ALARM_AX,		/*!< 	轴报警（Axis） */
//    ALARM_SV,		/*!< 	伺服报警（Servo） */
//    ALARM_PC,		/*!< 	PLC报警（PLC） */
//    ALARM_DV,		/*!< 	设备报警（Dev） */
//    ALARM_PS,		/*!< 	语法报警（Program Syntax） */
//    ALARM_UP,		/*!< 	用户PLC报警（User PLC） */
//    ALARM_HM,		/*!< 	HMI报警（HMI） */
//    ALARM_ETHCAT_SV,/*!< 	EtherCAT饲服报警 */
//    ALARM_TYPE_ALL
//};

///*! 	报警级别 */
//enum AlarmLevel
//{
//    ALARM_ERR = 0,	/*!< 	错误（Error） */
//    ALARM_MSG,		/*!< 	提示（Message）	 */
//    ALARM_LEVEL_ALL
//};

/*! 语言类型*/
enum LangType
{
    LANG_CH = 0,		/*!<	中文*/
    LANG_EN,			/*!<	英文*/
    LANG_RU,			/*!<	俄文*/

    LANG_COUNT
};

/*! 系统自定义宏变量(55000-59999) */
enum NcMAcro
{
    NC_MACRO_ST = 55000,

    /*!< 雷尼绍测量使用(预留200个) */
    NC_MACRO_LNS_MACRO_ST = 55600,

    NC_MACRO_LNS_TOOL_PORBE_X_RADIUS = NC_MACRO_LNS_MACRO_ST, /*!< X方向测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_Y_RADIUS,/*!< Y方向测球半径 */
    NC_MACRO_LNS_TOOL_ARROW_X_OFF,/*!< X轴测针偏置 */
    NC_MACRO_LNS_TOOL_ARROW_Y_OFF,/*!< Y轴测针偏置 */
    NC_MACRO_LNS_TOOL_DATA_ST,/*!< 测头校正数据存放区4 */
    NC_MACRO_LNS_TOOL_DATA_ED = NC_MACRO_LNS_TOOL_DATA_ST + 5,/*!< 测头校正数据存放区9 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_30,/*!< 30度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_60,/*!< 60度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_120,/*!< 120度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_150,/*!< 150度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_210,/*!< 210度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_240,/*!< 240度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_300,/*!< 300度测球半径 */
    NC_MACRO_LNS_TOOL_PORBE_RADIUS_ANGLE_330,/*!< 330度测球半径 */
    NC_MACRO_LNS_TOOL_CAL_Z_RADIUS,/*!< Z轴标定半径 */
    NC_MACRO_LNS_TOOL_CAL_ARROW_RADIUS,/*!< 标称测针半径 */

    NC_MACRO_LNS_DATA = NC_MACRO_LNS_MACRO_ST + 20, /*!< 其它量测设备校正数据存放区(60个) */
    NC_MACRO_LNS_POS_WAIT = NC_MACRO_LNS_DATA + 60,  /*!< 位置读取等待时间 */
    NC_MACRO_LNS_RESEVER_0,         /*!< 保留 */
    NC_MACRO_LNS_RESEVER_1,         /*!< 保留 */
    NC_MACRO_LNS_NO,                /*!< 授权号码 */
    NC_MACRO_LNS_P_ID,              /*!< 操作讯息P地址定义 */
    NC_MACRO_LNS_RESEVER_2,         /*!< 保留 */
    NC_MACRO_LNS_RESEVER_3,         /*!< 保留 */
    NC_MACRO_LNS_TOOL_DELAY_TIME,   /*!< 测头定位延迟时间 */
    NC_MACRO_LNS_TOOL_NO,           /*!< 测头刀具号 */
    NC_MACRO_LNS_RE_VAL,            /*!< 优化预设回退系数 */
    NC_MACRO_LNS_ST_POS,            /*!< 启始位置 */
    NC_MACRO_LNS_WSC_POS,           /*!< 回退工件坐标位置 */
    NC_MACRO_LNS_TOOL_EDGE_STATE,   /*!< 刀沿状态捕获 */
    NC_MACRO_LNS_SAFE_MAX_SPEED,    /*!< 测針安全最高速率 */
    NC_MACRO_LNS_MEAS_SLOW_SPEED,   /*!< 低速量测速度设定 */
    NC_MACRO_LNS_MEAS_FAST_SPEEP,   /*!< 高速量测速度设定 */
    NC_MACRO_LNS_MEAS_TIMES,        /*!< 量测次数设定 */
    NC_MACRO_LNS_ROTATE_TYPE,       /*!< 机器旋转轴配置设定 */
    NC_MACRO_LNS_5_AXIS_TOOL_TYPE,  /*!< 五轴型式设定 */
    NC_MACRO_LNS_RESEVER_4,         /*!< 保留 */
    NC_MACRO_LNS_RESEVER_5,         /*!< 保留 */
    NC_MACRO_LNS_A_AIXS,            /*!< A轴设定 */
    NC_MACRO_LNS_B_AXIS,            /*!< B轴设定 */
    NC_MACRO_LNS_C_AXIS,            /*!< C轴设定 */
    NC_MACRO_LNS_A_AIXS_OFF_DIR,    /*!< A轴WCS补偿方向设定 */
    NC_MACRO_LNS_B_AIXS_OFF_DIR,    /*!< B轴WCS补偿方向设定 */
    NC_MACRO_LNS_C_AIXS_OFF_DIR,    /*!< C轴WCS补偿方向设定 */
    NC_MACRO_LNS_SYS_VER,           /*!< 控制器系統版本 */
    NC_MACRO_LNS_5_AXIS_TYPE,       /*!< 5轴结构 */
    NC_MACRO_LNS_SPDL_RELEASE_M,    /*!< 主轴松弛M代码号 */
    NC_MACRO_LNS_G31_X_ADDR,        /*!< G31触发X接线位址 */
    NC_MACRO_LNS_G31_X_BIT,         /*!< 接线位编码 */
    NC_MACRO_LNS_G31_L,             /*!< G31 L */
    NC_MACRO_LNS_SPEED_CONTROL_M,   /*!< 速率可被控制M代码号 */
    NC_MACRO_LNS_FUUL_SPEED_M,      /*!< 速率100%M代码号 */
    NC_MACRO_LNS_TOOL_START_M,      /*!< 测头开启M代码号 */
    NC_MACRO_LNS_TOOL_FIN_M,        /*!< 测头关闭M代码号 */
    NC_MACRO_LNS_MEAS_SPEED,        /*!< 量测速度 */
    NC_MACRO_LNS_HIGH_SPEED_MEAS,   /*!< 高速量测 */
    NC_MACRO_LNS_RETURN_VAL,        /*!< 量测回退系數 */
    NC_MACRO_LNS_GO_PROBE_MODE,     /*!< Go Probe 的校正模式 */
    NC_MACRO_LNS_MEAS_PROG_TYPE,    /*!< O98xx 程序号码记录 */

    NC_MACRO_LNS_MACRO_ED = NC_MACRO_LNS_MACRO_ST + 200,/*!< 雷尼绍测量结束(预留200个) */

    /*!< 55800-56000 车床桁架使用(预留200个) */
    NC_MACRO_TG_MACRO_ST = 55800,

    NC_MACRO_TG_TRAY_PICK_CUR_ROW = 55812,        /*!< 桁架取料盘当前行  */
    NC_MACRO_TG_TRAY_PICK_CUR_COL,                /*!< 桁架取料盘当前列  */
    NC_MACRO_TG_TRAY_PICK_CUR_TIER,               /*!< 桁架取料盘当前层  */

    NC_MACRO_TG_TRAY_CUR_GROUP = 55817,           /*!< 桁架料盘当前组  */

    NC_MACRO_TG_TRAY_THROW_CUR_ROW = 55822,       /*!< 桁架放料盘当前行  */
    NC_MACRO_TG_TRAY_THROW_CUR_COL,               /*!< 桁架放料盘当前列  */
    NC_MACRO_TG_TRAY_THROW_CUR_TIER,              /*!< 桁架放料盘当前层  */

    NC_MACRO_TG_RUN_FROM_BEGINING = 55826,        /*!< 桁架从头开始运行标记  */

    NC_MACRO_TG_GRIPPER_SWITCH = 55828,           /*!< 桁架料爪空抓后选择 0：停止加工 1：继续取料  */

    NC_MACRO_TG_TRAY_PICK_ROW_NUM = 55842,        /*!< 桁架取料盘行数  */
    NC_MACRO_TG_TRAY_PICK_COL_NUM,                /*!< 桁架取料盘列数  */
    NC_MACRO_TG_TRAY_PICK_TIER_NUM,               /*!< 桁架取料盘层数  */

    NC_MACRO_TG_TRAY_TYPE = 55846,                /*!< 桁架料盘类型 0：取放合一； >0：取放分离  */

    NC_MACRO_TG_TRAY_THROW_ROW_NUM = 55872,       /*!< 桁架放料盘行数  */
    NC_MACRO_TG_TRAY_THROW_COL_NUM,               /*!< 桁架放料盘列数  */
    NC_MACRO_TG_TRAY_THROW_TIER_NUM,              /*!< 桁架放料盘层数  */

    NC_MACRO_TG_SIMU_X_POSITIVE_STAND = 55890,    /*!< 桁架仿真x轴正基准坐标  */
    NC_MACRO_TG_SIMU_X_MINUS_STAND,               /*!< 桁架仿真x轴负基准坐标  */
    NC_MACRO_TG_SIMU_Z_POSITIVE_STAND,            /*!< 桁架仿真z轴正基准坐标  */
    NC_MACRO_TG_SIMU_Z_MINUS_STAND,               /*!< 桁架仿真z轴负基准坐标  */

    NC_MACRO_TG_TECHPOINT_SHOW_AXIS = 55897,      /*!< 示教点显示轴  */
    NC_MACRO_TG_TRAY_DIR = 55898,                 /*!< 料盘方向  */

    NC_MACRO_TG_MACRO_ED = NC_MACRO_TG_MACRO_ST +200,

    /*!< 56000-56050五轴标定使用(预留50个) */
    NC_MACRO_5AXIS_MACRO_ST = 56000,
    NC_MACRO_5AXIS_MEAS_HAS_INIT = NC_MACRO_5AXIS_MACRO_ST,/*!< 56000 有/无初值标定 */
    NC_MACRO_5AXIS_MEAS_TYPE,/*!< 标定类型 */
    NC_MACRO_5AXIS_ROT_AXIS_TYPE,/*!<  旋转轴显示顺序 */
    NC_MACRO_5AXIS_ROT_AXIS_NAME,/*!<  旋转轴名 */
    NC_MACRO_5AXIS_SHOW_MODE,/*!<  显示模式 */
    NC_MACRO_5AXIS_STANDER_R,/*!<  标准球半径 */
    NC_MACRO_5AXIS_SAFTY_HEIGHT,/*!<  安全高度 */
    NC_MACRO_5AXIS_BACK_HEIGHT,/*!<  退刀高度 */
    NC_MACRO_5AXIS_FEED, /*!<  进给率"	*/
    NC_MACRO_5AXIS_STANDARD_ANGLE, /*!<  参考角度 */
    NC_MACRO_5AXIS_A_AXIS_ST_ANGLE, /*!<  A轴起始角 */
    NC_MACRO_5AXIS_A_AXIS_ED_ANGLE, /*!<  A轴终止角 */
    NC_MACRO_5AXIS_A_AXIS_OFF_ANGLE, /*!<  A轴仰角"	*/
    NC_MACRO_5AXIS_A_AXIS_MEAS_NUM, /*!<  A轴示教点个数"*/
    NC_MACRO_5AXIS_B_AXIS_ST_ANGLE, /*!<  B轴起始角 */
    NC_MACRO_5AXIS_B_AXIS_ED_ANGLE, /*!<  B轴终止角 */
    NC_MACRO_5AXIS_B_AXIS_OFF_ANGLE, /*!<  B轴仰角 */
    NC_MACRO_5AXIS_B_AXIS_MEAS_NUM, /*!<  B轴示教点个数 */
    NC_MACRO_5AXIS_C_AXIS_ST_ANGLE, /*!<  C轴起始角 */
    NC_MACRO_5AXIS_C_AXIS_ED_ANGLE, /*!<  C轴终止角 */
    NC_MACRO_5AXIS_C_AXIS_OFF_ANGLE, /*!<  C轴仰角 */
    NC_MACRO_5AXIS_C_AXIS_MEAS_NUM, /*!<  C轴示教点个数 */
    NC_MACRO_5AXIS_MEAS_TIMES, /*!<  探测次数 */
    NC_MACRO_5AXIS_MEAS_INIT, /*!<  是否完成初值设置 */

    NC_MACRO_5AXIS_MACRO_ED = NC_MACRO_5AXIS_MACRO_ST + 50,

    /*!< 56050-56100测量数据记录区(预留50个) 此区域数据五轴标定使用只做运行中间数据使用,运行完毕则不再需要,其他模块中间数据可复用 */
    NC_MACRO_5AXIS_MEAS_DATA_ST = 56050,
    NC_MACRO_5AXIS_MEAS_DATA_ED = NC_MACRO_5AXIS_MEAS_DATA_ST + 50,

    /*!<  56100-56149 标准球测头标定使用(预留50个) */
    NC_MACRO_WORK_MEAS_TOOL_BALL_ST = 56100,
    NC_MACRO_WORK_MEAS_TOOL_BALL_RADIUS = NC_MACRO_WORK_MEAS_TOOL_BALL_ST,  /*!< 标准球半径 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_SAFTY_HEIGHT,  /*!< 安全高度 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_MOVE_MODE,  /*!< 运动形式 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_MEAS_NUM,  /*!< 赤道标定数量 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_ANGLE,  /*!< 参考角度 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_LEN_ENABLE,  /*!< 长度标度使能 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_Z_POS,  /*!< 球心Z坐标 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DIVISION_VAL,  /*!< 标定维度分度*/

    NC_MACRO_WORK_MEAS_TOOL_BALL_RADIUS_X_P = 56110,  /*!< 真实半径X+ */
    NC_MACRO_WORK_MEAS_TOOL_BALL_RADIUS_X_N,  /*!< 真实半径X- */
    NC_MACRO_WORK_MEAS_TOOL_BALL_RADIUS_Y_P,  /*!< 真实半径Y+ */
    NC_MACRO_WORK_MEAS_TOOL_BALL_RADIUS_Y_N,  /*!< 真实半径Y- */
    NC_MACRO_WORK_MEAS_TOOL_BALL_X_OFF,  /*!< 偏心X */
    NC_MACRO_WORK_MEAS_TOOL_BALL_Y_OFF,  /*!< 偏心Y */
    NC_MACRO_WORK_MEAS_TOOL_BALL_X_TRIGGER,  /*!< 触发半径X */
    NC_MACRO_WORK_MEAS_TOOL_BALL_Y_TRIGGER,  /*!< 触发半径Y */
    NC_MACRO_WORK_MEAS_TOOL_BALL_TOOL_LEN,  /*!< 实际测量时的刀长补 */
    NC_MACRO_WORK_MEAS_TOOL_BALL_G108_TRIGGER_M19,  /*!< G108模式的触发半径(M19基准) */
    NC_MACRO_WORK_MEAS_TOOL_BALL_G108_TRIGGER_G108,  /*!< G108模式的触发半径(G108 C0基准) */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE0,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE1,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE2,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE3,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE4,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE5,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE6,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE7,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE8,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE9,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE10,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_DATA_SAVE11,  /*!< 矢量标定数据保存区域（M19为基准，赤道方向） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL0,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL1,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL2,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL3,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL4,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL5,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL6,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL7,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL8,  /*!< 3D标定（M19为基准，G108模式，0经度线） */
    NC_MACRO_WORK_MEAS_TOOL_BALL_3D_CAL9,  /*!< 3D标定（M19为基准，G108模式，0经度线） */

    NC_MACRO_WORK_MEAS_TOOL_BALL_ED = NC_MACRO_WORK_MEAS_TOOL_BALL_ST + 50,

    /*!<  56150-56199通用性工件测量参数(预留50个) */
    NC_MACRO_COM_WORK_MEAS_ST = 56150,
    NC_MACRO_COM_WORK_MEAS_JUMP_REG = NC_MACRO_COM_WORK_MEAS_ST, /*!< PLC跳段信号，X信号 */
    NC_MACRO_COM_WORK_MEAS_JUMP_BIT,  /*!< 56150定义的X信号点位，对应的2的N次方 */
    NC_MACRO_COM_WORK_MEAS_USERIN_BIT,  /*!< PLC中USERIN信号，对应2的N次方 */
    NC_MACRO_COM_WORK_MEAS_G31_SIGNAL,  /*!< PLC中工件测量G31信号 */
    NC_MACRO_COM_WORK_MEAS_FEED_CHANGABLE_M_CODE,  /*!< 进给速度可调M代码 */
    NC_MACRO_COM_WORK_MEAS_FEED_LOCKED_M_CODE,  /*!< 进给速度锁M代码 */
    NC_MACRO_COM_WORK_MEAS_PROBE_ON_M_CODE,  /*!< 测头开M代码 */
    NC_MACRO_COM_WORK_MEAS_PROBE_OFF_M_CODE,  /*!< 测头关M代码 */
    NC_MACRO_COM_WORK_MEAS_S_ORI_M_CODE,  /*!< 主轴定向M代码 */
    NC_MACRO_COM_WORK_MEAS_S_ORI_OFF_M_CODE,  /*!< 取消主轴定向M代码 */
    NC_MACRO_COM_WORK_MEAS_SND_SPEED,  /*!< 二次测量速度 */
    NC_MACRO_COM_WORK_MEAS_FRI_SPEED,  /*!< 一次测量速度 */
    NC_MACRO_COM_WORK_MEAS_RAPID_SPEED,  /*!< 快移速度 */
    NC_MACRO_COM_WORK_MEAS_BACK_LEN,  /*!< 回退距离 */
    NC_MACRO_COM_WORK_MEAS_OVER_LEN,  /*!< 越程距离 */
    NC_MACRO_COM_WORK_MEAS_MEAS_TYPE,  /*!< 测量/标定模式，0=M19，1=G108 */
    NC_MACRO_COM_WORK_MEAS_WAIT_TIME,  /*!< 读取位置等待时间，单位s */
    NC_MACRO_COM_WORK_MEAS_ROT_AXIS,  /*!< 机床旋转轴配置，12=AB，13=AC，23=BC，0=三轴 */
    NC_MACRO_COM_WORK_MEAS_5AIXT_TYPE,  /*!< 五轴结构类型，22=H+H，21=H+T, 11=T+T，0=三轴 */
    NC_MACRO_COM_WORK_MEAS_S_TO_C,  /*!< 主轴切换为位置模式，1=A,2=B,3=C，默认C轴 */
    NC_MACRO_COM_WORK_MEAS_PROBE_R,  /*!< 测头物理半径 */
    NC_MACRO_COM_WORK_TOOL_NO,      /*!< 测头刀具号 */


    NC_MACRO_COM_WORK_MEAS_WCS_X = 56180,  /*!< 输出，特征的X位置WCS */
    NC_MACRO_COM_WORK_MEAS_WCS_Y,  /*!< 输出，特征的Y位置WCS */
    NC_MACRO_COM_WORK_MEAS_WCS_Z,  /*!< 输出，特征的Z位置WCS */
    NC_MACRO_COM_WORK_MEAS_SIZE,  /*!< 输出，尺寸 */
    NC_MACRO_COM_WORK_MEAS_ANGLE,  /*!< 输出，角度 */
    NC_MACRO_COM_WORK_MEAS_X_ERR,  /*!< 输出，X误差 */
    NC_MACRO_COM_WORK_MEAS_Y_ERR,  /*!< 输出，Y误差 */
    NC_MACRO_COM_WORK_MEAS_Z_ERR,  /*!< 输出，Z误差/第二特征误差 */
    NC_MACRO_COM_WORK_MEAS_SIZE_ERR,  /*!< 输出，尺寸误差 */
    NC_MACRO_COM_WORK_MEAS_TYPE,  /*!< 测量类型 */

    NC_MACRO_COM_WORK_MEAS_4POINT_DIR = 56190,

    NC_MACRO_COM_WORK_MEAS_ED = NC_MACRO_COM_WORK_MEAS_ST + 50,

    /*!<  56200-56249通用性工件测量参数拓展区 */
    NC_MACRO_COM_WORK_MEAS_EXT_ST = 56200,
    NC_MACRO_COM_WORK_MEAS_EXT_ED = NC_MACRO_COM_WORK_MEAS_EXT_ST + 50,

    /*!<   56250-56299刀具测量使用 */
    NC_MACRO_TOOL_MEAS_ST = 56250,
    NC_MACRO_TOOL_MEAS_ST_POS = NC_MACRO_TOOL_MEAS_ST,  /*!< 刀具测量起始位 */
    NC_MACRO_TOOL_MEAS_TOOL_WEAR,  /*!< 刀具磨损使能，0=补偿到刀具几何，1=补偿到磨损 */
    NC_MACRO_TOOL_MEAS_MEAS_TYPE,  /*!< 测量选择1 = 刀长,2 = 半径,3 = 刀长+半径,4 = 标定 */
    NC_MACRO_TOOL_MEAS_TOOL_DIA,  /*!< 对刀仪直径、接触式为直径，非接触式为激光宽度 */
    NC_MACRO_TOOL_MEAS_OVER_SHOOT_R,  /*!< 刀具半径过冲量 */
    NC_MACRO_TOOL_MEAS_TOOL_NO,  /*!< 测量的刀号 */
    NC_MACRO_TOOL_MEAS_MID_POS,  /*!< 过渡点距离，即高速移动的定位速度目标点，刀尖距离对刀仪的高度 */
    NC_MACRO_TOOL_MEAS_RAPID_SPEED,  /*!< 定位速度 */
    NC_MACRO_TOOL_MEAS_FIR_SPEED,  /*!< 一次触碰速度 */
    NC_MACRO_TOOL_MEAS_SND_SPEED,  /*!< 二次测量速度 */
    NC_MACRO_TOOL_MEAS_SAFTY_H,  /*!< 安全平面高度，默认为G53 Z0 */
    NC_MACRO_TOOL_MEAS_R_MEAS_DIR,  /*!< 半径测量的测量方向选择 */
    NC_MACRO_TOOL_MEAS_WEAR_MAX,  /*!< 磨损允差上限，超过则报警刀具破损 */
    NC_MACRO_TOOL_MEAS_TOOL_POS_X,  /*!< 对刀仪位置X */
    NC_MACRO_TOOL_MEAS_TOOL_POS_Y,  /*!< 对刀仪位置Y */
    NC_MACRO_TOOL_MEAS_TOOL_POS_Z,  /*!< 对刀仪位置Z */
    NC_MACRO_TOOL_MEAS_SIGNAL_BIT,  /*!< 信号点位，1.1,1.2,1.3…… */
    NC_MACRO_TOOL_MEAS_G31_SIGNAL,  /*!< G31跳段信号 */
    NC_MACRO_TOOL_MEAS_USERIN_ID,  /*!< PLC中USERIN序号 */
    NC_MACRO_TOOL_MEAS_OVER_SHOOT_MAX,  /*!< 过冲量，刀具达到最大过冲量时，应不损坏对刀仪 */
    NC_MACRO_TOOL_MEAS_S_SPEED,  /*!< 主轴转速，默认为0，不旋转；设定时为主轴转速，反转 */
    NC_MACRO_TOOL_MEAS_BLOW_ON_M_CODE,  /*!< 吹气开M代码 */
    NC_MACRO_TOOL_MEAS_BLOW_OFF_M_CODE,  /*!< 吹气关闭M代码 */
    NC_MACRO_TOOL_MEAS_TSG_TYPE,  /*!< 对刀仪类型，0 = 接触式，1 = 非接触式 */
    NC_MACRO_TOOL_MEAS_CAL_STATE,  /*!< 标定状态，0=未标定，1=已标定。固定循环赋值 */
    NC_MACRO_TOOL_MEAS_PROBE_REG,  /*!< PLC测头寄存器0 = X，1 = R  */
    NC_MACRO_TOOL_MEAS_TOOL_ANGLE_A,  /*!< 对刀仪摆角，A */
    NC_MACRO_TOOL_MEAS_TOOL_ANGLE_B,  /*!< 对刀仪摆角，B */
    NC_MACRO_TOOL_MEAS_TOOL_ANGLE_C,  /*!< 对刀仪摆角，C */
    NC_MACRO_TOOL_MEAS_MAC_TYPE,  /*!< 机床结构类型，22 = H+H，11 = T+T，21 = H+T，0 = 三轴,此功能不为零将激活五轴刀具测 */
    NC_MACRO_TOOL_MEAS_ROT_AXIS,  /*!< 机床旋转轴配置，13 = AC,12 = AB,23 = BC，0 = 三轴,此功能不为零将激活五轴刀具测量 */
    NC_MACRO_TOOL_MEAS_TOOL_TYPE, /*!< 刀具类型 */
    NC_MACRO_TOOL_MEAS_UPDATE_TYPE, /*!< 测量结果更新方式 0：自动更新；1：手动更新 */
    NC_MACRO_TOOL_MEAS_TSGSTATE_TYPE, /*!< 对刀仪运行状态 */
    NC_MACRO_TOOL_MEAS_TSGOPEN_MCODE, /*!< 对刀仪开启M代码 */
    NC_MACRO_TOOL_MEAS_TSGCLOSE_MCODE, /*!< 对刀仪关闭M代码 */
    NC_MACRO_TOOL_MEAS_MCODE_USE_TSGPROG_MODEL, /*!< M代码调用对刀仪程序模式 */
    NC_MACRO_TOOL_MEAS_PRESET_TOOL_COMPENSATION_LONGEST_TOOL, /*!< 预设刀补——最长刀 */
    NC_MACRO_TOOL_MEAS_PRESET_TOOL_COMPENSATION_SHORTEST_TOOL, /*!< 预设刀补——最短刀 */
    NC_MACRO_TOOL_MEAS_LONGEST_SHORTEST_TOOL_ENABLE, /*!< 最长最短刀使能 */
    NC_MACRO_TOOL_MEAS_PRESET_TOOL_COMPENSATION_MAX_RADIUS, /*!< 预设刀补——最大半径 */
    NC_MACRO_TOOL_MEAS_PRESET_TOOL_COMPENSATION_MIN_RADIUS, /*!< 预设刀补——最小半径 */

    NC_MACRO_TOOL_MEAS_ED = NC_MACRO_TOOL_MEAS_ST + 50,

    /*!<   56300-56304 长度标定使用 */
    NC_MACRO_TOOL_LEN_MEAS_ST = 56300,
    NC_MACRO_TOOL_LEN_Z_POS = NC_MACRO_TOOL_LEN_MEAS_ST,
    NC_MACRO_TOOL_LEN_MEAS_TYPE,
    NC_MACRO_TOOL_LEN_ANGLE,

    NC_MACRO_TOOL_LEN_MEAS_ED = NC_MACRO_TOOL_LEN_MEAS_ST + 5,

    /*!< 59000-59999预留给车床桁架使用 */
    NC_MACRO_TG_ST = 59000,
    NC_MACRO_TG_ED = 59999,
    NC_MACRO_ED = 59999,
};

/*! 	重解释状态 */
enum
{
    REDECODE_NONE,
    //REDECODE_HMI_CHECK,		/*!<界面检查是否要启动重解释流程*/
    REDECODE_START=2,			/*!<界面通知内核重解释流程开始*/
    //REDECODE_WAIT_PROG,     /*!<等待oredecode.nc准备好*/
    //REDECODE_PROG_RDY,      /*!<oredecode.nc准备好,程序管理器置位*/
    REDECODE_CHAN_RESET,     /*!<解释器程序程序准备好，通道做复位操作*/
    REDECODE_WAIT_RUN_SIG /*!<通道复位完成，等待按循环启动开始执行oredecode.nc*/
};


/******** 格力装备动力技术研究院 许晓婷 2025/12/31 新增 ***********/
/**************** var 变量 *********************/
// var 变量
#define VAR_CHAN_BASE 	(0)             // 通道数据的变量接口起始编号
#define VAR_SYS_BASE 	(40000)	        // 系统数据的变量接口起始编号
#define VAR_AXES_BASE 	(60000)	        // 轴数据的变量接口起始编号
#define VAR_TOOL_BASE 	(70000)	        // 刀具数据的变量起始编号
#define VAR_CHAN_NUM 	(3000)          // 0 ~ 2999
#define VAR_SYS_NUM 	(10000)         // 40000 ~ 59999
#define VAR_AXIS_NUM 	(100)           // 60000 ~ 69999
#define VAR_TOOL_NUM 	(30000)         // 70000 ~ 99999

// var 类型
#define VAR_TYPE_AXIS		(0)
#define VAR_TYPE_CHANNEL	(1)
#define VAR_TYPE_SYSTEM		(2)
#define VAR_TYPE_TOTAL		(3)

// type
#define INT_TYPE		(0)				// 整数类型 (int i)
#define DOUBLE_TYPE		(1)				// 浮点数类型 (double f)
#define UINT_TYPE		(2)				// 无符号整数类型 (unsigned int n)
#define CHAR_TYPE		(3)				// 字符串类型 (char s[8])
#define PSTR_TYPE		(4)				// 字符串指针类型 (char* pstr)

typedef enum{
    VAR_AXIS_NAME = 0,				/*!< 轴名 {Get(Bit8[PARAM_STR_LEN])} */
    VAR_AXIS_TYPE,					/*!< 轴类型 {Get(Bit32)} */
    VAR_AXIS_CHAN,					/*!< 获取通道号 {Get(Bit32)} */
    VAR_AXIS_CHAN_INDEX,			/*!< 获取在通道中的轴号 {Get(Bit32)} */
    VAR_AXIS_CHAN_SINDEX,			/*!< 获取在通道中的主轴号 {Get(Bit32)} */
    VAR_AXIS_LEAD,					/*!< 获取引导轴 {Get(Bit32)} */
    VAR_AXIS_ACT_POS = 6,			/*!< 机床实际位置 {Get(Bit64)} */
    VAR_AXIS_ACT_POS2,				/*!< 机床实际位置2 {Get(Bit64)} */
    VAR_AXIS_CMD_POS = 8,			/*!< 机床指令位置 {Get(Bit64)} */
    VAR_AXIS_ACT_POS_WCS,			/*!< 工件实际位置 {Get(Bit64)} */
    VAR_AXIS_CMD_POS_WCS,			/*!< 工件指令位置 {Get(Bit64)} */
    VAR_AXIS_ACT_POS_RCS,			/*!< 相对实际位置 {Get(Bit64)} */
    VAR_AXIS_CMD_POS_RCS,			/*!< 相对指令位置 {Get(Bit64)} */
    VAR_AXIS_ACT_PULSE,				/*!< 实际脉冲位置 {Get(Bit64)} */
    VAR_AXIS_CMD_PULSE,				/*!< 指令脉冲位置 {Get(Bit64)} */
    VAR_AXIS_PROG_POS,				/*!< 编程位置 {Get(Bit64)} */
    VAR_AXIS_ENC_CNTR,				/*!< 电机位置 {Get(Bit64)} */
    VAR_AXIS_CMD_VEL = 17,			/*!< 指令速度 {Get(Bit64)} */
    VAR_AXIS_ACT_VEL = 18,			/*!< 实际速度 {Get(fBit64)} */
    VAR_AXIS_LEFT_TOGO,				/*!< 工件剩余进给 {Get(Bit64)} */
    VAR_AXIS_WCS_ZERO,				/*!< 工件零点 {Get(Bit64)} */
    VAR_AXIS_WHEEl_OFF,				/*!< 手轮中断偏移量 {Get(Bit64)} */
    VAR_AXIS_FOLLOW_ERR,			/*!< 跟踪误差 {Get(Bit64)} */
    VAR_AXIS_SYN_ERR,				/*!< 同步误差	{Get(Bit64)} */
    VAR_AXIS_COMP,					/*!< 轴补偿值 {Get(Bit64)} */
    VAR_AXIS_ZSW_DIST,				/*!< Z脉冲偏移 {Get(Bit64)} */
    VAR_AXIS_REAL_ZERO,				/*!< 相对零点 {Get(Bit64)} */
    VAR_AXIS_MOTOR_REV,				/*!< 电机转速 {Get(fBit64)} */
    VAR_AXIS_DRIVE_CUR,				/*!< 驱动单元电流 {Get(fBit64)} */
    VAR_AXIS_LOAD_CUR,				/*!< 负载电流 {Get(fBit64)} */
    VAR_AXIS_RATED_CUR,				/*!< 额定电流 {Get(fBit64)} */
    VAR_AXIS_IS_HOMEF,				/*!< 回零完成 {Get(Bit32)} */
    VAR_AXIS_WAVE_FREQ,				/*!< 波形频率 {Get(fBit64)} */
    VAR_AXIS_DRIVE_VER,     		/*!< 伺服驱动版本 {Get(Bit8[32])} */
    VAR_AXIS_MOTOR_TYPE,    		/*!< 伺服类型 {Get(Bit8[32])} */
    VAR_AXIS_MOTOR_TYPE_FLAG,		/*!< 伺服类型出错标志 {Get(Bit32)} */
    VAR_AXIS_Z_SPACE1,				/*!< Z脉间距1 {Get(Bit64)} */
    VAR_AXIS_Z_SPACE2,				/*!< Z脉间距2 {Get(Bit64)} */
    VAR_AXIS_ACT_POS_EX,			/*!< 机床实际位置 {Get(fBit64)} */
    VAR_AXIS_ACT_POS2_EX,			/*!< 机床实际位置2 {Get(fBit64)} */
    VAR_AXIS_CMD_POS_EX,			/*!< 机床指令位置 {Get(fBit64)} */
    VAR_AXIS_ACT_POS_WCS_EX,		/*!< 工件实际位置 {Get(fBit64)} */
    VAR_AXIS_CMD_POS_WCS_EX,		/*!< 工件指令位置 {Get(fBit64)} */
    VAR_AXIS_ACT_POS_RCS_EX,		/*!< 相对实际位置 {Get(fBit64)} */
    VAR_AXIS_CMD_POS_RCS_EX,		/*!< 相对指令位置 {Get(fBit64)} */
    VAR_AXIS_PROG_POS_EX,			/*!< 编程位置 {Get(fBit64)} */
    VAR_AXIS_CMD_VEL_EX,			/*!< 指令速度 {Get(fBit64)} */
    VAR_AXIS_ACT_VEL_EX,			/*!< 实际速度 {Get(fBit64)} */
    VAR_AXIS_LEFT_TOGO_EX,			/*!< 工件系剩余进给 {Get(fBit64)} */
    VAR_AXIS_WCS_ZERO_EX,			/*!< 工件零点 {Get(fBit64)} */
    VAR_AXIS_WHEEL_OFF_EX,			/*!< 手轮中断偏移量 {Get(fBit64)} */
    VAR_AXIS_FOLLOW_ERR_EX,			/*!< 跟踪误差 {Get(fBit64)} */
    VAR_AXIS_SYN_ERR_EX,			/*!< 同步误差	{Get(fBit64)} */
    VAR_AXIS_COMP_EX,				/*!< 轴补偿值 {Get(fBit64)} */
    VAR_AXIS_ZSW_DIST_EX,			/*!< Z脉冲偏移 {Get(fBit64)} */
    VAR_AXIS_REAL_ZERO_EX,			/*!< 相对零点 {Get(fBit64)} */
    VAR_AXIS_DEV_TYPE,				/*!< 轴设备类型	{Get(Bit32)} */
    VAR_AXIS_VENDORID,				/*!< 轴厂家号(目前仅ECAT) {Get(Bit32)} */
    VAR_AXIS_PRODUCTCODE,			/*!< 轴产品号(目前仅ECAT) {Get(Bit32)} */
    VAR_AXIS_ACT_POS_2,				/*!< 第2编码器计算实际位置 {Get(Bit64)} */
    VAR_AXIS_ACT_VEL_2,				/*!< 第2编码器计算实际速度 {Get(fBit64)} */
    VAR_AXIS_POWER,					/*!< 实际功率 {Get(fBit64)} */
    VAR_AXIS_MCS_LEFT_TOGO,     	/*!< 机床系剩余进给 {Get(fBit64)} */
    VAR_AXIS_MCS_LEFT_TOGO_EX,  	/*!< 机床系剩余进给 {Get(fBit64)} */
    VAR_AXIS_TOTAL
}VarAxisIndex;

typedef enum{
    VAR_CHAN_IS_EXIST = 0,			/*!< 通道是否存在 {Get(Bit32)} */
    VAR_CHAN_MAC_TYPE,				/*!< 通道的机床类型 {Get(Bit32)} */
    VAR_CHAN_AXES_MASK,				/*!< 轴掩码 {Get(Bit32)} */
    VAR_CHAN_AXES_MASK1,			/*!< 轴掩码1 {Get(Bit32)} */
    VAR_CHAN_NAME,					/*!< 通道名 {Get(Bit8[PARAM_STR_LEN])} */
    VAR_CHAN_CMD_TYPE,				/*!< 读取当前G代码的标志 {Get(Bit32)} */
    VAR_CHAN_CMD_FEEDRATE,			/*!< 指令进给瞬时速度 {Get(fBit64)} */
    VAR_CHAN_ACT_FEEDRATE,			/*!< 实际进给速度 {Get(fBit64)} */
    VAR_CHAN_PROG_FEEDRATE,			/*!< 编程指令速度 {Get(fBit64)} */
    VAR_CHAN_FEED_OVERRIDE,			/*!< 进给修调 {Get(Bit64)} */
    VAR_CHAN_RAPID_OVERRIDE,		/*!< 快移修调 {Get(Bit64)} */
    VAR_CHAN_MCODE,             	/*!< 通道的M指令 {Get(Bit32)} */
    VAR_CHAN_TCODE,					/*!< 通道的T指令 {Get(Bit32)} */
    VAR_CHAN_TOFFS,					/*!< 通道中的刀偏号 {Get(Bit32)} */
    VAR_CHAN_TOOL_USE,				/*!< 当前刀具 {Get(Bit32)} */
    VAR_CHAN_TOOL_RDY,				/*!< 准备好交换的刀具 {Get(Bit32)} */
    VAR_CHAN_MODE,					/*!< 模式(返回值数据定义见下面) {Get(Bit32)} */
    VAR_CHAN_IS_MDI,				/*!< MDI {Get(Bit32)} */
    VAR_CHAN_CYCLE,					/*!< 循环启动 {Get(Bit32), Set(NULL)} */
    VAR_CHAN_HOLD,					/*!< 进给保持 {Get(Bit32), Set(NULL)} */
    VAR_CHAN_IS_PROGSEL,			/*!< 已选程序 {Get(Bit32)} */
    VAR_CHAN_IS_PROGEND,			/*!< 程序运行完成 {Get(Bit32)} */
    VAR_CHAN_IS_THREADING,			/*!< 螺纹加工 {Get(Bit32)} */
    VAR_CHAN_IS_RIGID,				/*!< 刚性攻丝 {Get(Bit32)} */
    VAR_CHAN_IS_REWINDED,			/*!< 重运行复位状态 {Get(Bit32)} */
    VAR_CHAN_IS_ESTOP,				/*!< 急停 {Get(Bit32)} */
    VAR_CHAN_IS_RESETTING,			/*!< 复位 {Get(Bit32)} */
    VAR_CHAN_IS_RUNNING,			/*!< 运行中 {Get(Bit32)} */
    VAR_CHAN_IS_HOMING,				/*!< 回零中 {Get(Bit32)} */
    VAR_CHAN_IS_MOVING,				/*!< 轴移动中 {Get(Bit32)} */
    VAR_CHAN_DIAMETER,				/*!< 直半径编程 {Get(Bit32)} */
    VAR_CHAN_VERIFY,				/*!< 校验 {Get(Bit32), Set(Bit32)} */
    VAR_CHAN_RUN_ROW,				/*!< 运行行 {Get(Bit32)} */
    VAR_CHAN_DCD_ROW,				/*!< 译码行 {Get(Bit32)} */
    VAR_CHAN_SEL_PROG,				/*!< 选择程序的编号 {Get(Bit32)} */
    VAR_CHAN_RUN_PROG,				/*!< 运行程序的编号 {Get(Bit32)} */
    VAR_CHAN_PART_CNTR,				/*!< 加工计数 {Get(Bit32), Set(Bit32)} */
    VAR_CHAN_PART_STATI,			/*!< 工件总数 {Get(Bit32), Set(Bit32)} */
    VAR_CHAN_HMI_RESET,				/*!< HMI复位 {Set(NULL)} */
    VAR_CHAN_CHG_PROG,				/*!< 程序修改标志 {Set(NULL)} */
    VAR_CHAN_PERIOD_TOTAL,			/*!< 过时的 */
    VAR_CHAN_LAX,					/*!< 通道轴对应的逻辑轴号 {Get(Bit32)} */
    VAR_CHAN_AXIS_NAME,				/*!< 编程轴名 {Get(Bit8[PARAM_STR_LEN])} */
    VAR_CHAN_SPDL_NAME,				/*!< 编程主轴名 {Get(Bit8[PARAM_STR_LEN])} */
    VAR_CHAN_MODAL,					/*!< 通道模态 共80组 {Get(Bit32)} */
    VAR_CHAN_SPDL_LAX,				/*!< 通道主轴对应的逻辑轴号，动态 {Get(Bit32)} */
    VAR_CHAN_SPDL_PARA_LAX,			/*!< 通道主轴对应的逻辑轴号，静态 {Get(Bit32)} */
    VAR_CHAN_CMD_SPDL_SPEED,		/*!< 主轴指令速度 {Get(fBit64)} */
    VAR_CHAN_ACT_SPDL_SPEED,		/*!< 主轴实际速度 {Get(fBit64)} */
    VAR_CHAN_SPDL_OVERRIDE,			/*!< 主轴修调 {Get(Bit64)} */
    VAR_CHAN_DO_HOLD,				/*!< 设置进给保持  */
    VAR_CHAN_BP_POS,				/*!< 断点位置 {Get(Bit64)} */
    VAR_CHAN_PROG_FLOW,				/*!< 正在调用手动子程序 {Get(Bit32)} */
    VAR_CHAN_H_OFF,					/*!< 当前使用刀具长度补偿号 {Get(Bit32)} */
    VAR_CHAN_D_OFF,					/*!< 当前使用刀具半径补偿号 {Get(Bit32)} */
    VAR_CHAN_BP_POS_EX,
    VAR_CHAN_IS_SBL,				/*!< 是否单段模式 {Get(Bit32)} */
    VAR_CHAN_PART_NEED,				/*!< 加工需求数 {Get(Bit32), Set(Bit32)} */
    VAR_CHAN_S_CODE,				/*!< 通道中主轴转速 {Get(Bit32), Set(Bit32)} */
    VAR_CHAN_F_G94_SPEED,			/*!< 通道中记录的F分进给模态速度 {Get(Bit64), Set(Bit64)} */
    VAR_CHAN_F_G95_SPEED,			/*!< 通道中记录的F转进给模态速度 {Get(Bit64), Set(Bit64)} */
    VAR_CHAN_EXT_FILE_STATE,		/*!< 界面告诉内核文件写/删状态(仿FANUC的DPRINT指令用) */
    VAR_CHAN_REFRESH_RUN_INFO,		/*!< 刷新程序行信息 */
    VAR_CHAN_MDI_IN_EXIT_ABLE, 		/*!< 是否可以进入/退出MDi状态 */
    VAR_CHAN_G50_ZERO,           	/*!< G50零点偏置坐标 {Get(fBit64)}*/
    VAR_CHAN_CMD_FEEDRATE_MEAN,		/*!< 指令进给平均速度 {Get(fBit64)} */
    VAR_CHAN_HEAT_FORECAST_K,       /*!< 热误差预测斜率K {Get(Bit64), Set(Bit64)} */
    VAR_CHAN_HEAT_FORECAST_B,       /*!< 热误差预测零点偏置B {Get(Bit64), Set(Bit64)} */
    VAR_PROG_DCD_ROW,
    VAR_PROG_DCD_IDX,
    VAR_PROG_REFRESH_DCD_INFO,
    VAR_CHAN_CYCLE_L,           	/*!< 固定循环剩余调用次数 */
    VAR_CHAN_AXIS_ACT_WCS,       	/*!< 工件实际位置 {Get(Bit32)} >*/
    VAR_CHAN_ABS_ACT_POS,       	/*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    VAR_CHAN_ABS_CMD_POS,       	/*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    VAR_CHAN_ABS_ACT_POS_SAFE,      /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    VAR_CHAN_ABS_CMD_POS_SAFE,      /*!< 虚轴开启时的实际位置 {Get(Bit64)} */
    VAR_CHAN_TOTAL
}VarChannelIndex;

typedef enum{
    VAR_SYS_CHAN_NUM = 0,			/*!< 获取系统通道数 {Get(Bit32)} */
    VAR_SYS_MOVE_UNIT,				/*!< 长度分辨率 {Get(Bit64)} */
    VAR_SYS_TURN_UNIT,				/*!< 角度分辨率 {Get(Bit64)} */
    VAR_SYS_METRIC_DISP,			/*!< 公英制 {Get(Bit32)} */
    VAR_SYS_SHOW_TIME,				/*!< 是否显示时间 {Get(Bit32)} */
    VAR_SYS_POP_ALARM,				/*!< 报警自动显示 {Get(Bit32)} */
    VAR_SYS_GRAPH_ERASE,			/*!< 图形自动擦除 {Get(Bit32)} */
    VAR_SYS_MAC_TYPE,				/*!< 机床类型 */
    VAR_SYS_PREC,					/*!< 坐标系精度 {Get(Bit64)} */
    VAR_SYS_F_PREC,					/*!< F精度 {Get(Bit64)} */
    VAR_SYS_S_PREC,					/*!< S精度 {Get(Bit64)} */
    VAR_SYS_NCK_VER,				/*!< NCK版本 {Get(Bit8[32])} */
    VAR_SYS_DRV_VER,				/*!< DRV版本 {Get(Bit8[32])} */
    VAR_SYS_PLC_VER,				/*!< PLC版本 {Get(Bit8[32])} */
    VAR_SYS_CNC_VER,				/*!< CNC版本(2.00不再使用) {Get(Bit32) Set(Bit32)} */
    VAR_SYS_MCP_KEY,				/*!< MCP面板钥匙开关 {Get(Bit32)} */
    VAR_SYS_ACTIVE_CHAN,			/*!< 活动通道 {Get(Bit32) Set(Bit32)} */
    VAR_SYS_REQUEST_CHAN,			/*!< 请求通道 {Get(Bit32)} */
    VAR_SYS_MDI_CHAN,				/*!< (无效)MDI运行通道 {Get(Bit32)} */
    VAR_SYS_REQUEST_CHAN_MASK,		/*!< 请求的通道屏蔽字 {Get(Bit32)} */
    VAR_SYS_CHAN_MASK,				/*!< 通道屏蔽字 {Set(Bit32)} */
    VAR_SYS_PLC_STOP,				/*!< plc停止 {Set(NULL)} */
    VAR_SYS_POWEROFF_ACT,			/*!< 断电应答 {Set(NULL)} */
    VAR_SYS_IS_HOLD_REDECODE,		/*!< 进给保持后是否重新解释 {Get(Bit32)} */
    VAR_SYS_NC_VER,             	/*!< NC版本(API版本) {Get(Bit8[32])} */
    VAR_SYS_SN_NUM,             	/*!< 数控装置SN号 {Get(Bit8[32])} */
    VAR_SYS_MACHINE_TYPE,			/*!< 机床型号 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_MACHINE_INFO,			/*!< 机床信息 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_MACFAC_INFO,			/*!< 机床厂信息 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_USER_INFO,				/*!< 用户信息 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_MACHINE_NUM,			/*!< 机床编号 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_EXFACTORY_DATE,     	/*!< 出厂时间 {Get(Bit8[32])} */
    VAR_SYS_ACCESS_LEVEL,			/*!< 权限等级 {Get(Bit32)} */
    VAR_SYS_MUL_CHAN_RESET,			/*!< 多通道复位流程启动标记 {Set(Bit32)[0/1]} */
    VAR_SYS_RIGHTS_KEY,				/*!< 权限钥匙锁 {Get(Bit32)} */
    VAR_SYS_REG_DAYS_REMANING,		/*!< 注册剩余天数 {Get(Bit32)} */
    VAR_SYS_PLC2_CUR_CYCLE,			/*!< PLC2当前运行周期 {Get(Bit32)} */
    VAR_SYS_PLC2_MIN_CYCLE,			/*!< PLC2最小运行周期 {Get(Bit32)} */
    VAR_SYS_PLC2_MAX_CYCLE,			/*!< PLC2最大运行周期 {Get(Bit32)} */
    VAR_SYS_PLC_ONLINE,				/*!< PLC在线调试 {Get(Bit8[48]) Set(Bit8[48])} */
    VAR_SYS_OS_VER,					/*!< OS(操作系统)版本 {Get(Bit8[64])} */
    VAR_SYS_HNC_VER,            	/*!< HNC版本(系统版本) {Get(Bit8[64]) Set(Bit8[64])} */
    VAR_SYS_FPGA_VER,           	/*!< FPGA版本 {Get(Bit8[64])} */
    VAR_SYS_PARM_VER,           	/*!< 参数文件版本(主版本+副版本) {Get(Bit8[64])} */
    VAR_SYS_PLCFILE_VER,        	/*!< PLC文件版本(主版本+副版本) {Get(Bit8[64])} */
    VAR_SYS_PLCONOFF_VER,       	/*!< PLC开关文件版本 {Get(Bit8[64])} */
    VAR_SYS_PLCALARM_VER,       	/*!< PLC报警文件版本 {Get(Bit8[64])} */
    VAR_SYS_USRP_VER,           	/*!< 用户P参数文件版本 {Get(Bit8[64])} */
    VAR_SYS_CYCLE_VER,          	/*!< 固定循环文件版本(铣MILLING.CYC,车TURNING.CYC) {Get(Bit8[64])} */
    VAR_SYS_USRCYCLE_VER,       	/*!< 用户自定义循环文件版本(USERDEF.CYC) {Get(Bit8[64])} */
    VAR_SYS_TOTAL
}VarSystemIndex;


#endif // __GNC_DATADEF_H__
