#include "parser.h"
#include "global.h"
#include <hmisysapi.h>
#include <alarmapi.h>
#include <string.h> 

static void parser_macro_init();
static GNC_RES parser_handle_recieve(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
static GNC_RES parser_waiting_to_end();
static GNC_RES parser_waiting_for_motion();
static GNC_RES parser_waiting_for_buf();
static GNC_RES parser_handle_mcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
static GNC_RES parser_handle_tcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
static GNC_RES parser_handle_scode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
static GNC_RES parser_handle_gcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
static GNC_RES parser_handle_null(NcItpState *itp_state, int *line_no);
static GNC_RES parser_set_content(NC_MODE nc_mode);
static bool parser_check_res(GNC_RES res);

PARSER_DATA* parser_data;

static void parser_macro_init()
{
    for (int i = 0; i < 30000; ++i) {
        parser_data->cmv[i] = 0;
    }
    parser_data->cmv[18004] = 0.1;  // 设置半径值超差
    parser_data->cmv[17031] = 0.0;  // 设置坐标旋转1轴运动计算方式
    parser_data->cmv[5410] = -90.0; // 设置坐标旋转默认角度
    parser_data->cmv[17028] = 1.0;  // 默认米制单位
    parser_data->cmv[8132] = 1.0;   // 激活比例缩放功能G51
    parser_data->cmv[8077] = 10.0;  // 设置渐开线插补误差最大值
    parser_data->cmv[5114] = 0.5;   // G73钻深孔,D默认值
    parser_data->cmv[5115] = 1.0;   // G83深孔加工循环,D默认值

    // 坐标偏置 todo:待shm更新后直接用，不用再赋值
    for (int a = 0; a < 3; ++a)
    {
        getCrdsValue(CRDS_TYPE_EXT, a, &parser_data->cmv[5201 + a]);
    }
    for (int t = 0; t < 6; ++t)
    {
        for (int a = 0; a < 3; ++a)
        {
            getCrdsValue(CRDS_TYPE_G5X + t, a, &parser_data->cmv[5221 + t * 20 + a]);
        }
    }
}

void parser_new(){
    parser_data = new PARSER_DATA();
    // 设置译码参数
    parser_macro_init();
    parser_data->parser_state = PARSER_IDLE;
    rt_set_parser_state(-1, 0);
}

GNC_RES parser_init(NC_MODE nc_mode)
{   
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    ItpReset();
    parser_data->parser_state = PARSER_READING;
    // 设置宏定义
    parser_macro_init();
    ItpCfgGetMacroVarCB(t_macroVarGetValue);
    ItpCfgSetMacroVarCB(t_macroVarSetValue);
    // 是否跳过注释段
    ItpSetBlockSkipEnable(rt_get_sys_state(SYS_IS_BLOCK_SKIP));
    // 刀偏
    int tool_count = TOOL_COUNT + 1;
    double toollens[tool_count];
    double toolRads[tool_count];
    toollens[0] = 0.0;
    toolRads[0] = 0.0;
    for (int i = 0; i < TOOL_COUNT; ++i)
    {
        double glen1 = 0.0;
        double wlen1 = 0.0;
        int ret = SHM_ToolGetToolPara(i, GTOOL_LEN1, &glen1, 0);
        ret = SHM_ToolGetToolPara(i, WTOOL_LEN1, &wlen1, 0);
        toollens[i + 1] = glen1 - wlen1;

        double grad1 = 0.0;
        double wrad1 = 0.0;
        ret = SHM_ToolGetToolPara(i, GTOOL_RAD1, &grad1, 0);
        ret = SHM_ToolGetToolPara(i, WTOOL_RAD1, &wrad1, 0);
        toolRads[i + 1] = grad1 - wrad1;
    }
    ItpSetToolLens(toollens, tool_count);
    ItpSetToolRads(toolRads, tool_count);
    // 刀偏参数
    ItpParam_temp itpParam;
    itpParam.cnv = par_ncu[PAR_NCU_PARSER_CNV].value.val_int;
    itpParam.cav = par_ncu[PAR_NCU_PARSER_CAV].value.val_int;
    itpParam.naa = par_ncu[PAR_NCU_PARSER_NAA].value.val_int;
    itpParam.sup = par_ncu[PAR_NCU_PARSER_SUP].value.val_int;
    itpParam.suv = par_ncu[PAR_NCU_PARSER_SUV].value.val_int;
    itpParam.ccc = par_ncu[PAR_NCU_PARSER_CCC].value.val_int;
    itpParam.lookAheadSegments = par_ncu[PAR_NCU_PARSER_LOOKAHEADSEGMENTS].value.val_int;
    ItpSetItpParam(itpParam);
    // 设置机床起点
    double ori_pos_x, ori_pos_y, ori_pos_z;
    ori_pos_x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    ori_pos_y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    ori_pos_z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
    ItpSetOriginMacPos(ori_pos_x, ori_pos_y, ori_pos_z, 0);
    // 设置程序段来源
    res = parser_set_content(nc_mode);
    if(res != GNC_OPERATOR_SUCCESS) return res;
    rt_set_parser_state(-1, 0);
    return res;
}

static GNC_RES parser_set_content(NC_MODE nc_mode){
    // 设置程序段来源
    switch (nc_mode)
    {
    case MODE_MEM:
        // 选择NC文件
        getNcFilePath(parser_data->parser_file_path);
        // todo：界面增加校验，增加警告
        if((strlen(parser_data->parser_file_path) == 0)){
            LOG(LOG_LEVEL_WARN, "译码文件路径不存在");
            setAlarm(ALARM_CH, ALARM_WARNING, 1);
            return GNC_OPERATOR_ERROR;
        }else {
            delAlarm(ALARM_CH, ALARM_WARNING, 1);
            delAlarm(ALARM_CH, ALARM_WARNING, 2);
        }
        ItpSetFile(parser_data->parser_file_path);
        LOG(LOG_LEVEL_INFO, "mem程序 %s", parser_data->parser_file_path);
        break;
    case MODE_MDI:
        // 读取MDI内容
        getMdiContent(parser_data->mdiContent);
        if((strlen(parser_data->mdiContent) == 0)){
            LOG(LOG_LEVEL_WARN, "MDI内容为空");
            setAlarm(ALARM_CH, ALARM_WARNING, 2);
            return GNC_OPERATOR_ERROR;
        }else {
            delAlarm(ALARM_CH, ALARM_WARNING, 1);
            delAlarm(ALARM_CH, ALARM_WARNING, 2);
        }
        ItpSetContent(parser_data->mdiContent, 0);
        LOG(LOG_LEVEL_INFO, "mdi程序 %s", parser_data->mdiContent);
        break;
    default:
        break;
    }
    return GNC_OPERATOR_SUCCESS;
}

void parser_run_mode(){
    if (rt_get_chan_state(0, CHAN_IS_PARSER_RESET) == true)
    {
        parser_reset();
        queue_reset(parser_queue);
        rt_set_chan_state(0, CHAN_IS_PARSER_RESET, false);
    }
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    switch(parser_data->parser_state){
        case PARSER_IDLE:
        break;
        case PARSER_READING:
            res = parser_reading();
            CHECK_RES(parser_check_res(res));
            break;
        case PARSER_WAITING_FOR_BUF:
            res = parser_waiting_for_buf();
            CHECK_RES(parser_check_res(res));
            break;
        case PARSER_WAITING_FOR_MOTION:
        case PARSER_WAITING_FOR_IO:
            res = parser_waiting_for_motion();
            CHECK_RES(parser_check_res(res));
            break;
        case PARSER_WAITING_TO_END:
            res = parser_waiting_to_end();
            CHECK_RES(parser_check_res(res));
            break;
        default:
            break;
    }
}

GNC_RES parser_reading(){
    GNC_RES res = GNC_CAL_SUCCESS;
    ItpGetMotionData(&parser_data->itp_datas);
    ItpGetState(&parser_data->itp_state);
    ItpGetLineNumber(&parser_data->itp_line_no);
    if (parser_data->itp_datas.count > PARSER_BUF_SIZE) return GNC_PARSER_OVER;
    int error_no = ItpError();
    rt_set_parser_state(error_no, parser_data->itp_line_no);
    if (error_no >= 0)
    {
        LOG(LOG_LEVEL_DEBUG, "error line: %d", parser_data->itp_line_no);
        return GNC_PARSER_OVER;
    }
    if (parser_data->itp_datas.count <= 0)
    {
        res = parser_handle_recieve(nullptr, &parser_data->itp_state, &parser_data->itp_line_no);
        parser_check_res(res);
        // if (res == ?)
    }
    else
    {
        for (int i = 0; i < parser_data->itp_datas.count; ++i)
        {
            pmc_data motion_data = parser_data->itp_datas.datas[i];
            parser_data->interp_list_index = i;
            res = parser_handle_recieve(&motion_data, &parser_data->itp_state, &parser_data->itp_line_no);
            CHECK_RES(parser_check_res(res));
        }
    }
    if (res == GNC_PARSER_WAIT)
    {
        ItpNextLine();
        return res;
    }
    if (ItpIsOver())
    {
        // LOG(LOG_LEVEL_WARN, "译码结束");
        if (res == GNC_CAL_SUCCESS || res == GNC_OPERATOR_SUCCESS || res == GNC_PARSER_OVER)
        {
            return GNC_PARSER_OVER;
        }
        return GNC_PARSER_OVER_AND_WAIT;
    } 
    ItpNextLine();
    return res;
}

static bool parser_check_res(GNC_RES res){
    switch(res){
        case GNC_PARSER_OVER:
            LOG(LOG_LEVEL_INFO, "译码结束");
            parser_reset();
            return false;
        case GNC_PARSER_WAIT:
            parser_data->parser_state = PARSER_WAITING_FOR_MOTION;
            return false;
        case GNC_PARSER_OVER_AND_WAIT:
            parser_data->parser_state = PARSER_WAITING_TO_END;
            return false;
        case GNC_CAL_SUCCESS:
        case GNC_OPERATOR_SUCCESS:
            if(parser_data->parser_state == PARSER_WAITING_FOR_BUF)
            {
                parser_data->parser_state = PARSER_READING;
            }
            // else if (parser_data->parser_state == PARSER_WAITING_TO_END)
            // {
            //     return parser_check_res(GNC_PARSER_OVER);
            // }
            return true;
        case GNC_DATA_FULL:
            if(parser_data->parser_state == PARSER_READING){
                parser_data->parser_state = PARSER_WAITING_FOR_BUF;
                return false;
            }
            return true;
        default:
            parser_data->parser_state = PARSER_IDLE;
            parser_reset();
            return false;
    }
}

static GNC_RES parser_handle_recieve(pmc_data* motion_data, NcItpState *itp_state, int *line_no){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    if (motion_data) {
        // 模式
        switch(motion_data->mode){
            case PMC_MODE_EXACT_STOP:       // 精准停模式
            case PMC_MODE_CHOP:             // 切削模式
            case PMC_MODE_ARRIVE_CHECK:     // 到位检测
            case PMC_MODE_TAP:              // 攻丝模式
            case PMC_MODE_PAUSE:            // 暂停模式
            case PMC_MODE_TAP_RETURE:       // 攻丝回退
            case PMC_MODE_STOPSKIP:         // 跳转
                res = parser_handle_gcode(motion_data, itp_state, line_no);
                break; 
            case PMC_MODE_M_CMD:            // M指令
                res = parser_handle_mcode(motion_data, itp_state, line_no);
                break;
            // 更新：其他模式
            default:
                res = GNC_OPERATOR_SUCCESS;
                break;
        }
    }
    else {  // 空包
        res = parser_handle_null(itp_state, line_no);
    }
    return res;
}

static GNC_RES parser_handle_null(NcItpState *itp_state, int *line_no){
    PLAN_BLOCK plan_block;
    // 模态
    plan_block.state = *itp_state;
    // 模组
    plan_block.mode = pmc_mode::PMC_MODE_NULL;
    // 行号
    plan_block.line_number = *line_no;
    plan_block.is_breakpoint = false;
    // LOG(LOG_LEVEL_DEBUG, "空包: no %d mode %d type %d", plan_block.line_number, plan_block.mode, plan_block.type);
    // 加入队列
    if(queue_is_full(parser_queue)){
        return GNC_DATA_FULL;
    }
    queue_push(parser_queue, &plan_block);
    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES parser_handle_gcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no)
{
    PLAN_BLOCK plan_block;
    // 插补类型
    switch(motion_data->interp){
        case PMC_G00:
            plan_block.type = RAPID;
            break;
        case PMC_G01:
            plan_block.type = LINE;
            break;
        case PMC_G02:
            plan_block.type = CIRCLE_CW;
            // 圆心位置
            plan_block.circular_data.center_pos.x = motion_data->circle_center.x_coc;
            plan_block.circular_data.center_pos.y = motion_data->circle_center.y_coc;
            plan_block.circular_data.center_pos.z = motion_data->circle_center.z_coc;
            // 平面
            plan_block.circular_data.plane = (PLANE)(motion_data->plane);
            break;
        case PMC_G03:
            plan_block.type = CIRCLE_CCW;
            plan_block.circular_data.center_pos.x = motion_data->circle_center.x_coc;
            plan_block.circular_data.center_pos.y = motion_data->circle_center.y_coc;
            plan_block.circular_data.center_pos.z = motion_data->circle_center.z_coc;
            plan_block.circular_data.plane = (PLANE)(motion_data->plane);
            break;
        case PMC_G02_5:
            plan_block.type = HELIX_CW;
            plan_block.circular_data.center_pos.x = motion_data->circle_center.x_coc;
            plan_block.circular_data.center_pos.y = motion_data->circle_center.y_coc;
            plan_block.circular_data.center_pos.z = motion_data->circle_center.z_coc;
            plan_block.circular_data.plane = (PLANE)(motion_data->plane);
            plan_block.helical_data.screw_number = motion_data->screw_number;
            plan_block.helical_data.screw_offset = motion_data->screw_offset;
            break;
        case PMC_G03_5:
            plan_block.type = HELIX_CCW;
            plan_block.circular_data.center_pos.x = motion_data->circle_center.x_coc;
            plan_block.circular_data.center_pos.y = motion_data->circle_center.y_coc;
            plan_block.circular_data.center_pos.z = motion_data->circle_center.z_coc;
            plan_block.circular_data.plane = (PLANE)(motion_data->plane);
            plan_block.helical_data.screw_number = motion_data->screw_number;
            plan_block.helical_data.screw_offset = motion_data->screw_offset;
            break;
        case PMC_G33_1: // 攻丝循环
            plan_block.type = RIGID_TAP;
            break;
        case PMC_G33_2: // 攻丝回退
            plan_block.type = TAP_RETURN;
            break;
        default:
            // 未支持的插补类型
            rt_set_parser_state(3001, *line_no);
            LOG(LOG_LEVEL_DEBUG, "不支持该插补类型: %d", motion_data->interp);
            return GNC_PARSER_OVER;
    }
    switch(motion_data->mode){
        case PMC_MODE_STOPSKIP:
            plan_block.type = PROBE;
            break;
        default:
            break;
    }
    // 进给速度
    plan_block.feed_vel = motion_data->feed.value / TIME_MINUS_2_S;
    // 位置
    plan_block.end_pos.x = motion_data->position.x;
    plan_block.end_pos.y = motion_data->position.y;
    plan_block.end_pos.z = motion_data->position.z;
    // 模态
    plan_block.state = *itp_state;
    // 模组
    plan_block.mode = motion_data->mode;
    // 行号
    plan_block.line_number = *line_no;
    // 暂停时间
    plan_block.pause_time = motion_data->pause_time;
    // 回参考点
    plan_block.ref_number = (REF_TYPE)motion_data->ref_point_number;
    plan_block.ref_axis = motion_data->ref_axis_flag;
    // 回断点
    plan_block.is_breakpoint = false;
    // 加入队列
    if(queue_is_full(parser_queue)){
        return GNC_DATA_FULL;
    }
    // ======= 测试 ===========
    // if(plan_block.type == CIRCLE_CW || plan_block.type == CIRCLE_CCW ){
    //     LOG(LOG_LEVEL_DEBUG, "center pos %f %f %f no %d", plan_block.circular_data.center_pos.x, plan_block.circular_data.center_pos.y, plan_block.circular_data.center_pos.z, plan_block.line_number);  
    // }
    // LOG(LOG_LEVEL_DEBUG, " no %d mode %d type:%d end pos %f %f %f", plan_block.line_number, plan_block.mode, plan_block.type, motion_data->position.x, motion_data->position.y, motion_data->position.z);
    // ======= 测试 ===========
    queue_push(parser_queue, &plan_block);
    if(plan_block.mode == pmc_mode::PMC_MODE_STOPSKIP) return GNC_PARSER_WAIT;

    return GNC_OPERATOR_SUCCESS;
}

static GNC_RES parser_handle_mcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no){
    PLAN_BLOCK plan_block;
    // 子程序调用M98和M99
    if(motion_data->m_code == 98 || motion_data->m_code == 99){
        return GNC_OPERATOR_SUCCESS;
    }
    // 进给速度
    plan_block.feed_vel = motion_data->feed.value;
    // 位置
    plan_block.end_pos.x = motion_data->position.x;
    plan_block.end_pos.y = motion_data->position.y;
    plan_block.end_pos.z = motion_data->position.z;
    // 模态
    plan_block.state = *itp_state;
    // 模组
    plan_block.mode = motion_data->mode;
    // 行号
    plan_block.line_number = *line_no;
    // M码
    plan_block.mst_data.m_code = motion_data->m_code;
    // 主轴
    plan_block.mst_data.spindle = motion_data->spindle;
    // 刀号
    plan_block.mst_data.tool_number = motion_data->tool_number; 
    // 子程序或宏程序
    plan_block.mst_data.sub_program = motion_data->sub_program;
    // 预留
    plan_block.mst_data.size = 1;
    plan_block.is_breakpoint = false;
    if (queue_is_full(parser_queue))
    {
        return GNC_DATA_FULL;
    }
    queue_push(parser_queue, &plan_block);
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES parser_waiting_for_buf(){
    if(queue_is_full(parser_queue)) return GNC_DATA_FULL;
    GNC_RES res = GNC_CAL_SUCCESS;
    int start_index = parser_data->interp_list_index;
    for(int i = start_index; i < parser_data->itp_datas.count; i++){
        pmc_data motion_data = parser_data->itp_datas.datas[i];
        parser_data->interp_list_index = i;
        res = parser_handle_recieve(&motion_data, &parser_data->itp_state, &parser_data->itp_line_no);
        CHECK_RES(parser_check_res(res));
    }
    return res;
}


GNC_RES parser_waiting_for_motion() {
    if(!rt_get_chan_state(0, CHAN_IS_MOTION_SYNC)) return GNC_CAL_SUCCESS;

    int start_index = parser_data->interp_list_index+1;
    for(int i = start_index; i < parser_data->itp_datas.count; i++){
        pmc_data motion_data = parser_data->itp_datas.datas[i];
        GNC_RES res = parser_handle_recieve(&motion_data, &parser_data->itp_state, &parser_data->itp_line_no);
        if(res == GNC_PARSER_WAIT)
        {
            parser_data->interp_list_index = i;
            rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, false);
            return GNC_CAL_SUCCESS;
        }
        if(!parser_check_res(res)) return GNC_CAL_SUCCESS;
        parser_data->interp_list_index = i;
    }
    rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, false);
    if(ItpIsOver())
    {
        return GNC_PARSER_OVER;
    }
    parser_data->parser_state = PARSER_READING;
    return GNC_CAL_SUCCESS;
}

GNC_RES parser_waiting_to_end() {
    GNC_RES res = GNC_CAL_SUCCESS;
    int start_index = parser_data->interp_list_index;
    for(int i = start_index; i < parser_data->itp_datas.count; i++){
        pmc_data motion_data = parser_data->itp_datas.datas[i];
        parser_data->interp_list_index = i;
        res = parser_handle_recieve(&motion_data, &parser_data->itp_state, &parser_data->itp_line_no);
        CHECK_RES(parser_check_res(res));
    }
    if (res == GNC_CAL_SUCCESS || res == GNC_OPERATOR_SUCCESS)
    {
        return GNC_PARSER_OVER;
    }
    return GNC_PARSER_OVER_AND_WAIT;
}

void parser_reset(){
    ItpReset();
    parser_data->parser_state = PARSER_IDLE;
    rt_set_chan_state(0, CHAN_IS_PARSERING, false);
    rt_set_chan_state(0, CHAN_IS_MOTION_SYNC, false);
    rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
}

int t_macroVarGetValue(int no, double *mac)
{
    // todo: 待更新shm后，将以下内容改为从shm获取
    // if (no < 0 || no >= MacroVarCount)
    // {
    //     return -1;
    // }
    // *mac = parser_data->cmv[no];
    // return 0;

    return macroVarGetValue(no, mac, 0);
}

int t_macroVarSetValue(int no, double mac)
{
    // todo: 待更新shm后，将以下内容改为从shm获取
    // if (no < 0 || no >= MacroVarCount)
    // {
    //     return -1;
    // }
    // parser_data->cmv[no] = mac;
    // return 0;
    
    return macroVarSetValue(no, mac, 0);
}

/*
1、G31

G90 G54 G40 G00 X1 Y1 Z1
G31 G90 X3.0 F100
Y100

*/