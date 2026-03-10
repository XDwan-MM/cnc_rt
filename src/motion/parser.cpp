#include "parser.h"
#include "global.h"
#include "log.h"
#include <logapi.h>
#include <alarmapi.h>
#include <hmisysapi.h>

STATIC void parser_macro_init();
STATIC GNC_RES parser_handle_recieve(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
STATIC GNC_RES parser_handle_mcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
STATIC GNC_RES parser_handle_tcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
STATIC GNC_RES parser_handle_scode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
STATIC GNC_RES parser_handle_gcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no);
STATIC bool parser_check_res(GNC_RES res);

PARSER_DATA* parser_data;

STATIC void parser_macro_init()
{
    for (int i = 0; i < 30000; ++i) {
        parser_data->cmv[i].varDouble = 0;
        parser_data->cmv[i].valid = false;
        parser_data->cmv[i].read = 6;
        parser_data->cmv[i].write = 6;
    }
    parser_data->cmv[18004].varDouble = 10.0; // 设置半径值超差
    parser_data->cmv[17031].varDouble = 0.0;  // 设置坐标旋转1轴运动计算方式
    parser_data->cmv[5410].varDouble = -90.0; // 设置坐标旋转默认角度
    parser_data->cmv[17028].varDouble = 1.0;  // 默认米制单位
    parser_data->cmv[8132].varDouble = 1.0;   // 激活比例缩放功能G51
    parser_data->cmv[8077].varDouble = 10.0;  // 设置渐开线插补误差最大值
    parser_data->cmv[5114].varDouble = 0.5;   // G73钻深孔,D默认值
    parser_data->cmv[5115].varDouble = 1.0;   // G83深孔加工循环,D默认值

    // 坐标偏置
    for (int a = 0; a < 3; ++a)
    {
        getCrdsValue(CRDS_TYPE_EXT, a, &parser_data->cmv[5201 + a].varDouble);
    }
    for (int t = 0; t < 6; ++t)
    {
        for (int a = 0; a < 3; ++a)
        {
            getCrdsValue(CRDS_TYPE_G5X + t, a, &parser_data->cmv[5221 + t * 20 + a].varDouble);
        }
    }
}

void parser_new(){
    parser_data = new PARSER_DATA();
    // 设置译码参数
    parser_macro_init();
    parser_data->parser_state = PARSER_IDLE;
}

GNC_RES parser_init(const char* parser_file_path)
{   
    if(parser_file_path == NULL) {
        // log_system("[parser] 译码文件路径不存在");
        LOG_OUTPUT(ERROR, "译码文件路径不存在");
        setAlarm(ALARM_CH, ALARM_ERROR, 8);
        return GNC_OPERATOR_ERROR;
    }
    ItpReset();
    memcpy(parser_data->parser_file_path, parser_file_path, sizeof(char) * NCFILEPATH_CHAR_LEN);
    parser_data->parser_state = PARSER_READING;
    // 设置宏定义
    parser_macro_init();
    ItpSetMacroVariable(parser_data->cmv);
    // 设置机床起点
    double ori_pos_x, ori_pos_y, ori_pos_z;
    getVarChanDouble(AXIS_X, VAR_AXIS_CMD_POS_EX, &ori_pos_x, 0);
    getVarChanDouble(AXIS_Y, VAR_AXIS_CMD_POS_EX, &ori_pos_y, 0);
    getVarChanDouble(AXIS_Z, VAR_AXIS_CMD_POS_EX, &ori_pos_z, 0);
    ItpSetOriginMacPos(ori_pos_x, ori_pos_y, ori_pos_z, 0);
    // 选择NC文件（支持中文路径）
    ItpSetFile(parser_file_path);
    // 将GM指令设置为已完成
    rt_set_chan_state(0, CHAN_IS_GCODE_END, true);
    rt_set_chan_state(0, CHAN_IS_MCODE_END, true);
    return GNC_OPERATOR_SUCCESS;
}

STATIC GNC_RES parser_handle_recieve(pmc_data* motion_data, NcItpState *itp_state, int *line_no){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    // 模式
    switch(motion_data->mode){
        case PMC_MODE_EXACT_STOP:
        case PMC_MODE_CHOP: // 切削模式
        case PMC_MODE_ARRIVE_CHECK: // 到位检测
            res = parser_handle_gcode(motion_data, itp_state, line_no);
            break; 
        case PMC_MODE_M_CMD: // M指令
            res = parser_handle_mcode(motion_data, itp_state, line_no);
            break;
        // 更新：其他模式
        default:
            res = GNC_OPERATOR_SUCCESS;
            break;
    }
    return res;
}

STATIC GNC_RES parser_handle_gcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no)
{
    GCODE_BLOCK gcode_block;
    // 插补类型
    switch(motion_data->interp){
        case PMC_G00:
            gcode_block.type = RAPID;
            break;
        case PMC_G01:
            gcode_block.type = LINE;
            break;
        case PMC_G02:
            gcode_block.type = CIRCLE_CW;
            // 圆心位置
            gcode_block.center_pos.x = motion_data->circle_center.x_coc;
            gcode_block.center_pos.y = motion_data->circle_center.y_coc;
            gcode_block.center_pos.z = motion_data->circle_center.z_coc;
            // 平面
            gcode_block.plane = (PLANE)(motion_data->plane);
            break;
        case PMC_G03:
            gcode_block.type = CIRCLE_CCW;
            // 圆心位置
            gcode_block.center_pos.x = motion_data->circle_center.x_coc;
            gcode_block.center_pos.y = motion_data->circle_center.y_coc;
            gcode_block.center_pos.z = motion_data->circle_center.z_coc;
            // 平面
            gcode_block.plane = (PLANE)(motion_data->plane);
            break;
        default:
            break;
    }
    // 进给速度
    gcode_block.feed_vel = motion_data->feed.value;
    // 位置
    gcode_block.pos.x = motion_data->position.x;
    gcode_block.pos.y = motion_data->position.y;
    gcode_block.pos.z = motion_data->position.z;
    // 模态
    gcode_block.state = *itp_state;
    // 行号
    gcode_block.line_number = *line_no;
    // 加入队列
    if(gcode_queue_is_full(gcode_queue)){
        return GNC_DATA_FULL;
    }
    // 判断M代码是否已处理完成
    if (!rt_get_chan_state(0, CHAN_IS_MCODE_END))
    {
        return GNC_PARSER_WAIT;
    }
    gcode_queue_push(gcode_queue, &gcode_block);
    rt_set_chan_state(0, CHAN_IS_GCODE_END, false);
    // printf("type: %d, feed_vel: %.2f, pos:(%.2f, %.2f, %.2f)\n", gcode_block.type, gcode_block.feed_vel, gcode_block.pos.x, gcode_block.pos.y, gcode_block.pos.z);
    return GNC_OPERATOR_SUCCESS;
}

STATIC GNC_RES parser_handle_mcode(pmc_data* motion_data, NcItpState *itp_state, int *line_no){
    MCODE_BLOCK mcode_block;
    // M码
    mcode_block.m_code = motion_data->m_code;
    // 主轴
    mcode_block.spindle = motion_data->spindle;
    // 模态
    mcode_block.state = *itp_state;
    // 行号
    mcode_block.line_number = *line_no;
    if (mcode_queue_is_full(mcode_queue))
    {
        return GNC_DATA_FULL;
    }
    // 判断G代码是否已处理完成
    if (!rt_get_chan_state(0, CHAN_IS_GCODE_END))
    {
        return GNC_PARSER_WAIT;
    }
    if (!rt_get_chan_state(0, CHAN_IS_MCODE_END))
    {
        return GNC_PARSER_WAIT;
    }
    mcode_queue_push(mcode_queue, &mcode_block);
    rt_set_chan_state(0, CHAN_IS_MCODE_END, false);

    return GNC_OPERATOR_SUCCESS;
}

GNC_RES parser_reading(){
    GNC_RES res = GNC_CAL_SUCCESS;
    ItpGetMotionData(&parser_data->itp_datas);
    ItpGetState(&parser_data->itp_state);
    ItpGetLineNumber(&parser_data->itp_line_no);
    if (parser_data->itp_datas.count > PARSER_BUF_SIZE) return GNC_CAL_ERROR;
    int error_no = ItpError();
    if (error_no >= 0)
    {
        // log_system("[parser] 译码错误(%d)", error_no);
        setAlarm(ALARM_PS, ALARM_ERROR, error_no);
        LOG_OUTPUT(ERROR, "译码错误 %d", error_no);
        return GNC_CAL_ERROR;
    }
    if (parser_data->itp_datas.count <= 0)
    {
        // log_system("[parser] 译码为空");
        LOG_OUTPUT(WARN, "译码为空"); // 空包
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
    if (ItpIsOver())
    {
        if (res == GNC_CAL_SUCCESS || res == GNC_OPERATOR_SUCCESS)
        {
            return GNC_PARSER_OVER;
        }
        return GNC_PARSER_OVER_AND_WAIT;
    } 
    ItpNextLine();
    return res;
}

GNC_RES parser_waiting_for_buf(){
    if(gcode_queue_is_full(gcode_queue)) return GNC_DATA_FULL;
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

STATIC bool parser_check_res(GNC_RES res){
    switch(res){
        case GNC_PARSER_OVER:
            // log_system("[parser] 译码结束");
            LOG_OUTPUT(INFO, "译码结束");
            rt_set_chan_state(0, CHAN_IS_PARSERING, false);
            rt_set_chan_state(0, CHAN_IS_PARSER_RERUN, false);
            parser_data->parser_state = PARSER_IDLE;
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
            if(parser_data->parser_state == PARSER_WAITING_FOR_BUF || 
                parser_data->parser_state == PARSER_WAITING_FOR_MOTION)
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
            // todo：译码出错让运动暂停后报警，而不是直接进入报警
            // log_system("[parser] 译码报警(%d)", res);
            LOG_OUTPUT(ERROR, "译码报警 %d", res);
            setAlarm(ALARM_CH, ALARM_ERROR, 2);
            parser_reset();
            return false;
    }
}

void parser_run_mode(){
    if (rt_get_chan_state(0, CHAN_IS_PARSER_RESET) == true)
    {
        parser_reset();
        gcode_queue_reset(gcode_queue);
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

void parser_reset(){
    ItpReset();
    parser_data->parser_state = PARSER_IDLE;
}
