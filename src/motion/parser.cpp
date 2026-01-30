#include "parser.h"
#include "global.h"
#include "log.h"

STATIC void parser_macro_init();
STATIC GNC_RES parser_handle_recieve(pmc_data* motion_data);
STATIC GNC_RES parser_handle_gcode(pmc_data* motion_data);
STATIC GNC_RES parser_handle_mcode(pmc_data* motion_data);
STATIC GNC_RES parser_handle_tcode(pmc_data* motion_data);
STATIC GNC_RES parser_handle_scode(pmc_data* motion_data);
STATIC bool parser_check_res(GNC_RES res);

PARSER_DATA* parser_data;

STATIC void parser_macro_init()
{
    for (int i = 0; i < 30000; ++i)
    {
        parser_data->cmv[i].varDouble = 0.0;
    }
    parser_data->cmv[10004].varDouble= 10.0;     // 设置半径值超差
    parser_data->cmv[11600].varDouble= 0.0;      // 设置坐标旋转1轴运动计算方式
    parser_data->cmv[5410].varDouble= -90.0;     // 设置坐标旋转默认角度
    parser_data->cmv[17028].varDouble= 1.0;      // 默认米制单位
    parser_data->cmv[8132].varDouble= 1.0;       // 激活比例缩放功能G51
    parser_data->cmv[8077].varDouble= 10.0;      // 设置渐开线插补误差最大值
    parser_data->cmv[5114].varDouble= 0.5;       // G73钻深孔,D默认值
    parser_data->cmv[5115].varDouble= 1.0;       // G83深孔加工循环,D默认值
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
        LOG_OUTPUT(ERROR, "parser file path not exist");
        return GNC_OPERATOR_ERROR;
    }
    ItpReset();
    memcpy(parser_data->parser_file_path, parser_file_path, sizeof(char) * NCFILEPATH_CHAR_LEN);
    parser_data->parser_state = PARSER_READING;
    // 设置宏定义
    ItpSetMacroVariable(parser_data->cmv);
    // 设置机床起点
    ItpSetOriginMacPos(var_axis[AXIS_X].act_pos, var_axis[AXIS_Y].act_pos, var_axis[AXIS_Z].act_pos, 0);
    // 选择NC文件（支持中文路径）
    ItpSetFile(parser_file_path);
    return GNC_OPERATOR_SUCCESS;
}

STATIC GNC_RES parser_handle_recieve(pmc_data* motion_data){
    GNC_RES res = GNC_OPERATOR_SUCCESS;
    // 模态
    switch(motion_data->mode){
        case PMC_MODE_EXACT_STOP:
        case PMC_MODE_CHOP: // 切削模式
            res = parser_handle_gcode(motion_data);
            break; 
        case PMC_MODE_M_CMD: // M指令
            res = parser_handle_mcode(motion_data);
            break;
        // 更新：其他模态
        default:
            res = GNC_OPERATOR_SUCCESS;
            break;
    }
    return res;
}

STATIC GNC_RES parser_handle_gcode(pmc_data* motion_data)
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
    // 加入队列
    if(gcode_queue_is_full(gcode_queue)){
        LOG_OUTPUT(INFO, "gcode queue full");
        return GNC_DATA_FULL;
    }
    gcode_queue_push(gcode_queue, &gcode_block);
    // printf("type: %d, feed_vel: %.2f, pos:(%.2f, %.2f, %.2f)\n", gcode_block.type, gcode_block.feed_vel, gcode_block.pos.x, gcode_block.pos.y, gcode_block.pos.z);
    return GNC_OPERATOR_SUCCESS;
}

STATIC GNC_RES parser_handle_mcode(pmc_data* motion_data){
    // 更新：完成对M代码的处理
    return GNC_OPERATOR_SUCCESS;
}

GNC_RES parser_reading(){
    GNC_RES res = GNC_CAL_SUCCESS;
    ItpGetMotionData(&parser_data->itp_datas);
    ItpGetState(&parser_data->itp_state);
    if (parser_data->itp_datas.count > PARSER_BUF_SIZE) return GNC_CAL_ERROR;
    int error_no = ItpError();
    if (error_no >= 0)
    {
        LOG_OUTPUT(ERROR, "interpreter error: %d", error_no);   // 报警
        return GNC_CAL_ERROR;
    }
    if (parser_data->itp_datas.count <= 0)
    {
        LOG_OUTPUT(WARN, "interpreter res is empty"); // 空包
    }
    else
    {
        for (int i = 0; i < parser_data->itp_datas.count; ++i)
        {
            pmc_data motion_data = parser_data->itp_datas.datas[i];
            parser_data->interp_list_index = i;
            res = parser_handle_recieve(&motion_data);
            CHECK_RES(parser_check_res(res));
        }
    }
    if (ItpIsOver()) return GNC_PARSER_OVER;
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
        res = parser_handle_recieve(&motion_data);
        CHECK_RES(parser_check_res(res));
    }
    return res;
}

STATIC bool parser_check_res(GNC_RES res){
    switch(res){
        case GNC_PARSER_OVER:
            LOG_OUTPUT(INFO, "parser end");
            parser_data->parser_state = PARSER_IDLE;
            sys->is_parse_running = false;
            parser_reset();
            return false;
        case GNC_CAL_SUCCESS:
        case GNC_OPERATOR_SUCCESS:
            if(parser_data->parser_state == PARSER_WAITING_FOR_BUF){
                parser_data->parser_state = PARSER_READING;
                // return false;
            }     
            return true;
        case GNC_DATA_FULL:
            // LOG_OUTPUT(INFO, "parser waiting for buf");
            if(parser_data->parser_state == PARSER_READING){
                parser_data->parser_state = PARSER_WAITING_FOR_BUF;
                return false;
            }
            return true;
        default:
            LOG_OUTPUT(ERROR, "parser error: %d", res);
            parser_data->parser_state = PARSER_IDLE;
            // todo：译码出错让运动暂停后报警，而不是直接进入报警
            var_chan[0].is_estop = true;
            sys->current_mode = sys->target_mode = MODE_ALARM;
            LOG_OUTPUT(ERROR, "current_mode: ESTOP !!!");
            parser_reset();
            return false;
    }
}

void parser_run_mode(){
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
        
        default:
            break;
    }
}

void parser_reset(){
    ItpReset();
    parser_data->parser_state = PARSER_IDLE;
}
