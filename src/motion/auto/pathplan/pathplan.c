#include <stdlib.h>
#include <math.h>
#include "trajectory.h"
#include "pathplan.h"
#include "global.h"
#include "cnc_math.h"
#include <hmisysapi.h>
#include "rt_data.h"
#include "rerun.h"
#include <alarmapi.h>

static void block_cal_pos(PLAN_BLOCK* parser_block, PLAN_BLOCK* current_block, POS* start_pos);
static void block_cal_vel(PLAN_BLOCK* current_block);
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector);
static void block_cal_circle_vector(PLAN_BLOCK* current_block);
static double block_cal_circle_length(PLAN_BLOCK* current_block);
static void block_deal_run_row(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block);
static void block_deal_parser_block(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block);

// 将译码数据转换成段
GNC_RES chan_add_block(){
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* parser_block;

    if(queue_is_empty(parser_queue)) return GNC_DATA_EMPTY;

    // 每次最多处理 PRE_TREAT_SIZE
    for(int i = 0; i < PRE_TREAT_SIZE; i++){
        // 如果预处理队列已满，提前结束
        if(queue_is_full(pretreat_queue)) return GNC_DATA_FULL;
        // 如果译码器无数据，提前结束
        if(queue_is_empty(parser_queue)) return GNC_DATA_EMPTY;

        parser_block = queue_peak(parser_queue);
        if(parser_block == NULL) return GNC_DATA_NULL;
        pretreat_block = queue_peak_tail(pretreat_queue);
        if(pretreat_block == NULL) return GNC_DATA_NULL;

        block_deal_run_row(parser_block, pretreat_block);
    }
    return GNC_CAL_SUCCESS;
}
   
static void block_deal_run_row(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block){
    if(rerun_data->is_rerun){
        if(parser_block->line_number < rerun_data->rerun_row) rerun_data->row_state = BEFORE_THE_RERUN_ROW;
        else if(parser_block->line_number == rerun_data->rerun_row) rerun_data->row_state = EXACT_THE_RERUN_ROW;
        else rerun_data->row_state = AFTER_THE_RERUN_ROW;
    }else rerun_data->row_state = AFTER_THE_RERUN_ROW;
    switch (rerun_data->row_state)
    {
    // 再启动行号前
    case BEFORE_THE_RERUN_ROW:
        queue_pop(parser_queue);
        break;

    // 再启动行号
    case EXACT_THE_RERUN_ROW:
        // 再启动断点
        if(!check_rerun_row_is_valid()) break;
        // fanuc恢复辅助功能
        for(int i = 0; i < rerun_data->record_mcode.length; i++){
            if(record_queue_is_empty(&rerun_data->record_mcode)) break;
            PLAN_BLOCK* mcode_block = record_queue_peak(&rerun_data->record_mcode);
            mcode_block->line_number = rerun_data->rerun_row;
            pretreat_block = queue_peak_tail(pretreat_queue);
            memcpy(pretreat_block, mcode_block, sizeof(PLAN_BLOCK));
            block_deal_parser_block(mcode_block, pretreat_block);
            record_queue_pop(&rerun_data->record_mcode);
            queue_soft_push(pretreat_queue);
            LOG(LOG_LEVEL_DEBUG, "恢复第%d个辅助指令M%d", i+1, mcode_block->mst_data.m_code);
        }
        pretreat_block = queue_peak_tail(pretreat_queue);
        PLAN_BLOCK* rerun_block = (PLAN_BLOCK*)calloc(sizeof(PLAN_BLOCK), 1);
        if(rerun_block == NULL) break;
        // 华中回到断点位置
        create_rerun_block(rerun_block);
        memcpy(pretreat_block, rerun_block, sizeof(PLAN_BLOCK));
        block_deal_parser_block(rerun_block, pretreat_block);
        // fanuc回到程序终点
        // memcpy(pretreat_block, &rerun_data->break_block, sizeof(PLAN_BLOCK));
        // block_deal_parser_block(&rerun_data->break_block, pretreat_block);
        queue_soft_push(pretreat_queue);
        if(queue_is_full(pretreat_queue)) break;
        free(rerun_block);
        rerun_block = NULL;
        rerun_reset_is_rerun();
        break;

    case AFTER_THE_RERUN_ROW:
        memcpy(pretreat_block, parser_block, sizeof(PLAN_BLOCK));
        block_deal_parser_block(parser_block, pretreat_block);
        // 删除已处理的gcode
        queue_pop(parser_queue);
        // 更新预处理队列信息
        queue_soft_push(pretreat_queue);
        break;
    default:
        break;
    }
}

static void block_deal_parser_block(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block){
    switch (parser_block->mode)
    {
    // 处理空包和非移动指令
    case PMC_MODE_NULL:
    case PMC_MODE_M_CMD:
    case PMC_MODE_PAUSE:
        pretreat_block->max_vel = pretreat_block->feed_vel = 0;
        pretreat_block->length = 0;
        pretreat_block->angle_rad = PI;
        break;
    // 处理移动指令 
    default:
        // 构建起点
        if(rt_get_chan_state(0, CHAN_IS_FIRST_RUN)){
            // todo：var变量在路径线程中被读取，存在数据竞争风险，需加锁保护
            pretreat_queue->pre_pos = (POS){.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, .y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, .z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur};
            rt_set_chan_state(0, CHAN_IS_FIRST_RUN, false);
            // LOG(LOG_LEVEL_DEBUG,  "%f %f %f", pretreat_queue->pre_pos.x, pretreat_queue->pre_pos.y, pretreat_queue->pre_pos.z);
        }
        // 构造成段
        pretreat_block->max_vel = pretreat_block->feed_vel = parser_block->feed_vel;
        block_cal_pos(parser_block, pretreat_block, &pretreat_queue->pre_pos);
        // 计算速度
        block_cal_vel(pretreat_block);
        // 计算切矢
        block_cal_vector(pretreat_block);
        // 计算长度
        block_cal_length(pretreat_block);
        // 计算两段衔接夹角
        block_cal_angle(pretreat_block, &pretreat_queue->pre_vec);
        // 更新上一段终点
        pretreat_queue->pre_pos = pretreat_block->end_pos;
        pretreat_queue->pre_vec = pretreat_block->end_vec;
        break;
    }
}

// 计算段位置信息
static void block_cal_pos(PLAN_BLOCK* parser_block, PLAN_BLOCK* current_block, POS* start_pos){
    POS_COPY(&current_block->start_pos, start_pos);
    POS_COPY(&current_block->end_pos, &parser_block->end_pos);
    // LOG(LOG_LEVEL_DEBUG,  "start_pos %f %f %f", current_block->start_pos.x, current_block->start_pos.y, current_block->start_pos.z);
    // LOG(LOG_LEVEL_DEBUG,  "end_pos %f %f %f", current_block->end_pos.x, current_block->end_pos.y, current_block->end_pos.z);
    // 计算顺逆方向
    switch(parser_block->type){
        case CIRCLE_CCW:
        case HELIX_CCW:
            current_block->circular_data.dir = COUNTERCLOCK;
            POS_COPY(&current_block->circular_data.center_pos, &parser_block->circular_data.center_pos);
            current_block->plane = parser_block->plane;
            break;
        case CIRCLE_CW:
        case HELIX_CW:
            current_block->circular_data.dir = CLOCK;
            POS_COPY(&current_block->circular_data.center_pos, &parser_block->circular_data.center_pos);
            current_block->plane = parser_block->plane;
            break;
        default:
            break;
    }
    // 计算半径
    switch(parser_block->type){
        case CIRCLE_CCW:
        case CIRCLE_CW:
            current_block->circular_data.radius = POS_Distance(&current_block->end_pos, &current_block->circular_data.center_pos);
            break;
        case HELIX_CCW:
        case HELIX_CW:
            POS helix_circle_pos = {0};
            POS_COPY(&helix_circle_pos, &current_block->end_pos);
            switch (current_block->plane)
            {
            case PLANE_XY:
                helix_circle_pos.z = current_block->circular_data.center_pos.z;
                current_block->circular_data.radius = POS_Distance(&helix_circle_pos, &current_block->circular_data.center_pos);
                break;
            case PLANE_XZ:
                helix_circle_pos.y = current_block->circular_data.center_pos.y;
                current_block->circular_data.radius = POS_Distance(&helix_circle_pos, &current_block->circular_data.center_pos);
                break;
            case PLANE_YZ:
                helix_circle_pos.x = current_block->circular_data.center_pos.x;
                current_block->circular_data.radius = POS_Distance(&helix_circle_pos, &current_block->circular_data.center_pos);
                break;
            default:
                break;
            }
            break;
        default:
            break;
    }
}

// 计算段最大允许速度
static void block_cal_vel(PLAN_BLOCK* current_block){
    double min_feed = MIN3(rt_data->axis_data[0].max_feed_vel, rt_data->axis_data[1].max_feed_vel, rt_data->axis_data[2].max_feed_vel);
    double min_rapid_feed = MIN3(MIN(rt_data->axis_data[0].jog_lowspd, rt_data->axis_data[0].max_rapid_vel), MIN(rt_data->axis_data[1].jog_lowspd, rt_data->axis_data[1].max_rapid_vel), MIN(rt_data->axis_data[2].jog_lowspd, rt_data->axis_data[2].max_rapid_vel));  // G00最大速度为设置的快速进给速度
    switch(current_block->type){
        case RAPID:
            current_block->max_vel = current_block->feed_vel = min_rapid_feed;
            current_block->max_acc = MIN3(rt_data->axis_data[0].max_rapid_acc, rt_data->axis_data[1].max_rapid_acc, rt_data->axis_data[2].max_rapid_acc);
            current_block->max_jerk = MIN3(rt_data->axis_data[0].max_rapid_jk, rt_data->axis_data[1].max_rapid_jk, rt_data->axis_data[3].max_rapid_jk);
            break;
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
        case PROBE:
            current_block->max_vel = current_block->feed_vel = MIN(current_block->feed_vel, min_feed);   // G01最大速度
            current_block->max_acc = MIN3(rt_data->axis_data[0].max_feed_acc, rt_data->axis_data[1].max_feed_acc, rt_data->axis_data[2].max_feed_acc);
            current_block->max_jerk = MIN3(rt_data->axis_data[0].max_feed_jk, rt_data->axis_data[1].max_feed_jk, rt_data->axis_data[2].max_feed_jk);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            current_block->max_acc = MIN3(rt_data->axis_data[0].max_feed_acc, rt_data->axis_data[1].max_feed_acc, rt_data->axis_data[2].max_feed_acc);
            current_block->max_jerk = MIN3(rt_data->axis_data[0].max_feed_jk, rt_data->axis_data[1].max_feed_jk, rt_data->axis_data[2].max_feed_jk);
            double vc = vp_cal_circular_max_vel(current_block, rt_data->chan_data[0].path_tolerance, rt_data->chan_data[0].max_fv_arc_radius, rt_data->chan_data[0].radius_r_max_fv, rt_data->chan_data[0].circular_min_vel, rt_data->ncu_data.cycle);
            current_block->max_vel = current_block->feed_vel = MIN3(current_block->feed_vel, min_feed, vc);   // G02G03最大速度
            break;
        default:
            break;
    }
}

static double cal_two_vectors_angle(VECTOR* vector1, VECTOR* vector2){
    double norm1 = POS_Norm(*vector1);
    double norm2 = POS_Norm(*vector2);
    // 注意：如果norm为0，该段长度为0
    if(IsEqualTo(norm1, 0) || IsEqualTo(norm2, 0)) return 0;
    double cos_angle = POS_Dot(*vector1, *vector2) / (norm1 * norm2);
    // 对于数值误差导致的cos_angle超出[-1, 1]范围进行修正，避免acos函数返回NaN
    if(cos_angle > 1.0) cos_angle = 1.0;
    else if(cos_angle < -1.0) cos_angle = -1.0;
    return acos(cos_angle);
}

// 根据切失计算两段之间的衔接角度
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector){
    switch(current_block->type){
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            double angle_rad_start = cal_two_vectors_angle(pre_end_vector, &current_block->start_vec);
            double angle_rad_end = cal_two_vectors_angle(pre_end_vector, &current_block->end_vec);
            current_block->angle_rad = fmax(angle_rad_start, angle_rad_end);
            // LOG(LOG_LEVEL_DEBUG, "no %d, angle:%f", current_block->line_number, current_block->angle_rad);
            break;
        case RAPID:
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
        case PROBE:
        default:
            current_block->angle_rad = cal_two_vectors_angle(pre_end_vector, &current_block->start_vec);
            break;
    }
}

static void block_cal_circle_vector(PLAN_BLOCK* current_block){     
    // 计算半径矢量
    VECTOR rs = VECTOR_Unit(&current_block->circular_data.center_pos, &current_block->start_pos);
    VECTOR re = VECTOR_Unit(&current_block->circular_data.center_pos, &current_block->end_pos);

    switch(current_block->plane){
        case PLANE_XY:
            switch(current_block->type){
            case CIRCLE_CW:
            case HELIX_CW:
                current_block->start_vec.x = rs.y;
                current_block->start_vec.y = - rs.x;
                current_block->start_vec.z = 0;
                current_block->end_vec.x = re.y;
                current_block->end_vec.y = - re.x;
                current_block->end_vec.z = 0;
                break;
            case CIRCLE_CCW:
            case HELIX_CCW:
                current_block->start_vec.x = - rs.y;
                current_block->start_vec.y = rs.x;
                current_block->start_vec.z = 0;
                current_block->end_vec.x = - re.y;
                current_block->end_vec.y = re.x;
                current_block->end_vec.z = 0;
                break;
            }
        break;
        case PLANE_XZ:
            switch(current_block->type){
            case CIRCLE_CW:
            case HELIX_CW:
                current_block->start_vec.x = rs.z;
                current_block->start_vec.y = 0;
                current_block->start_vec.z = - rs.x;
                current_block->end_vec.x = re.z;
                current_block->end_vec.y = 0;
                current_block->end_vec.z = - re.x;
                break;
            case CIRCLE_CCW:
            case HELIX_CCW:
                current_block->start_vec.x = - rs.z;
                current_block->start_vec.y = 0;
                current_block->start_vec.z = rs.x;
                current_block->end_vec.x = - re.z;
                current_block->end_vec.y = 0;
                current_block->end_vec.z = re.x;
                break;
            }
        break;
        case PLANE_YZ:
            switch(current_block->type){
            case CIRCLE_CW:
            case HELIX_CW:
                current_block->start_vec.x = 0;
                current_block->start_vec.y = rs.z;
                current_block->start_vec.z = - rs.y;
                current_block->end_vec.x = 0;
                current_block->end_vec.y = re.z;
                current_block->end_vec.z = - re.y;
                break;
            case CIRCLE_CCW:
            case HELIX_CCW:
                current_block->start_vec.x = 0;
                current_block->start_vec.y = - rs.z;
                current_block->start_vec.z = rs.y;
                current_block->end_vec.x = 0;
                current_block->end_vec.y = - re.z;
                current_block->end_vec.z = re.y;
                break;
            }
        break;
    }
}

static double block_cal_circle_length(PLAN_BLOCK* current_block){
    VECTOR rs = POS_VECTOR(&current_block->circular_data.center_pos, &current_block->start_pos);
    VECTOR re = POS_VECTOR(&current_block->circular_data.center_pos, &current_block->end_pos);
    // 计算切失
    switch(current_block->plane){
        case PLANE_XY:
            current_block->circular_data.statr_rad = atan2(rs.y, rs.x);
            current_block->circular_data.end_rad = atan2(re.y, re.x);
            break;
        case PLANE_XZ:
            current_block->circular_data.statr_rad = atan2(rs.z, rs.x);
            current_block->circular_data.end_rad = atan2(re.z, re.x);
            break;
        case PLANE_YZ:
            current_block->circular_data.statr_rad = atan2(rs.z, rs.y);
            current_block->circular_data.end_rad = atan2(re.z, re.y);
            break;
    }
    // 计算弧度
    double angDelta = current_block->circular_data.end_rad - current_block->circular_data.statr_rad;
    switch(current_block->type){
        case CIRCLE_CW:
        case HELIX_CW:
            if (IsGreaterThanOrEqual(angDelta, 0)) angDelta -= 2 * PI;
            break;
        case CIRCLE_CCW:
        case HELIX_CCW:
            if (IsLessThanOrEqual(angDelta, 0)) angDelta += 2 * PI;
            break;
        default:
            break;
    }
    current_block->circular_data.rad = fabs(angDelta);
    // 计算长度
    switch(current_block->type){
        case CIRCLE_CW:
        case CIRCLE_CCW:
            return current_block->circular_data.rad * current_block->circular_data.radius;
        case HELIX_CW:
        case HELIX_CCW:
            current_block->helical_data.screw_rad = atan2(current_block->helical_data.screw_offset, current_block->circular_data.rad * current_block->circular_data.radius);
            return sqrt(pow(current_block->circular_data.rad * current_block->circular_data.radius, 2) + pow(current_block->helical_data.screw_offset, 2));
        default:
            return 0;
    }
}

// 计算段长度|圆弧长
void block_cal_length(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
        case PROBE:
            current_block->length = POS_Distance(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            current_block->length = block_cal_circle_length(current_block);
            break;
        default:
            break;
    }
}

// 计算段首末切失
void block_cal_vector(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
        case PROBE:
            current_block->end_vec = current_block->start_vec = VECTOR_Unit(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case HELIX_CW:
        case HELIX_CCW:
            block_cal_circle_vector(current_block);
            return;
        default:
            return;
    }
}
