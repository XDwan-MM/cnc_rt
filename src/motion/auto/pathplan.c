#include <stdlib.h>
#include <math.h>
#include "trajectory.h"
#include "pathplan.h"
#include "global.h"
#include "cnc_math.h"
#include <hmisysapi.h>
#include "rt_data.h"
#include <alarmapi.h>

static void block_cal_pos(PLAN_BLOCK* parser_block, PLAN_BLOCK* current_block, POS* start_pos);
static void block_cal_vel(PLAN_BLOCK* current_block);
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector);
static void block_cal_circle_vector(PLAN_BLOCK* current_block);
static double block_cal_circle_length(PLAN_BLOCK* current_block);

// 将译码数据转换成段
GNC_RES chan_add_block(){
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* parser_block;

    if(queue_is_empty(parser_queue)) return GNC_DATA_EMPTY;

    // 译码第一段类型为g00，起点为当前机床位置
    if(rt_get_chan_state(0, CHAN_IS_FIRST_RUN)){
        // todo：var变量在路径线程中被读取，存在数据竞争风险，需加锁保护
        pretreat_queue->pre_pos = (POS){.x = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur, .y = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur, .z = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur};
        rt_set_chan_state(0, CHAN_IS_FIRST_RUN, false);
        // LOG(LOG_LEVEL_DEBUG,  "%f %f %f", pretreat_queue->pre_pos.x, pretreat_queue->pre_pos.y, pretreat_queue->pre_pos.z);
    }

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

        // 拷贝信息
        memcpy(pretreat_block, parser_block, sizeof(PLAN_BLOCK));

        // 构造成段
        pretreat_block->max_vel = pretreat_block->feed_vel = parser_block->feed_vel / TIME_MINUS_2_S;
        block_cal_pos(parser_block, pretreat_block, &pretreat_queue->pre_pos);

        // 计算速度
        block_cal_vel(pretreat_block);

        // 计算切矢
        block_cal_vector(pretreat_block);

        // 计算长度
        block_cal_length(pretreat_block);

        // 计算两段衔接夹角
        block_cal_angle(pretreat_block, &pretreat_queue->pre_vec, &pretreat_block->start_vec);

        // 删除已处理的gcode
        queue_pop(parser_queue);

        // 更新预处理队列信息
        queue_soft_push(pretreat_queue);
        pretreat_queue->pre_pos = pretreat_block->end_pos;
        pretreat_queue->pre_vec = pretreat_block->end_vec;
    }
    return GNC_CAL_SUCCESS;
}

// 计算段的首末位置
static void block_cal_pos(PLAN_BLOCK* parser_block, PLAN_BLOCK* current_block, POS* start_pos){
    POS_COPY(&current_block->start_pos, start_pos);
    POS_COPY(&current_block->end_pos, &parser_block->end_pos);
    // LOG(LOG_LEVEL_DEBUG,  "start_pos %f %f %f", current_block->start_pos.x, current_block->start_pos.y, current_block->start_pos.z);
    // LOG(LOG_LEVEL_DEBUG,  "end_pos %f %f %f", current_block->end_pos.x, current_block->end_pos.y, current_block->end_pos.z);
    switch(parser_block->type){
        case CIRCLE_CCW:
        case CIRCLE_CW:
            // 计算圆心和半径
            POS_COPY(&current_block->circular_data.center_pos, &parser_block->circular_data.center_pos);
            current_block->circular_data.radius = POS_Distance(&current_block->end_pos, &current_block->circular_data.center_pos);
            current_block->circular_data.plane = parser_block->circular_data.plane;
            current_block->circular_data.dir = (parser_block->type == CIRCLE_CW) ? CLOCK : COUNTERCLOCK;
            break;
        default:
            break;
    }
}

// 计算段最大允许速度
static void block_cal_vel(PLAN_BLOCK* current_block){
    double min_feed = MIN3(par_axis[0][PAR_AX_FEED_SPD].value.val_real, par_axis[1][PAR_AX_FEED_SPD].value.val_real, par_axis[2][PAR_AX_FEED_SPD].value.val_real);
    double min_rapid_feed = MIN3(MIN(par_axis[0][PAR_AX_JOG_LOWSPD].value.val_real, par_axis[0][PAR_AX_RAPID_SPD].value.val_real), MIN(par_axis[1][PAR_AX_JOG_LOWSPD].value.val_real, par_axis[1][PAR_AX_RAPID_SPD].value.val_real), MIN(par_axis[2][PAR_AX_JOG_LOWSPD].value.val_real, par_axis[2][PAR_AX_RAPID_SPD].value.val_real));  // G00最大速度为设置的快速进给速度
    switch(current_block->type){
        case RAPID:
            current_block->max_vel = current_block->feed_vel = min_rapid_feed;
            current_block->max_acc = MIN3(par_axis[0][PAR_AX_RAPID_ACC].value.val_real, par_axis[1][PAR_AX_RAPID_ACC].value.val_real, par_axis[2][PAR_AX_RAPID_ACC].value.val_real);
            current_block->max_jerk = MIN3(par_axis[0][PAR_AX_RAPID_JK].value.val_real, par_axis[1][PAR_AX_RAPID_JK].value.val_real, par_axis[2][PAR_AX_RAPID_JK].value.val_real);
            break;
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
            current_block->max_vel = current_block->feed_vel = MIN(current_block->feed_vel, min_feed);   // G01最大速度
            current_block->max_acc = MIN3(par_axis[0][PAR_AX_FEED_ACC].value.val_real, par_axis[1][PAR_AX_FEED_ACC].value.val_real, par_axis[2][PAR_AX_FEED_ACC].value.val_real);
            current_block->max_jerk = MIN3(par_axis[0][PAR_AX_FEED_JK].value.val_real, par_axis[1][PAR_AX_FEED_JK].value.val_real, par_axis[2][PAR_AX_FEED_JK].value.val_real);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            current_block->max_acc = MIN3(par_axis[0][PAR_AX_FEED_ACC].value.val_real, par_axis[1][PAR_AX_FEED_ACC].value.val_real, par_axis[2][PAR_AX_FEED_ACC].value.val_real);
            current_block->max_jerk = MIN3(par_axis[0][PAR_AX_FEED_JK].value.val_real, par_axis[1][PAR_AX_FEED_JK].value.val_real, par_axis[2][PAR_AX_FEED_JK].value.val_real);
            double vc = vp_cal_circular_max_vel(current_block, par_chan[0][PAR_CH_PATH_TOLERANCE].value.val_real, (double)par_ncu[PAR_NCU_CYCLE].value.val_uint / TIME_US_2_S);
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
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector){
    switch(current_block->type){
        case RAPID:
        case LINE:
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case RIGID_TAP:
        case TAP_RETURN:
            current_block->angle_rad = cal_two_vectors_angle(pre_end_vector, cur_start_vector);
            break;
        default:
            break;
    }
}

static void block_cal_circle_vector(PLAN_BLOCK* current_block){     
    // 计算半径矢量
    VECTOR rs = VECTOR_Unit(&current_block->circular_data.center_pos, &current_block->start_pos);
    VECTOR re = VECTOR_Unit(&current_block->circular_data.center_pos, &current_block->end_pos);

    switch(current_block->circular_data.plane){
        case PLANE_XY:
            switch(current_block->type){
            case CIRCLE_CW:
                current_block->start_vec.x = rs.y;
                current_block->start_vec.y = - rs.x;
                current_block->start_vec.z = 0;
                current_block->end_vec.x = re.y;
                current_block->end_vec.y = - re.x;
                current_block->end_vec.z = 0;
                break;
            case CIRCLE_CCW:
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
                current_block->start_vec.x = rs.z;
                current_block->start_vec.y = 0;
                current_block->start_vec.z = - rs.x;
                current_block->end_vec.x = re.z;
                current_block->end_vec.y = 0;
                current_block->end_vec.z = - re.x;
                break;
            case CIRCLE_CCW:
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
                current_block->start_vec.x = 0;
                current_block->start_vec.y = rs.z;
                current_block->start_vec.z = - rs.y;
                current_block->end_vec.x = 0;
                current_block->end_vec.y = re.z;
                current_block->end_vec.z = - re.y;
                break;
            case CIRCLE_CCW:
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
    switch(current_block->circular_data.plane){
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
            if (IsGreaterThanOrEqual(angDelta, 0)) angDelta -= 2 * PI;
            break;
        case CIRCLE_CCW:
            if (IsLessThanOrEqual(angDelta, 0)) angDelta += 2 * PI;
            break;
    }
    current_block->circular_data.rad = fabs(angDelta);
    return current_block->circular_data.rad * current_block->circular_data.radius;
}

// 计算段长度|圆弧长
void block_cal_length(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
        case LINE:
        case RIGID_TAP:
        case TAP_RETURN:
            current_block->length = POS_Distance(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
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
            current_block->end_vec = current_block->start_vec = VECTOR_Unit(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            // 计算圆弧的切失，指向圆心
            block_cal_circle_vector(current_block);
            return;
        default:
            return;
    }
}
