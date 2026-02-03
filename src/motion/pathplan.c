#include <stdlib.h>
#include <math.h>
#include "trajectory.h"
#include "pathplan.h"
#include "gcode_queue.h"
#include "global.h"
#include "cnc_math.h"
#include "log.h"

static void block_cal_pos(GCODE_BLOCK* gcode_block, PLAN_BLOCK* current_block, POS* start_pos);
static void block_cal_vel(PLAN_BLOCK* current_block);
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector);
static void block_cal_circle_vector(PLAN_BLOCK* current_block);
static double block_cal_circle_length(PLAN_BLOCK* current_block);
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel);

// 将译码数据转换成段
GNC_RES chan_add_block(){
    PLAN_BLOCK* pretreat_block;
    GCODE_BLOCK* gcode_block;

    if(gcode_queue_is_empty(gcode_queue)) return GNC_DATA_EMPTY;

    // 译码第一段类型为g00，起点为当前机床位置
    if(sys->is_first_run){
        pretreat_queue->pre_pos = (POS){.x = var_axis[AXIS_X].cmd_pos, .y = var_axis[AXIS_Y].cmd_pos, .z = var_axis[AXIS_Z].cmd_pos};
        sys->is_first_run = false;
    }

    // 每次最多处理 PRE_TREAT_SIZE
    for(int i = 0; i < PRE_TREAT_SIZE; i++){
        // 如果预处理队列已满，提前结束
        if(queue_is_full(pretreat_queue)) return GNC_DATA_FULL;
        // 如果译码器无数据，提前结束
        if(gcode_queue_is_empty(gcode_queue)) return GNC_DATA_EMPTY;

        gcode_block = gcode_queue_peak(gcode_queue);
        if(gcode_block == NULL) return GNC_DATA_NULL;
        pretreat_block = queue_peak_tail(pretreat_queue);
        if(pretreat_block == NULL) return GNC_DATA_NULL;

        // 拷贝信息
        pretreat_block->mode = gcode_block->mode;
        pretreat_block->line_number = gcode_block->line_number;

        // 构造成段
        pretreat_block->type = gcode_block->type;
        pretreat_block->feed_vel = gcode_block->feed_vel;
        block_cal_pos(gcode_block, pretreat_block, &pretreat_queue->pre_pos);
        
        // 计算速度
        block_cal_vel(pretreat_block);

        // 计算切矢
        block_cal_vector(pretreat_block);

        // 计算长度
        block_cal_length(pretreat_block);

        // 计算两段衔接夹角
        block_cal_angle(pretreat_block, &pretreat_queue->pre_vec, &pretreat_block->start_vec);

        // 删除已处理的gcode
        gcode_queue_pop(gcode_queue);

        // 更新预处理队列信息
        queue_soft_push(pretreat_queue);
        pretreat_queue->pre_pos = pretreat_block->end_pos;
        pretreat_queue->pre_vec = pretreat_block->end_vec;
    }
    return GNC_CAL_SUCCESS;
}

GNC_RES chan_look_ahead(double vs){
    vs = fabs(vs);
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* cur_block;
    PLAN_BLOCK* pre_block;

    // 拷贝 pretreat_queue 的数据
    for(int i = 0; i < LOOK_AHEAD_SIZE; i++){
        // 如果前瞻队列已满，提前结束
        if(queue_is_full(plan_queue)) break;
        // 如果预处理队列无数据，提前结束
        if(queue_is_empty(pretreat_queue)) break;
        pretreat_block = queue_peak(pretreat_queue);
        if(pretreat_block == NULL) return GNC_DATA_NULL;
        // 注意：如果长度为0，删除该段
        if(IsEqualTo(pretreat_block->length, 0)){
            queue_pop(pretreat_queue);
            continue;
        }
        queue_push(plan_queue, pretreat_block);
        queue_pop(pretreat_queue);
    }

    if(queue_is_empty(plan_queue)) return GNC_DATA_EMPTY;

    /******************************************
    1. 初始化首末速度，从末端往前计算，每次前瞻的末段末速度为0，plan_queue->next_vel = 0
    2. 只有两段以上才进行遍历 i > 0
    3. 当为G01时，首段初速度会修正为前一段的末速度；其他类型的初速度为0 */
    plan_queue->next_vel = 0;
    // 两段及以上，初始化首末速度
    for(int i = plan_queue->size - 1; i > 0; i--){
        cur_block = queue_get_block(plan_queue, i);
        if(cur_block == NULL) return GNC_DATA_NULL;
        pre_block = queue_get_block(plan_queue, i - 1);
        if(pre_block == NULL) return GNC_DATA_NULL;
        block_cal_lookahead_vel(cur_block, pre_block->type, plan_queue->next_vel);
        plan_queue->next_vel = cur_block->start_vel;
    }
    // 修正首段速度
    cur_block = queue_peak(plan_queue);
    block_cal_lookahead_vel(cur_block, plan_queue->pre_head_type, plan_queue->next_vel);
    cur_block->start_vel = vs;
    // 判断是否大于最大速度
    if(cur_block->start_vel > cur_block->max_vel) cur_block->max_vel = cur_block->start_vel;
    /*******************************************/
    vp_cal_bid_lookahead(par_chan[0].vplan_mode, plan_queue, par_ncu.cycle);
    return GNC_CAL_SUCCESS;
}


// 计算段的首末速度、最大速度、加速度、加加速度
static void block_cal_lookahead_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel){
    switch(current_block->type){
        case RAPID:
            current_block->max_vel = current_block->feed_vel * var_chan[0].rapid_override;
            current_block->start_vel = 0;
            current_block->end_vel = 0;
            break;

        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            current_block->max_vel = current_block->feed_vel * var_chan[0].feed_override;
            // 预留：如果开启单段，首末速度为0
            current_block->end_vel = next_start_vel;
            if(sys->is_lookahead && (pre_type == LINE || pre_type == CIRCLE_CCW || pre_type == CIRCLE_CW)){
                // block1 <-- block2 <-- block3
                //  ------   --------   ------
                // |?--100| |100--100| |100--0|
                //  ------   --------   ------
                // 如果开启前瞻，段首末速度初始化为段内最大允许速度
                current_block->start_vel = current_block->max_vel;  
            }else{
                current_block->start_vel = 0;
            }
            break;

        default:
            break;
    }
}

// 计算段的首末位置
static void block_cal_pos(GCODE_BLOCK* gcode_block, PLAN_BLOCK* current_block, POS* start_pos){
    POS_COPY(&current_block->start_pos, start_pos);
    POS_COPY(&current_block->end_pos, &gcode_block->pos);
    switch(gcode_block->type){
        case CIRCLE_CCW:
        case CIRCLE_CW:
            // 计算圆心和半径
            POS_COPY(&current_block->circular_block.center_pos, &gcode_block->center_pos);
            current_block->circular_block.radius = POS_Distance(&current_block->start_pos, &current_block->circular_block.center_pos);
            current_block->circular_block.plane = gcode_block->plane;
            current_block->circular_block.dir = (gcode_block->type == CIRCLE_CW) ? CLOCK : COUNTERCLOCK;
            break;
        default:
            break;
    }
}

// 计算段最大允许速度
static void block_cal_vel(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
            current_block->feed_vel = MIN3(par_axis[0].rapid_vel, par_axis[1].rapid_vel, par_axis[2].rapid_vel);  // G00最大速度为设置的快速进给速度
            current_block->max_vel = current_block->feed_vel;
            current_block->max_acc = MIN3(par_axis[0].rapid_acc, par_axis[1].rapid_acc, par_axis[2].rapid_acc);
            current_block->max_jerk = MIN3(par_axis[0].rapid_jerk, par_axis[1].rapid_jerk, par_axis[2].rapid_jerk);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            double min_feed = MIN3(par_axis[0].feed_vel, par_axis[1].feed_vel, par_axis[2].feed_vel);
            current_block->feed_vel = MIN(current_block->feed_vel, min_feed);   // G01|G02|G03最大速度
            current_block->max_vel = current_block->feed_vel;                   // 倍率在前瞻时计算
            current_block->max_acc = MIN3(par_axis[0].feed_acc, par_axis[1].feed_acc, par_axis[2].feed_acc);
            current_block->max_jerk = MIN3(par_axis[0].feed_jerk, par_axis[1].feed_jerk, par_axis[2].feed_jerk);
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
    return acos(POS_Dot(*vector1, *vector2) / (norm1 * norm2));
}

// 根据切失计算两段之间的衔接角度
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector){
    switch(current_block->type){
        case RAPID:
        case CIRCLE_CCW:
        case CIRCLE_CW:
        case LINE:
            current_block->angle_rad = cal_two_vectors_angle(pre_end_vector, cur_start_vector);
            break;
        default:
            break;
    }
}

static void block_cal_circle_vector(PLAN_BLOCK* current_block){     
    // 计算半径矢量
    VECTOR rs = VECTOR_Unit(&current_block->circular_block.center_pos, &current_block->start_pos);
    VECTOR re = VECTOR_Unit(&current_block->circular_block.center_pos, &current_block->end_pos);

    switch(current_block->circular_block.plane){
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
    VECTOR rs = POS_VECTOR(&current_block->circular_block.center_pos, &current_block->start_pos);
    VECTOR re = POS_VECTOR(&current_block->circular_block.center_pos, &current_block->end_pos);

    // 计算切失
    switch(current_block->circular_block.plane){
        case PLANE_XY:
            current_block->circular_block.statr_rad = atan2(rs.y, rs.x);
            current_block->circular_block.end_rad = atan2(re.y, re.x);
            break;
        case PLANE_XZ:
            current_block->circular_block.statr_rad = atan2(rs.z, rs.x);
            current_block->circular_block.end_rad = atan2(re.z, re.x);
            break;
        case PLANE_YZ:
            current_block->circular_block.statr_rad = atan2(rs.z, rs.y);
            current_block->circular_block.end_rad = atan2(re.z, re.y);
            break;
    }

    // 计算弧度
    double angDelta = current_block->circular_block.end_rad - current_block->circular_block.statr_rad;
    switch(current_block->type){
        case CIRCLE_CW:
            if (angDelta > 0) angDelta -= 2 * PI;
            break;
        case CIRCLE_CCW:
            if (angDelta < 0) angDelta += 2 * PI;
            break;
    }
    current_block->circular_block.rad = fabs(angDelta);
    return current_block->circular_block.rad * current_block->circular_block.radius;
}

// 计算段长度|圆弧长
void block_cal_length(PLAN_BLOCK* current_block){
    // 注意：如果段长度为0，该段无效
    switch(current_block->type){
        case RAPID:
        case LINE:
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

//static double cal_three_points_angle(POS* pre_pos, POS* cur_pos, POS* next_pos){
//    POS vector1 = POS_VECTOR(pre_pos, cur_pos);
//    POS vector2 = POS_VECTOR(cur_pos, next_pos);
//    double norm1 = POS_Norm(vector1);
//    double norm2 = POS_Norm(vector2);
//    // 注意：如果norm为0，该段长度为0
//    if(IsEqualTo(norm1, 0) || IsEqualTo(norm2, 0)) return 0;
//    return acos(POS_Dot(vector1, vector2) / (norm1 * norm2));
//}
