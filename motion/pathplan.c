#include "pathplan.h"
#include <math.h>

// 将译码数据转换成段
GNC_RES chan_add_block(){
    PLAN_BLOCK* pretreat_block;
    GCODE_BLOCK* gcode_block;

    // todo：确认译码第一段类型是否为G00
    if(pretreat_queue->isFirstRUN){
        // 如果为程序第一段gcode，构造一个G00段，起始点为当前位置，终点为gcode位置
        pretreat_queue->pre_pos = hal->current_pos;      
        pretreat_queue->isFirstRUN = false;
    }

    // 每次最多处理 PRE_TREAT_SIZE
    for(int i = 0; i < PRE_TREAT_SIZE; i++){
        // 如果预处理队列已满，提前结束
        if(queue_is_full(pretreat_queue)) return GNC_DATA_FULL;
        // 如果译码器无数据，提前结束
        if(gcode_queue_is_empty(gcode_queue)) return GNC_DATA_EMPTY;
        gcode_block = gcode_queue_peak(gcode_queue);
        pretreat_block = queue_peak_tail(pretreat_queue);

        // 构造成段
        pretreat_block->type = gcode_block->type;
        pretreat_block->feed_vel = gcode_block->feed_vel;
        block_cal_pos(pretreat_block->type, pretreat_block, &pretreat_queue->pre_pos, &gcode_block->pos);

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
    PLAN_BLOCK* next_block;

    // 拷贝 pretreat_queue 的数据
    for(int i = 0; i < LOOK_AHEAD_SIZE; i++){
        // 如果前瞻队列已满，提前结束
        if(queue_is_full(plan_queue)) break;
        // 如果预处理队列无数据，提前结束
        if(queue_is_empty(pretreat_queue)) break;
        pretreat_block = queue_pop(pretreat_queue);
        queue_push(plan_queue, pretreat_block);
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
        pre_block = queue_get_block(plan_queue, i - 1);
        block_cal_vel(cur_block, pre_block->type, plan_queue->next_vel);
        plan_queue->next_vel = cur_block->start_vel;
    }
    // 修正首段速度
    cur_block = queue_peak(plan_queue);
    block_cal_vel(cur_block, plan_queue->pre_head_type, plan_queue->next_vel);
    cur_block->start_vel = vs;
    /*******************************************/
    cal_look_ahead(params->velplan_type, plan_queue);

    return GNC_CAL_SUCCESS;
}


// 计算段的首末速度、最大速度、加速度、加加速度
void block_cal_vel(PLAN_BLOCK* current_block, INTERP_TYPE pre_type, double next_start_vel){
    switch(current_block->type){
        case RAPID:
        case CIRCLE_CCW:
        case CIRCLE_CW: // 预留：直线和圆弧也可以速度衔接
            current_block->start_vel = 0;
            current_block->end_vel = 0;
            break;

        case LINE:
            // 预留：如果开启单段，首末速度为0
            current_block->end_vel = next_start_vel;
            if(params->is_lookahead && current_block->type == pre_type){
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
void block_cal_pos(INTERP_TYPE type, PLAN_BLOCK* current_block, POS* start_pos, POS* end_pos){
    memcpy(&current_block->start_pos, start_pos, sizeof(POS));
    memcpy(&current_block->end_pos, end_pos, sizeof(POS));
    switch(type){
         case RAPID:
            current_block->max_vel = params->rapid_vel;        // G00最大速度为设置的快速进给速度
            current_block->max_acc = params->rapid_acc;
            current_block->max_jerk = params->rapid_jerk;
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            current_block->max_vel = current_block->feed_vel;  // G02|G03最大速度为F进给速度
            current_block->max_acc = params->feed_acc;
            current_block->max_jerk = params->feed_jerk;
            // todo：计算圆心和半径
            break;
        case LINE:
            current_block->max_vel = current_block->feed_vel;  // G01最大速度为F进给速度
            current_block->max_acc = params->feed_acc;
            current_block->max_jerk = params->feed_jerk;
            break;
        default:
            break;
    }
}

static double cal_three_points_angle(POS* pre_pos, POS* cur_pos, POS* next_pos){
    POS vector1 = POS_VECTOR(pre_pos, cur_pos);
    POS vector2 = POS_VECTOR(cur_pos, next_pos);
    double norm1 = POS_Norm(vector1);
    double norm2 = POS_Norm(vector2);
    // 注意：如果norm为0，该段长度为0
    if(IsEqualTo(norm1, 0) || IsEqualTo(norm2, 0)) return 0;
    return acos(POS_Dot(vector1, vector2) / (norm1 * norm2));
} 

static double cal_two_vectors_angle(VECTOR* vector1, VECTOR* vector2){
    double norm1 = POS_Norm(*vector1);
    double norm2 = POS_Norm(*vector2);
    // 注意：如果norm为0，该段长度为0
    if(IsEqualTo(norm1, 0) || IsEqualTo(norm2, 0)) return 0;
    return acos(POS_Dot(*vector1, *vector2) / (norm1 * norm2));
}

// 根据切失计算两段之间的衔接角度
void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector, VECTOR* cur_start_vector){
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

// 计算段长度| 圆弧长
void block_cal_length(PLAN_BLOCK* current_block){
    // 注意：如果段长度为0，该段无效
    switch(current_block->type){
        case RAPID:
        case LINE:
            current_block->length = POS_Distance(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            {   
                // todo：已知圆心和首末端点，计算弧长
                double rad = cal_three_points_angle(&current_block->start_pos, &current_block->circular_block.center_pos, &current_block->end_pos);
                current_block->length = rad * PI * current_block->circular_block.radius;
                break;
            }
        default:
            // 更新：【报警模块】翟亚飞
            break;
    }
}

// 计算段首末切失
void block_cal_vector(PLAN_BLOCK* current_block){
    switch(current_block->type){
        case RAPID:
        case LINE:
            current_block->end_vec = current_block->start_vec = POS_VECTOR(&current_block->start_pos, &current_block->end_pos);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            // todo：计算圆弧的切失
            return;
        default:
            return;
    }
}


// 调试：demo测试用，实际逻辑需修改
GNC_RES dcd_to_block(GCODE_QUEUE *gcode_queue, int size, PLAN_QUEUE* pl_queue, PARAMS* param){

    PLAN_BLOCK* tc = (PLAN_BLOCK*)malloc(sizeof(PLAN_BLOCK));

    // 先判断插补队列是否为空，如果为空，解释队列获取两个点，如果解释队列小于两个点，return
    // 如果插补队列不为空，拷贝插补队列最后一个点，解释队列获取一个点，如果解释队列小于一个点，return

    for(int i = 0; i < size - 1; i++){
        if(queue_is_full(pl_queue)) return GNC_OPERATOR_ERROR;

        GCODE_BLOCK* cur_dcd = gcode_queue_get_block(gcode_queue, i);
        GCODE_BLOCK* next_dcd = gcode_queue_get_block(gcode_queue, i + 1);

        tc->type = cur_dcd->type;
        tc->start_pos = cur_dcd->pos;
        tc->end_pos = next_dcd->pos;
        block_cal_length(tc);
        block_cal_vector(tc);
        
        // 根据是否前瞻，设置起始速度
        tc->start_vel = param->is_lookahead ? param->feed_vel : 0;
        tc->end_vel = param->is_lookahead ? param->feed_vel : 0;
        tc->max_vel = param->feed_vel;

        queue_push(pl_queue, tc);
    }

    PLAN_BLOCK* head_tc = queue_get_block(pl_queue, 0);
    PLAN_BLOCK* tail_tc = queue_get_block(pl_queue, pl_queue->size - 1);
    head_tc->start_vel = 0;
    tail_tc->end_vel = 0;

    // 计算block之间的夹角, todo: 未考虑圆弧
    for(int i = 0; i < pl_queue->size - 1; i++){
        PLAN_BLOCK* cur = queue_get_block(pl_queue, i);
        PLAN_BLOCK* next = queue_get_block(pl_queue, i + 1);
        if (cur && next) {
            next->angle_rad = cal_three_points_angle(&cur->start_pos, &cur->end_pos, &next->end_pos);
        }
    }

    free(tc);
    tc = NULL;
    return GNC_CAL_SUCCESS;
}

// 调试：demo测试用，实际逻辑需修改
/// @brief 速度前瞻
/// @param pl_queue 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/15
GNC_RES dcd_look_ahead(PLAN_QUEUE* pl_queue, PARAMS* params, VELPLAN_TYPE type){
    // if(queue_is_empty(pl_queue)) return GNC_DATA_EMPTY;
    // if(queue_is_full(pl_queue)) return GNC_DATA_FULL;

    double vmax = params->feed_vel;
    double amax = params->feed_acc;
    double jmax = params->feed_jerk;
    double cycle_t = params->cycle_time;

    cal_look_ahead(params->velplan_type ,pl_queue);

    return GNC_CAL_SUCCESS;
}
