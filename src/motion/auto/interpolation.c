#include "plan_queue.h"
#include "global.h"

static POS interpolate_line(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_circle(POS current_pos, PLAN_BLOCK* current_block, double accum_distance);

POS mem_cal_pos(POS current_pos, double accum_distance){
    if(accum_distance <= 0) return current_pos;
    PLAN_BLOCK* current_block = cycle_tp->plan_block;
    if(current_block == NULL) return current_pos;
    POS interp_pos;
    switch(current_block->type){
        case RAPID:
            interp_pos = interpolate_rapid(current_block, accum_distance);
            break;
        case LINE:
            interp_pos = interpolate_line(current_block, accum_distance);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            interp_pos = interpolate_circle(current_pos, current_block, accum_distance);
            break;
        // 刚性攻丝
        case RIGID_TAP:
        case UNKNOWN:
        default:
            interp_pos = current_pos;
            break;
    }
    return interp_pos;
}

// 计算直线段插补点
static POS interpolate_line(PLAN_BLOCK* current_block, double accum_distance) {
    POS interp_pos;
    interp_pos.x = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    interp_pos.y = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    interp_pos.z = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
    return interp_pos;
}

// 计算快速段插补点
static POS interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance) {
    POS interp_pos;
    interp_pos.x = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
    interp_pos.y = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
    interp_pos.z = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
    return interp_pos;
}

// 计算圆弧段插补点
static POS interpolate_circle(POS current_pos, PLAN_BLOCK* current_block, double accum_distance) {
    double angle = accum_distance / current_block->circular_block.radius;
    double angle_interp = 0;
    switch(current_block->circular_block.dir){
        case CLOCK:
            angle_interp = current_block->circular_block.statr_rad - angle;
            break;
        case COUNTERCLOCK:
            angle_interp = current_block->circular_block.statr_rad + angle;
            break;
        default:
            break;
    }
    POS interp_pos;
    switch(current_block->circular_block.plane){
        case PLANE_XY:
            interp_pos.x = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.y = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            interp_pos.z = current_pos.z;
            break;
        case PLANE_XZ:
            interp_pos.x = current_block->circular_block.center_pos.x + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.y = current_pos.y;
            interp_pos.z = current_block->circular_block.center_pos.y + current_block->circular_block.radius * sin(angle_interp);
            break;
        case PLANE_YZ:
            interp_pos.x = current_pos.z;
            interp_pos.y = current_block->circular_block.center_pos.y + current_block->circular_block.radius * cos(angle_interp);
            interp_pos.z = current_block->circular_block.center_pos.z + current_block->circular_block.radius * sin(angle_interp);
            break;
        default:
            interp_pos = current_pos;
            break;
    }
    return interp_pos;
}