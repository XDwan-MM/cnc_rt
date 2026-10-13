#include "plan_queue.h"
#include "global.h"
#include "nurbsfit.h"

static POS interpolate_line(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_rapid(PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_circle(POS current_pos, PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_helix(POS current_pos, PLAN_BLOCK* current_block, double accum_distance);
static POS interpolate_tap(PLAN_BLOCK* current_block, double accum_distance, double target_distance);

POS auto_cal_interp_pos(POS_CMD_DATA* pos_cmd_data, double accum_distance, double target_distance){
    if(accum_distance <= 0) return pos_cmd_data->current_pos;
    PLAN_BLOCK* current_block = cycle_tp->plan_block;
    if(current_block == NULL) return pos_cmd_data->current_pos;
    POS interp_pos;
    switch(current_block->type){
        case RAPID:
            interp_pos = interpolate_rapid(current_block, accum_distance);
            break;
        case LINE: 
        case PROBE:
            interp_pos = interpolate_line(current_block, accum_distance);
            break;
        case CIRCLE_CCW:
        case CIRCLE_CW:
            interp_pos = interpolate_circle(pos_cmd_data->current_pos, current_block, accum_distance);
            break;
        case HELIX_CW:
        case HELIX_CCW:
            interp_pos = interpolate_helix(pos_cmd_data->current_pos, current_block, accum_distance);
            break;
        case NURBS:
            interp_pos = interpolate_nurbs(target_distance, &pos_cmd_data->u_current, &pos_cmd_data->u_next);
            break;
        case RIGID_TAP:
        case TAP_RETURN:
            interp_pos = interpolate_tap(current_block, accum_distance, target_distance);
            break;
        case UNKNOWN:
		
        default:
            interp_pos = pos_cmd_data->current_pos;
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
    double angle = accum_distance / current_block->circular_data.radius;
    double angle_interp = 0;
    switch(current_block->circular_data.dir){
        case CLOCK:
            angle_interp = current_block->circular_data.statr_rad - angle;
            break;
        case COUNTERCLOCK:
            angle_interp = current_block->circular_data.statr_rad + angle;
            break;
        default:
            break;
    }
    POS interp_pos;
    switch(current_block->plane){
        case PLANE_XY:
            interp_pos.x = current_block->circular_data.center_pos.x + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.y = current_block->circular_data.center_pos.y + current_block->circular_data.radius * sin(angle_interp);
            interp_pos.z = current_pos.z;
            break;
        case PLANE_XZ:
            interp_pos.x = current_block->circular_data.center_pos.x + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.y = current_pos.y;
            interp_pos.z = current_block->circular_data.center_pos.z + current_block->circular_data.radius * sin(angle_interp);
            break;
        case PLANE_YZ:
            interp_pos.x = current_pos.x;
            interp_pos.y = current_block->circular_data.center_pos.y + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.z = current_block->circular_data.center_pos.z + current_block->circular_data.radius * sin(angle_interp);
            break;
        default:
            interp_pos = current_pos;
            break;
    }
    return interp_pos;
}

static POS interpolate_helix(POS current_pos, PLAN_BLOCK* current_block, double accum_distance) {
    double circle_distance = accum_distance * cos(current_block->helical_data.screw_rad);
    double screw_distance = accum_distance * sin(current_block->helical_data.screw_rad);
    double angle = circle_distance / current_block->circular_data.radius;
    double angle_interp = 0;
    switch(current_block->circular_data.dir){
        case CLOCK:
            angle_interp = current_block->circular_data.statr_rad - angle;
            break;
        case COUNTERCLOCK:
            angle_interp = current_block->circular_data.statr_rad + angle;
            break;
        default:
            break;
    }
    POS interp_pos;
    switch(current_block->plane){
        case PLANE_XY:
            interp_pos.x = current_block->circular_data.center_pos.x + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.y = current_block->circular_data.center_pos.y + current_block->circular_data.radius * sin(angle_interp);
            interp_pos.z = current_block->start_pos.z + screw_distance;
            break;
        case PLANE_XZ:
            interp_pos.x = current_block->circular_data.center_pos.x + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.y = current_block->start_pos.y + screw_distance;
            interp_pos.z = current_block->circular_data.center_pos.z + current_block->circular_data.radius * sin(angle_interp);
            break;
        case PLANE_YZ:
            interp_pos.x = current_block->start_pos.x + screw_distance;
            interp_pos.y = current_block->circular_data.center_pos.y + current_block->circular_data.radius * cos(angle_interp);
            interp_pos.z = current_block->circular_data.center_pos.z + current_block->circular_data.radius * sin(angle_interp);
            break;
        default:
            interp_pos = current_pos;
            break;
    }
    return interp_pos;
}

/* 刚性攻丝同步（进给 = 转速 × 螺距：F = S × Pitch）
主轴位置 Z = Pitch × (θ/360°)
G0 X0 Y0 Z0;
M29 S100;
G84 Z-10 R5 F100;
*/
static POS interpolate_tap(PLAN_BLOCK* current_block, double accum_distance, double target_distance){
    // 进给值插补位置
    POS interp_pos = current_block->start_pos;
    switch(current_block->plane){
        case PLANE_XY:
            interp_pos.z = current_block->start_pos.z + current_block->start_vec.z * accum_distance;
            break;
        case PLANE_XZ:
            interp_pos.y = current_block->start_pos.y + current_block->start_vec.y * accum_distance;
            break;
        case PLANE_YZ:
            interp_pos.x = current_block->start_pos.x + current_block->start_vec.x * accum_distance;
            break;
    }
    // 主轴旋转角度
    double spl_pos = fmod(target_distance / current_block->rigid_data.screw_offset, current_block->rigid_data.screw_offset);
    double spl_angle = spl_pos * 360;
    if(current_block->rigid_data.reverse) spl_angle = - spl_angle;
    // 测试
    double sp_wcs_cmd_pos_cur = fmod(rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur, 360);
    rt_data->axis_data[AXIS_SP].wcs_cmd_pos_cur += spl_angle;
    // printf("z:%f, sp:%f, 螺距：%f, 同步：%f\n", spl_pos, sp_wcs_cmd_pos_cur, current_block->rigid_data.screw_offset, target_distance / spl_pos);
    return interp_pos;
}