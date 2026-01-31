#include "trajectory.h"
#include <math.h>

static double cal_scurve_cri_length(double vs, double ve, double amax, double jmax);
static double cal_sin_cri_length(double vs, double ve, double amax, double jmax);
static double cal_scurve_decel_ve(double vs, double amax, double jmax, double decline_s);
static double cal_sin_decel_ve(double vs, double amax, double jmax, double decline_s);
static double S_shape_lookahead(double jmax, double amax, double v, double length);
static double sin_shape_lookahead(double jmax, double amax, double v, double length);
/* 梯形加减速 */
static GNC_RES T_shape_cal_param(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* vel_profile);
static GNC_RES T_shape_cal_round_param(double cycle_time, PLAN_BLOCK* pl_block, R_VEL_PROFILE* rProfile);
static void T_shape_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data);
/* S型加减速 */
static GNC_RES scurve_cal_param1(double cycle_time, PLAN_BLOCK* pl_block, double tol, VEL_PROFILE* vel_profile);
static GNC_RES scurve_cal_param2(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* vel_profile);
static void scurve_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data);
/* sin型加减速 */
static GNC_RES sin_cal_param1(double cycle_time, PLAN_BLOCK* pl_block, double tol, VEL_PROFILE* vel_profile);
static GNC_RES sin_cal_param2(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* vel_profile);
static void sin_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data);


GNC_RES check_block_params_valid(PLAN_BLOCK* plan_block){
    if(plan_block == NULL) return GNC_DATA_NULL;
    if(plan_block->length <= 0) return GNC_PARAM_UNVALID;
    if(plan_block->start_vel < 0) return GNC_PARAM_UNVALID;
    if(plan_block->end_vel < 0) return GNC_PARAM_UNVALID;
    if(plan_block->feed_vel <= 0) return GNC_PARAM_UNVALID;
    if(plan_block->max_vel<= 0) return GNC_PARAM_UNVALID;
    return GNC_CAL_SUCCESS;
}

GNC_RES check_vel_params_valid(VEL_PROFILE* vel_profile){
    if(vel_profile == NULL) return GNC_DATA_NULL;
    if(vel_profile->vs < 0) return GNC_PARAM_UNVALID;
    if(vel_profile->ve < 0) return GNC_PARAM_UNVALID;
    if(vel_profile->vpeak < vel_profile->vs || vel_profile->vpeak < vel_profile->ve) return GNC_PARAM_UNVALID;
    return GNC_CAL_SUCCESS;
}

double vp_cal_decel_dist(VELPLAN_TYPE velplan_type, double vs, double ve, double amax, double jmax){
    switch (velplan_type)
    {
        case T_SHAPE:
            return 0.5 * vs * vs / amax;
        case S_SHAPE:
            return cal_scurve_cri_length(vs, 0, amax, jmax);
        case SIN_SHAPE:
            return cal_sin_cri_length(vs, 0, amax, jmax);
        default:
            // ERROR
            return -1;
    }
}

void vp_cal_traj_data(VELPLAN_TYPE velplan_type, VEL_PROFILE* vel_profile, int interp_cycle, TRAJ_PARAMS* traj_data, double cycle_time){
    switch (velplan_type)
    {
        case T_SHAPE:
            T_shape_cal_traj(cycle_time, vel_profile, interp_cycle, traj_data);
            return;
        case S_SHAPE:
            scurve_cal_traj(cycle_time, vel_profile, interp_cycle, traj_data);
            return;
        case SIN_SHAPE:
            sin_cal_traj(cycle_time, vel_profile, interp_cycle, traj_data);
            return;
        default:
            return; // ERROR
    }
}

double vp_cal_decel_vel(VELPLAN_TYPE velplan_type, double vs, double amax, double jmax, double decline_s){
    switch (velplan_type)
    {
        case T_SHAPE:
            return sqrt(vs * vs - 2  * amax * decline_s);
        case S_SHAPE:
            return cal_scurve_decel_ve(vs, amax, jmax, decline_s);
        case SIN_SHAPE:
            return cal_sin_decel_ve(vs, amax, jmax, decline_s);
        default:
            // ERROR
            return -1;
    }
}

GNC_RES vp_cal_vel_profile(VELPLAN_TYPE velplan_type, PLAN_BLOCK* plan_block, VEL_PROFILE* vel_profile, double cycle_time){
    // 对输入参数合法性进行判断
    GNC_RES res = check_block_params_valid(plan_block);
    if(res != GNC_CAL_SUCCESS) return res;

    switch (velplan_type)
    {
        case T_SHAPE:
            res = T_shape_cal_param(cycle_time, plan_block, vel_profile);
            break;
        case S_SHAPE:
            // return scurve_cal_param1(cycle_time, plan_block,1E-6);
            res = scurve_cal_param2(cycle_time, plan_block, vel_profile);
            break;
        case SIN_SHAPE:
            res = sin_cal_param2(cycle_time, plan_block, vel_profile);
            break;
        default:
            res = GNC_CAL_ERROR;
            break;
    }

    // 判断 vel_profile 是否计算成功
    if(res == GNC_CAL_ERROR) return res;

    // 判断 vel_profile 是否有效
    res = check_vel_params_valid(vel_profile);

    return res;
}

static double cal_look_ahead_vel(VELPLAN_TYPE velplan_type, double vel, double acc, double jerk, double length){
    switch (velplan_type)
    {
        case T_SHAPE:
            return sqrt(vel * vel + 2 * acc * length);
        case S_SHAPE:
            return S_shape_lookahead(jerk, acc, vel, length);
        case SIN_SHAPE:
            return sin_shape_lookahead(jerk, acc, vel, length);
        default:
            return 0;
    }
}

void vp_cal_bid_lookahead(VELPLAN_TYPE velplan_type, PLAN_QUEUE* plan_queue, double cycle_time){
    PLAN_BLOCK* cur_block;
    PLAN_BLOCK* pre_block;
    PLAN_BLOCK* next_block;

    // 反向遍历 backfoward scanning
    for(int i = plan_queue->size - 1; i > 0; i--){
        cur_block = queue_get_block(plan_queue, i);
        pre_block = queue_get_block(plan_queue, i - 1);

        double v[4];
        v[0] = cal_look_ahead_vel(velplan_type, cur_block->end_vel, cur_block->max_acc, cur_block->max_jerk, cur_block->length);
        v[1] = 0.5 * cur_block->max_acc * cycle_time / sin(PI - cur_block->angle_rad / 2);   
        v[2] = cur_block->length / cycle_time; // 单插补周期内走完的速度
        v[3] = cur_block->max_vel;
        // 预留：圆弧弓高误差delta下的约束速度
        //v[4] = 2 * sqrt(2 * delta * r - pow(delta, 2)) / cycle_t; 

        double vmin = min_double_array(v, 4);
        if(vmin < cur_block->start_vel) {
            if(IsEqualTo(vmin, v[2])) cur_block->max_vel = vmin;
            cur_block->start_vel = vmin;  
            pre_block->end_vel = cur_block->start_vel;
        }
    }

    // 正向遍历 forward scanning
    for(int i = 0; i < plan_queue->size - 1; i++){
        cur_block = queue_get_block(plan_queue, i);
        next_block = queue_get_block(plan_queue, i + 1);

        double v[2];
        v[0] = cal_look_ahead_vel(velplan_type, cur_block->start_vel, cur_block->max_acc, cur_block->max_jerk, cur_block->length);
        v[1] = cur_block->length / cycle_time; 

        double vmin = min_double_array(v, 2);
        if(vmin < cur_block->end_vel){
            cur_block->end_vel = vmin; 
            next_block->start_vel = cur_block->end_vel;
        }
    }
}

static double S_shape_lookahead(double jmax, double amax, double v, double length){
    double a = 1; 
    double b = v;
    double c = - v * v;
    double d = - pow(v,3) - jmax * length * length;

    double roots[3];
    int rootCount = solve_cubic_equation(a, b, c, d, roots);
    double vm = get_min_nonnegative_real_root(rootCount, roots);

    if(vm > (pow(amax,2) / jmax + v)){
        double a = jmax; 
        double b = amax * amax;
        double c = - jmax * v * v + amax * amax * v - 2 * amax * jmax * length;

        double roots[2];
        int rootCount = solve_quadratic_equation(a, b, c, roots);
        vm = get_min_nonnegative_real_root(rootCount, roots);
    }

    return vm;
}

static double sin_shape_lookahead(double jmax, double amax, double v, double length){
    double a = 1; 
    double b = v;
    double c = - v * v;
    double d = - pow(v,3) - 2 * jmax * length * length / PI;

    double roots[3];
    int rootCount = solve_cubic_equation(a, b, c, d, roots);
    double vm = get_min_nonnegative_real_root(rootCount, roots);

    if(vm > (pow(amax,2) / jmax + v)){
        double a = 2 * jmax; 
        double b = amax * amax * PI;
        double c = - 2 * jmax * v * v + amax * amax * PI * v - 4 * amax * jmax * length;

        double roots[2];
        int rootCount = solve_quadratic_equation(a, b, c, roots);
        vm = get_min_nonnegative_real_root(rootCount, roots);
    }

    return vm;
}


static double cal_scurve_cri_length(double vs, double ve, double amax, double jmax) {
    double s = 0;
    if (IsLessThanOrEqual(fabs(ve - vs), (amax * amax / jmax))) {
        // 未达到amax
        s = (vs + ve) * sqrt(fabs(ve - vs) / jmax);
    } else {
        // 达到amax
        s = 0.5 * (vs + ve) * ((amax / jmax) + fabs(ve - vs) / amax);
    }
    return s;
}

static double cal_sin_cri_length(double vs, double ve, double amax, double jmax) {
    double s = 0;
    if (IsLessThanOrEqual(fabs(ve - vs), (0.5 * amax * amax * PI / jmax))) {
        // 未达到amax
        s = (vs + ve) * sqrt(0.5 * fabs(ve - vs) * PI / jmax);
    } else {
        // 达到amax
        s = 0.5 * (vs + ve) * ((0.5 * amax * PI / jmax) + fabs(ve - vs) / amax);
    }
    return s;
}

static double cal_scurve_decel_ve(double vs, double amax, double jmax, double decline_s) {
    double ve = 0;
    if (IsLessThanOrEqual(vs, (amax * amax / jmax))) {
        // 未达到amax
        ve = decline_s / sqrt(vs / jmax) - vs; 
    } else {
        // 达到amax
        ve = 2 * decline_s / ((amax / jmax) + vs / amax) - vs;
    }
    return ve;
}

static double cal_sin_decel_ve(double vs, double amax, double jmax, double decline_s) {
    double ve = 0;
    if (IsLessThanOrEqual(vs, (0.5 * amax * amax * PI / jmax))) {
        // 未达到amax
        ve = decline_s / sqrt(0.5 * vs * PI / jmax) - vs;
    } else {
        // 达到amax
        ve = 2 * decline_s / ((0.5 * amax * PI / jmax) + vs / amax) - vs;
    }
    return ve;
}

/// @brief 计算T型加减速曲线
/// @param params 机床参数
/// @param pl_block 插补段信息
/// @param profile 加减速曲线
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static GNC_RES T_shape_cal_param(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* profile){
    double vs = pl_block->start_vel;
    double ve = pl_block->end_vel;
    double vmax = pl_block->max_vel;
    double amax = pl_block->max_acc;  // 自动加工和回零的加速度和加加速度不同
    double jmax = pl_block->max_jerk;
    double s = pl_block->length;
    double sa, sd, sc = 0;

    double sscri = fabs(pow(vs,2) - pow(ve,2))/(2*amax);
    double slcri = (2*pow(vmax,2) - pow(vs,2) - pow(ve,2))/(2*amax);

    if (IsLessThan(s, sscri)) return GNC_CAL_ERROR;

    if (IsEqualTo(s, sscri)){
        if (vs > ve){ // 仅减速
            profile->vpeak =  vs;
            sa = 0;
            sd = s;
            sc = 0;
        }
        if (vs < ve){ // 仅加速
            profile->vpeak =  ve;
            sa = s;
            sd = 0;
            sc = 0;
        }
    }
    else
    {
        if (IsGreaterThan(s, slcri)){
            profile->vpeak =  vmax;
            sa = (pow(vmax,2) - pow(vs,2))/(2*amax);
            sd = (pow(vmax,2) - pow(ve,2))/(2*amax);
            sc = s - sa -sd;
        }
        else{
            profile->vpeak =  sqrt((2*amax*s + pow(vs,2) + pow(ve,2))/2);
            sa = (pow(profile->vpeak,2) - pow(vs,2))/(2*amax);
            sd = (pow(profile->vpeak,2) - pow(ve,2))/(2*amax);
            sc = 0;
        }
    }

    profile->t[0] = (profile->vpeak - vs)/amax;
    profile->t[1] = profile->t[0] + sc/profile->vpeak;
    profile->t[2] = profile->t[1] + (profile->vpeak - ve)/amax;

    profile->v[0] = profile->vpeak;
    profile->v[1] = profile->vpeak;
    profile->v[2] = ve;

    profile->s[0] = sa;
    profile->s[1] = profile->s[0] + sc;
    profile->s[2] = profile->s[1] + sd;

    profile->t_cycle = ceil(profile->t[2] / cycle_time);
    profile->vs = vs;
    profile->ve = ve;
    profile->ata = amax;
    profile->atd = amax;

    return GNC_CAL_SUCCESS;
}

/// @brief 根据加减速曲线计算每个插补周期的位移
/// @param params 机床参数
/// @param pl_block 插补段信息
/// @param profile 加减速曲线
/// @param cycle 第n个插补周期
/// @return s 第n个插补周期的距离（总距离，累加）
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static void T_shape_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data){
    double t = cycle * cycle_time;

    if (IsLessThanOrEqual(t - profile->t[0], 0)){
        traj_data->s = profile->vs * t + 0.5 * profile->ata * pow(t,2);
        traj_data->v = profile->vs + profile->ata * t;
        traj_data->a = profile->ata;
        traj_data->j = 0;
    }
    else if(IsLessThanOrEqual(t - profile->t[1], 0)){
        traj_data->s = profile->s[0] + profile->vpeak * (t - profile->t[0]);
        traj_data->v = profile->v[0];
        traj_data->a = 0;
        traj_data->j = 0;
    }
    else if(IsLessThanOrEqual(t - profile->t[2], 0)){
        traj_data->s = profile->s[1] +  profile->vpeak * (t - profile->t[1]) - 0.5 * profile->atd * pow((t - profile->t[1]),2);
        traj_data->v = profile->v[1] - profile->atd * (t - profile->t[1]);
        traj_data->a = -profile->atd;
        traj_data->j = 0;
    }
    else{
        traj_data->s = profile->s[2];
        traj_data->v = profile->v[2];
        traj_data->a = 0;
        traj_data->j = 0;
    }
}

/// @brief s型加减速计算方法1，二分法逼近1
/// @param params 
/// @param pl_block 
/// @param tol 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static GNC_RES scurve_cal_param1(double cycle_time, PLAN_BLOCK* pl_block, double tol, VEL_PROFILE* profile) {
    double vs = pl_block->start_vel;
    double ve = pl_block->end_vel;
    double vmax = pl_block->max_vel;
    double amax = pl_block->max_acc;  // 自动加工和回零的加速度和加加速度不同
    double jmax = pl_block->max_jerk;
    double s = pl_block->length;

    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
    double vpeak = vmax;

    double sscri = cal_scurve_cri_length(vs, ve, amax, jmax);
    double slcri = cal_scurve_cri_length(vs, vmax, amax, jmax) + cal_scurve_cri_length(vmax, ve, amax, jmax);

    if (IsLessThan(s, sscri)) return GNC_CAL_ERROR;

    if (IsEqualTo(s, sscri)) {
        if ((IsLessThan(vs, ve)) && (IsLessThanOrEqual(ve - vs, (amax * amax / jmax)))) {
            t1 = sqrt((ve - vs) / jmax);
            t3 = t1;
            vpeak = ve;
        }
        if ((IsLessThan(vs, ve)) && (IsGreaterThan(ve - vs, (amax * amax / jmax)))) {
            vpeak = ve;
            t1 = amax / jmax;
            t2 = ((vpeak - vs) / amax - amax / jmax);
            t3 = t1;            
        }
        if ((IsGreaterThan(vs, ve)) && (IsLessThanOrEqual(vs - ve, (amax * amax / jmax)))) {
            t5 = sqrt((vs - ve) / jmax);
            t7 = t5;
            vpeak = vs;
        }
        if ((IsGreaterThan(vs, ve)) && (IsGreaterThan(vs - ve, (amax * amax / jmax)))) {
            vpeak = vs;
            t5 = amax / jmax;
            t6 = ((vpeak - ve) / amax - amax / jmax);
            t7 = t5;  
        }
    } else if (IsGreaterThan(s, slcri)) {
        if ((IsLessThan(vs, vmax)) && (IsLessThanOrEqual(vmax - vs, (amax * amax / jmax)))) {
            t1 = sqrt((vmax - vs) / jmax);
            t3 = t1;            
        }
        if ((IsLessThan(vs, vmax)) && (IsGreaterThan(vmax - vs, (amax * amax / jmax)))) {
            t1 = amax / jmax;
            t2 = ((vmax - vs) / amax - amax / jmax);
            t3 = t1;            
        }
        if ((IsGreaterThan(vmax, ve)) && (IsLessThanOrEqual(vmax - ve, (amax * amax / jmax)))) {
            t5 = sqrt((vmax - ve) / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vmax, ve)) && (IsGreaterThan(vmax - ve, (amax * amax / jmax)))) {
            t5 = amax / jmax;
            t6 = ((vmax - ve) / amax - amax / jmax);
            t7 = t5;  
        }
        t4 = (s - slcri) / vmax;
        vpeak = vmax;
    } else {
        double v0 = fmax(vs, ve);
        double v1 = vpeak;
        double v = (v0 + v1) / 2;
        while (true) {
            double sa = cal_scurve_cri_length(v, vs, amax, jmax);
            double sb = cal_scurve_cri_length(v, ve, amax, jmax);
            if (fabs(sa + sb - s) <= tol) {
                vpeak = v;
                break;
            }
            if ((sa + sb - s) > tol) {
                v1 = v;
                v = (v0 + v) / 2;
            }
            if ((sa + sb - s) < -tol) {
                v0 = v;
                v = (v + v1) / 2;
            }
        }
        
        if ((IsLessThan(vs, vpeak)) && (IsLessThanOrEqual(vpeak - vs, (amax * amax / jmax)))) {
            t1 = sqrt((vpeak - vs) / jmax);
            t3 = t1;
        }
        if ((IsLessThan(vs, vpeak)) && (IsGreaterThan(vpeak - vs, (amax * amax / jmax)))) {
            t1 = amax / jmax;
            t2 = ((vpeak - vs) / amax - amax / jmax);
            t3 = t1;            
        }
        if ((IsGreaterThan(vpeak, ve)) && (IsLessThanOrEqual(vpeak - ve, (amax * amax / jmax)))) {
            t5 = sqrt((vpeak - ve) / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vpeak, ve)) && (IsGreaterThan(vpeak - ve, (amax * amax / jmax)))) {
            t5 = amax / jmax;
            t6 = ((vpeak - ve) / amax - amax / jmax);
            t7 = t5;  
        }
    }

    profile->t[0] = t1;
    profile->t[1] = profile->t[0] + t2;
    profile->t[2] = profile->t[1] + t3;
    profile->t[3] = profile->t[2] + t4;
    profile->t[4] = profile->t[3] + t5;
    profile->t[5] = profile->t[4] + t6;
    profile->t[6] = profile->t[5] + t7;

    profile->jta = jmax;
    profile->jtd = jmax;
    profile->ata = jmax * t1;
    profile->atd = jmax * t5;
    profile->vs = pl_block->start_vel;
    profile->ve = pl_block->end_vel;
    profile->vpeak = vpeak;
    profile->t_cycle = ceil(profile->t[6] / cycle_time);
    
    profile->v[0] = pl_block->start_vel + 0.5 * jmax * t1 * t1;
    profile->v[1] = profile->v[0] + profile->ata * t2;
    profile->v[2] = profile->v[1] + profile->ata * t3 - 0.5 * jmax * t3 * t3;
    profile->v[3] = profile->v[2];
    profile->v[4] = profile->v[3] - 0.5 * jmax * t5 * t5;
    profile->v[5] = profile->v[4] - profile->atd * t6;
    profile->v[6] = profile->v[5] - profile->atd * t7 + 0.5 * jmax * t7 * t7;

    profile->s[0] = vs * profile->t[0] + 1.0/6.0 * jmax * pow(t1, 3);
    profile->s[1] = profile->s[0] + profile->v[0] * t2 + 0.5 * profile->ata * pow(t2, 2);
    profile->s[2] = profile->s[1] + profile->v[1] * t3 + 0.5 * profile->ata * pow(t3, 2) - 1.0/6.0 * jmax * pow(t3, 3);
    profile->s[3] = profile->s[2] + profile->v[2] * t4;
    profile->s[4] = profile->s[3] + profile->v[3] * t5 - 1.0/6.0 * jmax * pow(t5, 3);
    profile->s[5] = profile->s[4] + profile->v[4] * t6 - 0.5 * profile->atd * pow(t6, 2);
    profile->s[6] = pl_block->length;

    return GNC_CAL_SUCCESS;
}

/// @brief s型加减速计算方法2：解析法
/// @param params 
/// @param pl_block 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static GNC_RES scurve_cal_param2(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* profile) {
    double vs = pl_block->start_vel;
    double ve = pl_block->end_vel;
    double vmax = pl_block->max_vel;
    double amax = pl_block->max_acc;  // 自动加工和回零的加速度和加加速度不同
    double jmax = pl_block->max_jerk;
    double s = pl_block->length;
    double tol = 1e-6;

    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
    double vpeak = vmax;

    double sscri = cal_scurve_cri_length(vs, ve, amax, jmax);
    double slcri = cal_scurve_cri_length(vs, vmax, amax, jmax) + cal_scurve_cri_length(vmax, ve, amax, jmax);

    if (IsLessThan(s, sscri)) return GNC_CAL_ERROR;

    if (IsEqualTo(s, sscri)) {
        if ((IsLessThan(vs, ve)) && (IsLessThanOrEqual(ve - vs, (amax * amax / jmax)))) {
            t1 = sqrt((ve - vs) / jmax);
            t3 = t1;
            vpeak = ve;
        }
        if ((IsLessThan(vs, ve)) && (IsGreaterThan(ve - vs, (amax * amax / jmax)))) {
            vpeak = ve;
            t1 = amax / jmax;
            t2 = ((vpeak - vs) / amax - amax / jmax);
            t3 = t1;            
        }
        if ((IsGreaterThan(vs, ve)) && (IsLessThanOrEqual(vs - ve, (amax * amax / jmax)))) {
            t5 = sqrt((vs - ve) / jmax);
            t7 = t5;
            vpeak = vs;
        }
        if ((IsGreaterThan(vs, ve)) && (IsGreaterThan(vs - ve, (amax * amax / jmax)))) {
            vpeak = vs;
            t5 = amax / jmax;
            t6 = ((vpeak - ve) / amax - amax / jmax);
            t7 = t5;  
        }
    } else if (IsGreaterThan(s, slcri)) {
        if ((IsLessThan(vs, vmax)) && (IsLessThanOrEqual(vmax - vs, (amax * amax / jmax)))) {
            t1 = sqrt((vmax - vs) / jmax);
            t3 = t1;            
        }
        if ((IsLessThan(vs, vmax)) && (IsGreaterThan(vmax - vs, (amax * amax / jmax)))) {
            t1 = amax / jmax;
            t2 = ((vmax - vs) / amax - amax / jmax);
            t3 = t1;            
        }
        if ((IsGreaterThan(vmax, ve)) && (IsLessThanOrEqual(vmax - ve, (amax * amax / jmax)))) {
            t5 = sqrt((vmax - ve) / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vmax, ve)) && (IsGreaterThan(vmax - ve, (amax * amax / jmax)))) {
            t5 = amax / jmax;
            t6 = ((vmax - ve) / amax - amax / jmax);
            t7 = t5;  
        }
        t4 = (s - slcri) / vmax;
        vpeak = vmax;
    } else {
        double v_temp  = fmax(vs, ve);
        int v_flag = 1;
        if(IsGreaterThan(v_temp, vs)){
            ve = vs;
            vs = v_temp;
            v_flag = 0;
        }

        double roots[4];
        int num_roots;
        
        // vs > ve
        double a_2 = 2.0 / amax;
        double b_2 = 2.0 * amax / jmax;
        double c_2 = amax / jmax * (vs + ve) - (vs*vs + ve*ve) / amax - 2.0 * s;
        num_roots = solve_quadratic_equation(a_2, b_2, c_2, roots);
        double vm = get_min_nonnegative_real_root(num_roots, roots);

        if ((IsGreaterThan(vm, vs + amax * amax / jmax)) && (IsLessThan(vm, vmax))) { // 匀加速段和匀减速段
            vpeak = vm;
            if (v_flag) {
                t1 = amax / jmax; t2 = ((vm - vs) / amax - t1); t3 = t1;
                t5 = amax / jmax; t6 = ((vm - ve) / amax - t5); t7 = t5;
            } else {
                t5 = amax / jmax; t6 = ((vm - vs) / amax - t5); t7 = t5;
                t1 = amax / jmax; t2 = ((vm - ve) / amax - t1); t3 = t1;
            }
        } else {
            double a_4 = 1.0 / (amax * amax);
            double b_4 = -2.0 / jmax;
            double c_4 = (amax * amax) / (jmax * jmax) - 4.0 * s / amax + 2.0 * ve / jmax - 2.0 * ve * ve / (amax * amax) - 4.0 * vs / jmax;
            double d_4 = -4.0 * amax * s / jmax + 2.0 * amax * amax * ve / (jmax * jmax) - 2.0 * ve * ve / jmax + 4.0 * vs * vs / jmax;
            double e_4 = pow(2.0 * s - amax * ve / jmax + ve * ve / amax, 2) + 4.0 * vs * vs * vs / jmax;
            num_roots = solve_quartic_equation(a_4, b_4, c_4, d_4, e_4, roots);
            vm = get_min_nonnegative_real_root(num_roots, roots);

            double v1 = fmax(vs, ve + amax * amax / jmax);
            double v2_ = fmin(vs + amax * amax / jmax, vmax);
            if ((IsGreaterThan(vm, v1)) && (IsLessThan(vm, v2_))) { // 只有匀减速段
                vpeak = vm;
                if (v_flag) {
                    t1 = sqrt((vm - vs) / jmax); t2 = 0; t3 = t1;
                    t5 = amax / jmax; t6 = ((vm - ve) / amax - t5); t7 = t5;
                } else {
                    t5 = sqrt((vm - vs) / jmax); t6 = 0; t7 = t5;
                    t1 = amax / jmax; t2 = ((vm - ve) / amax - t1); t3 = t1;
                }
            } else { // 没有匀加速段和匀减速段
                float a_4_2 = pow((ve - vs) / jmax, 2);
                float b_4_2 = 2.0 * (ve - vs) * (vs * vs - ve * ve) / (jmax * jmax) - 4.0 * s * s / jmax;
                float c_4_2 = pow((vs * vs - ve * ve) / jmax, 2) + 2.0 * (ve - vs) / jmax * (s * s + (pow(vs, 3) - pow(ve, 3)) / jmax) - 4.0 * s * s * ve / jmax;
                float d_4_2 = 2.0 * (vs * vs - ve * ve) / jmax * (s * s + (pow(vs, 3) - pow(ve, 3)) / jmax) + 4.0 * s * s * ve * ve / jmax;
                float e_4_2 = pow(s * s + (pow(vs, 3) - pow(ve, 3)) / jmax, 2) + 4.0 * s * s * pow(ve, 3) / jmax;
                num_roots = solve_quartic_equation(a_4_2, b_4_2, c_4_2, d_4_2, e_4_2, roots);
                vm = get_min_nonnegative_real_root(num_roots, roots);
                // todo：检查求解结果合法性
                vpeak = vm;
                if (v_flag) {
                    t1 = sqrt((vm - vs) / jmax); t2 = 0; t3 = t1;
                    t5 = sqrt((vm - ve) / jmax); t6 = 0; t7 = t5;
                } else {
                    t5 = sqrt((vm - vs) / jmax); t6 = 0; t7 = t5;
                    t1 = sqrt((vm - ve) / jmax); t2 = 0; t3 = t1;
                }
            }
        }
    }

    profile->t[0] = t1;
    profile->t[1] = profile->t[0] + t2;
    profile->t[2] = profile->t[1] + t3;
    profile->t[3] = profile->t[2] + t4;
    profile->t[4] = profile->t[3] + t5;
    profile->t[5] = profile->t[4] + t6;
    profile->t[6] = profile->t[5] + t7;

    profile->jta = jmax;
    profile->jtd = jmax;
    profile->ata = jmax * t1;
    profile->atd = jmax * t5;
    profile->vs = pl_block->start_vel;
    profile->ve = pl_block->end_vel;
    profile->vpeak = vpeak;
    profile->t_cycle = ceil(profile->t[6] /cycle_time);
    
    profile->v[0] = pl_block->start_vel + 0.5 * jmax * t1 * t1;
    profile->v[1] = profile->v[0] + profile->ata * t2;
    profile->v[2] = profile->v[1] + profile->ata * t3 - 0.5 * jmax * t3 * t3;
    profile->v[3] = profile->v[2];
    profile->v[4] = profile->v[3] - 0.5 * jmax * t5 * t5;
    profile->v[5] = profile->v[4] - profile->atd * t6;
    profile->v[6] = profile->v[5] - profile->atd * t7 + 0.5 * jmax * t7 * t7;

    profile->s[0] = pl_block->start_vel * t1 + 1.0/6.0 * jmax * pow(t1, 3);
    profile->s[1] = profile->s[0] + profile->v[0] * t2 + 0.5 * profile->ata * pow(t2, 2);
    profile->s[2] = profile->s[1] + profile->v[1] * t3 + 0.5 * profile->ata * pow(t3, 2) - 1.0/6.0 * jmax * pow(t3, 3);
    profile->s[3] = profile->s[2] + profile->v[2] * t4;
    profile->s[4] = profile->s[3] + profile->v[3] * t5 - 1.0/6.0 * jmax * pow(t5, 3);
    profile->s[5] = profile->s[4] + profile->v[4] * t6 - 0.5 * profile->atd * pow(t6, 2);
    profile->s[6] = pl_block->length;

    return GNC_CAL_SUCCESS;
}

/// @brief s型加减速插补s计算
/// @param params 
/// @param profile 
/// @param cycle 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static void scurve_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data) {
    double current_t = cycle *cycle_time;

    if (IsLessThanOrEqual(current_t, profile->t[0])) {
        traj_data->s = profile->vs * current_t + 1.0/6.0 * profile->jta * pow(current_t, 3);
        traj_data->v = profile->vs + 0.5 * profile->jta * pow(current_t, 2);
        traj_data->a = profile->jta * current_t;
        traj_data->j = profile->jta;
    } else if (IsLessThanOrEqual(current_t, profile->t[1])) {
        double temp_t = current_t - profile->t[0];
        traj_data->s = profile->s[0] + profile->v[0] * temp_t + 0.5 * profile->ata * pow(temp_t, 2);
        traj_data->v = profile->v[0] + profile->ata * temp_t;
        traj_data->a = profile->ata;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[2])) {
        double temp_t = current_t - profile->t[1];
        traj_data->s = profile->s[1] + profile->v[1] * temp_t + 0.5 * profile->ata * pow(temp_t, 2) - 1.0/6.0 * profile->jta * pow(temp_t, 3);
        traj_data->v = profile->v[1] + profile->ata * temp_t - 0.5 * profile->jta * pow(temp_t, 2);
        traj_data->a = profile->ata - profile->jta * temp_t;
        traj_data->j = - profile->jta;
    } else if (IsLessThanOrEqual(current_t, profile->t[3])) {
        double temp_t = current_t - profile->t[2];
        traj_data->s = profile->s[2] + profile->v[2] * temp_t;
        traj_data->v = profile->v[2];
        traj_data->a = 0;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[4])) {
        double temp_t = current_t - profile->t[3];
        traj_data->s = profile->s[3] + profile->v[3] * temp_t - 1.0/6.0 * profile->jtd * pow(temp_t, 3);
        traj_data->v = profile->v[3] - 0.5 * profile->jtd * pow(temp_t, 2);
        traj_data->a = - profile->jtd * temp_t;
        traj_data->j = - profile->jtd;
    } else if (IsLessThanOrEqual(current_t, profile->t[5])) {
        double temp_t = current_t - profile->t[4];
        traj_data->s = profile->s[4] + profile->v[4] * temp_t - 0.5 * profile->atd * pow(temp_t, 2);
        traj_data->v = profile->v[4] - profile->atd * temp_t;
        traj_data->a = - profile->atd;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[6])) {
        double temp_t = current_t - profile->t[5];
        traj_data->s = profile->s[5] + profile->v[5] * temp_t - 0.5 * profile->atd * pow(temp_t, 2) + 1.0/6.0 * profile->jtd * pow(temp_t, 3);
        traj_data->v = profile->v[5] - profile->atd * temp_t + 0.5 * profile->jtd * pow(temp_t, 2);
        traj_data->a = - profile->atd + profile->jtd * temp_t;
        traj_data->j = - profile->jtd;
    } else {
        traj_data->s = profile->s[6];
        traj_data->v = profile->v[6];
        traj_data->a = 0;
        traj_data->j = 0;
    }
}

/// @brief sin型加减速计算方法1，二分法逼近
/// @param params 
/// @param pl_block 
/// @param tol 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static GNC_RES sin_cal_param1(double cycle_time, PLAN_BLOCK* pl_block, double tol, VEL_PROFILE* profile) {
    double vs = pl_block->start_vel;
    double ve = pl_block->end_vel;
    double amax = pl_block->max_acc;  // 自动加工和回零的加速度和加加速度不同
    double jmax = pl_block->max_jerk;
    double vmax = pl_block->max_vel;
    double s = pl_block->length;

    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
    double vpeak = vmax;

    double sscri = cal_sin_cri_length(vs, ve, amax, jmax);
    double slcri = cal_sin_cri_length(vs, vmax, amax, jmax) + cal_sin_cri_length(vmax, ve, amax, jmax);

    if (IsLessThan(s, sscri)) return GNC_CAL_ERROR;

    if (IsEqualTo(s, sscri)) {
        if ((IsLessThan(vs, ve)) && (IsLessThanOrEqual(ve - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = sqrt(0.5 * (ve - vs) * PI / jmax);
            t3 = t1;
            vpeak = ve;
        }
        if ((IsLessThan(vs, ve)) && (IsGreaterThan(ve - vs, (0.5 * amax * amax * PI / jmax)))) {
            vpeak = ve;
            t1 = 0.5 * amax * PI / jmax;
            t2 = ((vpeak - vs) / amax - t1);
            t3 = t1;            
        }
        if ((IsGreaterThan(vs, ve)) && (IsLessThanOrEqual(vs - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = sqrt(0.5 * (vs - ve) * PI / jmax);
            t7 = t5;
            vpeak = vs;
        }
        if ((IsGreaterThan(vs, ve)) && (IsGreaterThan(vs - ve, (0.5 * amax * amax * PI / jmax)))) {
            vpeak = vs;
            t5 = 0.5 * amax * PI / jmax;
            t6 = ((vpeak - ve) / amax - t5);
            t7 = t5;  
        }
    } else if (IsGreaterThan(s, slcri)) {
        if ((IsLessThan(vs, vmax)) && (IsLessThanOrEqual(vmax - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = sqrt(0.5 * (vmax - vs) * PI / jmax);
            t3 = t1;            
        }
        if ((IsLessThan(vs, vmax)) && (IsGreaterThan(vmax - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = 0.5 * amax * PI / jmax;
            t2 = ((vmax - vs) / amax - t1);
            t3 = t1;            
        }
        if ((IsGreaterThan(vmax, ve)) && (IsLessThanOrEqual(vmax - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = sqrt(0.5 * (vmax - ve) * PI / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vmax, ve)) && (IsGreaterThan(vmax - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = 0.5 * amax * PI / jmax;
            t6 = ((vmax - ve) / amax - t5);
            t7 = t5;  
        }
        t4 = (s - slcri) / vmax;
        vpeak = vmax;
    } else {
        double v0 = fmax(vs, ve);
        double v1_ = vpeak;
        double v = (v0 + v1_) / 2;
        while (true) {
            double sa = cal_sin_cri_length(v, vs, amax, jmax);
            double sb = cal_sin_cri_length(v, ve, amax, jmax);
            if (fabs(sa + sb - s) <= tol) {
                vpeak = v;
                break;
            }
            if ((sa + sb - s) > tol) {
                v1_ = v;
                v = (v0 + v) / 2;
            }
            if ((sa + sb - s) < -tol) {
                v0 = v;
                v = (v + v1_) / 2;
            }
        }
        
        if ((IsLessThan(vs, vpeak)) && (IsLessThanOrEqual(vpeak - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = sqrt(0.5 * (vpeak - vs) * PI / jmax);
            t3 = t1;
        }
        if ((IsLessThan(vs, vpeak)) && (IsGreaterThan(vpeak - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = 0.5 * amax * PI / jmax;
            t2 = ((vpeak - vs) / amax - t1);
            t3 = t1;            
        }
        if ((IsGreaterThan(vpeak, ve)) && (IsLessThanOrEqual(vpeak - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = sqrt(0.5 * (vpeak - ve) * PI / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vpeak, ve)) && (IsGreaterThan(vpeak - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = 0.5 * amax * PI / jmax;
            t6 = ((vpeak - ve) / amax - t5);
            t7 = t5;  
        }
    }

    profile->t[0] = t1;
    profile->t[1] = profile->t[0] + t2;
    profile->t[2] = profile->t[1] + t3;
    profile->t[3] = profile->t[2] + t4;
    profile->t[4] = profile->t[3] + t5;
    profile->t[5] = profile->t[4] + t6;
    profile->t[6] = profile->t[5] + t7;

    profile->vs = pl_block->start_vel;
    profile->ve = pl_block->end_vel;
    profile->vpeak = vpeak;
    profile->jta = jmax;
    profile->jtd = jmax;
    profile->ata = 2 * t1 * jmax / PI;
    profile->atd = 2 * t5 * jmax / PI;

    profile->v[0] = profile->vs + profile->ata * t1 / 2.0;
    profile->v[1] = profile->v[0] + profile->ata * t2;
    profile->v[2] = profile->v[1] + profile->ata * t3 / 2.0;
    profile->v[3] = profile->v[2];
    profile->v[4] = profile->v[3] - profile->atd * t5 / 2.0;
    profile->v[5] = profile->v[4] - profile->atd * t6;
    profile->v[6] = profile->ve;

    profile->s1_ = profile->ata / 2.0 * pow(t1, 2) / (PI * PI);
    profile->s5_ = profile->atd / 2.0 * pow(t5, 2) / (PI * PI);

    profile->s[0] = - profile->s1_ + vs * t1 - 0.5 * profile->ata * pow(t1 / PI, 2) + 0.25 * profile->ata * pow(t1, 2);
    profile->s[1] = profile->s[0] + profile->v[0] * t2 + 0.5 * profile->ata * pow(t2, 2);
    profile->s[2] = profile->s1_ + profile->s[1] + profile->v[1] * t3 + 0.5 * profile->ata * pow(t3 / PI, 2) + 0.25 * profile->ata * pow(t3, 2);
    profile->s[3] = profile->s[2] + profile->v[2] * t4;
    profile->s[4] = profile->s5_ + profile->s[3] + profile->v[3] * t5 + 0.5 * profile->atd * pow(t5 / PI, 2) - 0.25 * profile->atd * pow(t5, 2);
    profile->s[5] = profile->s[4] + profile->v[4] * t6 - 0.5 * profile->atd * pow(t6, 2);
    profile->s[6] = pl_block->length;

    profile->t_cycle = ceil(profile->t[6] / cycle_time);

    return GNC_CAL_SUCCESS;
}

/// @brief sin型加减速计算方法2：解析法
/// @param params 
/// @param pl_block 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static GNC_RES sin_cal_param2(double cycle_time, PLAN_BLOCK* pl_block, VEL_PROFILE* profile) {
    double vs = pl_block->start_vel;
    double ve = pl_block->end_vel;
    double vmax = pl_block->max_vel;
    double amax = pl_block->max_acc;  // 自动加工和回零的加速度和加加速度不同
    double jmax = pl_block->max_jerk;
    double s = pl_block->length;

    double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
    double vpeak = vmax;

    double sscri = cal_sin_cri_length(vs, ve, amax, jmax);
    double slcri = cal_sin_cri_length(vs, vmax, amax, jmax) + cal_sin_cri_length(vmax, ve, amax, jmax);

    if (IsLessThan(s, sscri)) return GNC_CAL_ERROR;

    if (IsEqualTo(s, sscri)) {
        if ((IsLessThan(vs, ve)) && (IsLessThanOrEqual(ve - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = s / (ve + vs);
            t3 = t1;
            vpeak = ve;
        }
        if ((IsLessThan(vs, ve)) && (IsGreaterThan(ve - vs, (0.5 * amax * amax * PI / jmax)))) {
            vpeak = ve;
            t1 = 0.5 * amax * PI / jmax;
            t2 = ((vpeak - vs) / amax - t1);
            t3 = t1;
        }
        if ((IsGreaterThan(vs, ve)) && (IsLessThanOrEqual(vs - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = s / (ve + vs);
            t7 = t5;
            vpeak = vs;
        }
        if ((IsGreaterThan(vs, ve)) && (IsGreaterThan(vs - ve, (0.5 * amax * amax * PI / jmax)))) {
            vpeak = vs;
            t5 = 0.5 * amax * PI / jmax;
            t6 = ((vpeak - ve) / amax - t5);
            t7 = t5;
        }
    } else if (IsGreaterThan(s, slcri)) {
        if ((IsLessThan(vs, vmax)) && (IsLessThanOrEqual(vmax - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = sqrt(0.5 * (vmax - vs) * PI / jmax);
            t3 = t1;
        }
        if ((IsLessThan(vs, vmax)) && (IsGreaterThan(vmax - vs, (0.5 * amax * amax * PI / jmax)))) {
            t1 = 0.5 * amax * PI / jmax;
            t2 = ((vmax - vs) / amax - t1);
            t3 = t1;
        }
        if ((IsGreaterThan(vmax, ve)) && (IsLessThanOrEqual(vmax - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = sqrt(0.5 * (vmax - ve) * PI / jmax);
            t7 = t5;
        }
        if ((IsGreaterThan(vmax, ve)) && (IsGreaterThan(vmax - ve, (0.5 * amax * amax * PI / jmax)))) {
            t5 = 0.5 * amax * PI / jmax;
            t6 = ((vmax - ve) / amax - t5);
            t7 = t5;
        }
        t4 = (s - slcri) / vmax;
        vpeak = vmax;
    } else {
        double v_temp  = fmax(vs, ve);
        int v_flag = 1;
        if(IsGreaterThan(v_temp, vs)){
            ve = vs;
            vs = v_temp;
            v_flag = 0;
        }

        double roots[4];
        int num_roots;

        // vs > ve
        double a_2 = 2.0 / amax;
        double b_2 = PI * amax / jmax;
        double c_2 = 0.5 * PI * amax / jmax * (vs + ve) - (vs * vs + ve * ve) / amax - 2.0 * s;
        num_roots = solve_quadratic_equation(a_2, b_2, c_2, roots);
        double vm = get_min_nonnegative_real_root(num_roots, roots);

        if ((IsGreaterThan(vm, vs + 0.5 * PI * amax * amax / jmax)) && (IsLessThan(vm, vmax))) { // 匀加速段和匀减速段
            vpeak = vm;
            if (v_flag) {
                t1 = 0.5 * amax * PI / jmax; t2 = ((vm - vs) / amax - t1); t3 = t1;
                t5 = 0.5 * amax * PI / jmax; t6 = ((vm - ve) / amax - t5); t7 = t5;
            } else {
                t5 = 0.5 * amax * PI / jmax; t6 = ((vm - vs) / amax - t5); t7 = t5;
                t1 = 0.5 * amax * PI / jmax; t2 = ((vm - ve) / amax - t1); t3 = t1;
            }
        } else {
            double a_4 = 0.25 / (amax * amax);
            double b_4 = -0.25 * PI / jmax;
            double c_4 = 1.0 / 16.0 * PI * PI * amax * amax / (jmax * jmax) - 0.5 * PI * vs / jmax + 1.0 / amax * (0.25 * PI * amax * ve / jmax - 0.5 * ve * ve / amax - s);
            double d_4 = 0.5 * PI * vs * vs / jmax + 0.5 * PI * amax / jmax * (0.25 * PI * amax * ve / jmax - 0.5 * ve * ve / amax - s);
            double e_4 = 0.5 * PI * pow(vs, 3) / jmax + pow(0.25 * PI * amax * ve / jmax - 0.5 * ve * ve / amax - s, 2);
            num_roots = solve_quartic_equation(a_4, b_4, c_4, d_4, e_4, roots);
            vm = get_min_nonnegative_real_root(num_roots, roots);

            double v1 = fmax(vs, ve + 0.5 * PI * amax * amax / jmax);
            double v2_ = fmin(vs + 0.5 * PI * amax * amax / jmax, vmax);
            if ((IsGreaterThan(vm, v1)) && (IsLessThan(vm, v2_))) { // 只有匀减速段
                vpeak = vm;
                if (v_flag) {
                    t1 = sqrt(0.5 * (vm - vs) * PI / jmax); t2 = 0; t3 = t1;
                    t5 = 0.5 * amax * PI / jmax; t6 = ((vm - ve) / amax - t5); t7 = t5;
                } else {
                    t5 = sqrt(0.5 * (vm - vs) * PI / jmax); t6 = 0; t7 = t5;
                    t1 = 0.5 * amax * PI / jmax; t2 = ((vm - ve) / amax - t1); t3 = t1;
                }
            } else { // 没有匀加速段和匀减速段
                double a_4_2 = pow(ve - vs, 2);
                double b_4_2 = 2.0 * (ve - vs) * (vs * vs - ve * ve) - 8.0 * jmax * s * s / PI;
                double c_4_2 = pow(vs * vs - ve * ve, 2) + 2.0 * (ve - vs) * (pow(vs, 3) - pow(ve, 3) + 2.0 * jmax * s * s / PI) - 8.0 * jmax * s * s / PI * ve;
                double d_4_2 = 2.0 * (vs * vs - ve * ve) * (pow(vs, 3) - pow(ve, 3) + 2.0 * jmax * s * s / PI) + 8.0 * jmax * s * s / PI * ve * ve;
                double e_4_2 = pow(pow(vs, 3) - pow(ve, 3) + 2.0 * jmax * s * s / PI, 2) + 8.0 * jmax * s * s / PI * pow(ve, 3);
                num_roots = solve_quartic_equation(a_4_2, b_4_2, c_4_2, d_4_2, e_4_2, roots);
                vm = get_min_nonnegative_real_root(num_roots, roots);
                vpeak = vm;
                if (v_flag) {
                    t1 = sqrt(0.5 * (vm - vs) * PI / jmax); t2 = 0; t3 = t1;
                    t5 = sqrt(0.5 * (vm - ve) * PI / jmax); t6 = 0; t7 = t5;
                } else {
                    t5 = sqrt(0.5 * (vm - vs) * PI / jmax); t6 = 0; t7 = t5;
                    t1 = sqrt(0.5 * (vm - ve) * PI / jmax); t2 = 0; t3 = t1;
                }
            }
        }
    }

    profile->t[0] = t1;
    profile->t[1] = profile->t[0] + t2;
    profile->t[2] = profile->t[1] + t3;
    profile->t[3] = profile->t[2] + t4;
    profile->t[4] = profile->t[3] + t5;
    profile->t[5] = profile->t[4] + t6;
    profile->t[6] = profile->t[5] + t7;

    profile->vs = pl_block->start_vel;
    profile->ve = pl_block->end_vel;
    profile->vpeak = vpeak;
    profile->jta = jmax;
    profile->jtd = jmax;
    profile->ata = 2 * t1 * jmax / PI;
    profile->atd = 2 * t5 * jmax / PI;

    profile->v[0] = profile->vs + profile->ata * t1 / 2.0;
    profile->v[1] = profile->v[0] + profile->ata * t2;
    profile->v[2] = profile->v[1] + profile->ata * t3 / 2.0;
    profile->v[3] = profile->v[2];
    profile->v[4] = profile->v[3] - profile->atd * t5 / 2.0;
    profile->v[5] = profile->v[4] - profile->atd * t6;
    profile->v[6] = profile->ve;

    profile->s1_ = profile->ata / 2.0 * pow(t1, 2) / (PI * PI);
    profile->s5_ = profile->atd / 2.0 * pow(t5, 2) / (PI * PI);

    profile->s[0] = - profile->s1_ + vs * t1 - 0.5 * profile->ata * pow(t1 / PI, 2) + 0.25 * profile->ata * pow(t1, 2);
    profile->s[1] = profile->s[0] + profile->v[0] * t2 + 0.5 * profile->ata * pow(t2, 2);
    profile->s[2] = profile->s1_ + profile->s[1] + profile->v[1] * t3 + 0.5 * profile->ata * pow(t3 / PI, 2) + 0.25 * profile->ata * pow(t3, 2);
    profile->s[3] = profile->s[2] + profile->v[2] * t4;
    profile->s[4] = profile->s5_ + profile->s[3] + profile->v[3] * t5 + 0.5 * profile->atd * pow(t5 / PI, 2) - 0.25 * profile->atd * pow(t5, 2);
    profile->s[5] = profile->s[4] + profile->v[4] * t6 - 0.5 * profile->atd * pow(t6, 2);
    profile->s[6] = pl_block->length;

    profile->t_cycle = ceil(profile->t[6] / cycle_time);

    return GNC_CAL_SUCCESS;
}

/// @brief sin型加减速插补s计算
/// @param params 
/// @param profile 
/// @param cycle 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/16
static void sin_cal_traj(double cycle_time, VEL_PROFILE* profile, int cycle, TRAJ_PARAMS* traj_data) {
    double current_t = cycle * cycle_time;

    if (IsLessThanOrEqual(current_t, profile->t[0])) {
        double temp_t = current_t;
        traj_data->s = -profile->s1_ + profile->vs * temp_t + 0.5 * profile->ata * pow(profile->t[0] / PI, 2) * cos(temp_t * PI / profile->t[0]) + 0.25 * profile->ata * pow(temp_t, 2);
        traj_data->v = profile->vs + 0.5 * profile->ata * temp_t - 0.5 * profile->ata * profile->t[0] / PI * sin(temp_t * PI / profile->t[0]);
        traj_data->a = - 0.5 * profile->ata * cos(temp_t * PI / profile->t[0]) + 0.5 * profile->ata;
        traj_data->j = 0.5 * profile->ata * PI / profile->t[0] * sin(temp_t * PI / profile->t[0]);
    } else if (IsLessThanOrEqual(current_t, profile->t[1])) {
        double temp_t = current_t - profile->t[0];
        traj_data->s = profile->s[0] + profile->v[0] * temp_t + 0.5 * profile->ata * pow(temp_t, 2);
        traj_data->v = profile->v[0] + profile->ata * temp_t;
        traj_data->a = profile->ata;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[2])) {
        double temp_t = current_t - profile->t[1];
        double t3 = profile->t[2] - profile->t[1];
        traj_data->s = profile->s1_ + profile->s[1] + profile->v[1] * temp_t - 0.5 * profile->ata * pow(t3 / PI, 2) * cos(temp_t * PI / t3) + 0.25 * profile->ata * pow(temp_t, 2);
        traj_data->v = profile->v[1] + 0.5 * profile->ata * temp_t + 0.5 * profile->ata * t3 / PI * sin(temp_t * PI / t3);
        traj_data->a = 0.5 * profile->ata * cos(temp_t * PI / t3) + 0.5 * profile->ata;
        traj_data->j = -0.5 * profile->ata * PI / t3 * sin(temp_t * PI / t3);
    } else if (IsLessThanOrEqual(current_t, profile->t[3])) {
        double temp_t = current_t - profile->t[2];
        traj_data->s = profile->s[2] + profile->v[2] * temp_t;
        traj_data->v = profile->v[2];
        traj_data->a = 0;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[4])) {
        double temp_t = current_t - profile->t[3];
        double t5 = profile->t[4] - profile->t[3];
        traj_data->s = profile->s5_ + profile->s[3] + profile->v[3] * temp_t - 0.5 * profile->atd * pow(t5 / PI, 2) * cos(temp_t * PI / t5) - 0.25 * profile->atd * pow(temp_t, 2);
        traj_data->v = profile->v[3] - 0.5 * profile->atd * temp_t + 0.5 * profile->atd * t5 / PI * sin(temp_t * PI / t5);
        traj_data->a = 0.5 * profile->atd * cos(temp_t * PI / t5) - 0.5 * profile->atd;
        traj_data->j = -0.5 * profile->atd * PI / t5 * sin(temp_t * PI / t5);
    } else if (IsLessThanOrEqual(current_t, profile->t[5])) {
        double temp_t = current_t - profile->t[4];
        traj_data->s = profile->s[4] + profile->v[4] * temp_t - 0.5 * profile->atd * pow(temp_t, 2);
        traj_data->v = profile->v[4] - profile->atd * temp_t;
        traj_data->a = -profile->atd;
        traj_data->j = 0;
    } else if (IsLessThanOrEqual(current_t, profile->t[6])) {
        double temp_t = current_t - profile->t[5];
        double t7 = profile->t[6] - profile->t[5];
        traj_data->s = -profile->s5_ + profile->s[5] + profile->v[5] * temp_t + 0.5 * profile->atd * pow(t7 / PI, 2) * cos(temp_t * PI / t7) - 0.25 * profile->atd * pow(temp_t, 2);
        traj_data->v = profile->v[5] - 0.5 * profile->atd * temp_t - 0.5 * profile->atd * t7 / PI * sin(temp_t * PI / t7);
        traj_data->a = - 0.5 * profile->atd - 0.5 * profile->atd * cos(temp_t * PI / t7);
        traj_data->j = 0.5 * profile->atd * PI / t7 * sin(temp_t * PI / t7);
    } else {
        traj_data->s = profile->s[6];
        traj_data->v = profile->v[6];
        traj_data->a = 0;
        traj_data->j = 0;
    }
}

/// @brief T型加减速计算，圆整
/// @param params 
/// @param pl_block 
/// @param rProfile 
/// @return 
/// @version v1.0
/// @author 许晓婷
/// @date 2025/10/15
// static GNC_RES T_shape_cal_round_param(double cycle_time,  PLAN_BLOCK* pl_block, R_VEL_PROFILE* rProfile){
    
//     VEL_PROFILE* profile = T_shape_cal_param(params, pl_block);

//     double vs = pl_block->start_vel;
//     double ve = pl_block->end_vel;
//     double vpeak = pl_block->max_vel;
//     double s = pl_block->length;

//     // 计算圆整后时间
//     int N3[3];
//     N3[0] = ceil(profile->t[0] / cycle_time);
//     N3[1] = N3[0] + ceil((profile->t[1] - profile->t[0]) / cycle_time);
//     N3[2] = N3[1] + ceil((profile->t[2] - profile->t[1]) / cycle_time);

//     rProfile->T[0] = N3[0] * cycle_time;
//     rProfile->T[1] = N3[1] * cycle_time;
//     rProfile->T[2] = N3[2] * cycle_time;

//     if(N3[0] != 0){
//         rProfile->ata = (vpeak - vs) / rProfile->T[0];
//     }
//     if(N3[2] != 0){
//         rProfile->atd = (vpeak - ve) / (rProfile->T[2] - rProfile->T[1]);
//     }

//     // 计算圆整误差
//     double sa, sd, sc, se, de = 0;
//     if(N3[0] != 0 && N3[1] != 0 && N3[2] != 0){
//         sa = vs * rProfile->T[0] + 0.5 * rProfile->ata * pow(rProfile->T[0],2);
//         sc = vpeak * (rProfile->T[1] - rProfile->T[0]);
//         sd = vpeak * (rProfile->T[2] - rProfile->T[1]) - 0.5 * rProfile->atd * pow((rProfile->T[2] - rProfile->T[1]),2);
//     }
//     else if(N3[0] == 0 && N3[1] != 0 && N3[2] != 0){
//         sc = vpeak * (rProfile->T[1] - rProfile->T[0]);
//         sd = vpeak * (rProfile->T[2] - rProfile->T[1]) - 0.5 * rProfile->atd * pow((rProfile->T[2] - rProfile->T[1]),2);
//     }
//     else if(N3[0] != 0 && N3[1] != 0 && N3[2] == 0){
//         sa = vs * rProfile->T[0] + 0.5 * rProfile->ata * pow(rProfile->T[0],2);
//         sc = vpeak * (rProfile->T[1] - rProfile->T[0]);
//     }
//     else if(N3[0] != 0 && N3[1] == 0 && N3[2] != 0){
//         sa = vs * rProfile->T[0] + 0.5 * rProfile->ata * pow(rProfile->T[0],2);
//         sd = vpeak * (rProfile->T[2] - rProfile->T[1]) - 0.5 * rProfile->atd * pow((rProfile->T[2] - rProfile->T[1]),2);
//     }
//     else if(N3[0] == 0 && N3[1] == 0 && N3[2] != 0){
//         sd = vpeak * (rProfile->T[2] - rProfile->T[1]) - 0.5 * rProfile->atd * pow((rProfile->T[2] - rProfile->T[1]),2);
//     }
//     else if(N3[0] != 0 && N3[1] == 0 && N3[2] == 0){
//         sa = vs * rProfile->T[0] + 0.5 * rProfile->ata * pow(rProfile->T[0],2);
//     }
//     else if(N3[0] == 0 && N3[1] != 0 && N3[2] == 0){
//         sc = vpeak * (rProfile->T[1] - rProfile->T[0]);
//     }

//     se = s - (sa + sc + sd);
//     de = se / (N3[2] - N3[1] + N3[0]);

//     rProfile->de = de;
//     rProfile->T_cycle = N3[2];
//     rProfile->vpeak = vpeak;
//     rProfile->S[0] = sa - de * N3[0];
//     rProfile->S[1] = rProfile->S[0] + sc;
//     rProfile->S[2] = rProfile->S[1] + sd - de * (N3[2] -N3[1]);

//     return GNC_CAL_SUCCESS;
// }

