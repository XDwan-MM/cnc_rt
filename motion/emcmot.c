# include "emcmot.h"

static void handle_alarm_cmd();
static void handle_jog_cmd();
static void output_to_hal();


static void set_operating_mode(){
    // 读取当前面板的加工模式 sys->target_mode：手动、手轮、回零、自动（单段）、MDI、报警
   // static const char* mode_names[] = { "未知","报警", "回零", "MDI", "自动","JOG点动", "手轮", "快速",  "编辑", "DNC"};

   // printf("sys->current_mode%s sys->target_mode%s\n",mode_names[sys->current_mode],mode_names[sys->target_mode]);
    // 检查是否急停
    if(sys->is_estop_actived){
        sys->current_mode = MODE_ALARM;
        return;
    }

    // 如果没有切换，不进行操作
    if(sys->current_mode == sys->target_mode) {
        sys->is_switching_mode = false;
        return;
    }

    // 根据切换的加工模式和当前加工模式，判断是否可以切换
    switch (sys->current_mode)
    {
    case MODE_IDLE:
    case MODE_JOG:
    case MODE_EDIT:
    case MODE_RAPID:
        sys->current_mode = sys->target_mode;
        break;

    case MODE_HOMING:
        if( x_homing_tp->homing_state == ABS_EXEC_IDLE &&
            y_homing_tp->homing_state == ABS_EXEC_IDLE &&
            z_homing_tp->homing_state == ABS_EXEC_IDLE){
            sys->current_mode = sys->target_mode;
            sys->is_switching_mode = false;
            break;
        }
        sys->nc_event = SYS_EVENT_RESET;
        sys->is_switching_mode = true;
        break;

    case MODE_MDI:
    case MODE_MEM:
    case MODE_DNC:
        if(mem_exec_state == EXEC_IDLE){
            sys->current_mode = sys->target_mode;
            sys->is_switching_mode = false;
            break;
        }
        if(mem_exec_state == EXEC_RUNNING){
            // 自动加工过程中不能切换模式，应先暂停
            printf("mode change fail: please press feed hold at first!\n");
            sys->target_mode = sys->current_mode;
            break;
        }
        sys->nc_event = SYS_EVENT_RESET;
        sys->is_switching_mode = true;
        break;

    default:
        break;
    }
}

// 不同加工模式的运动控制
static void get_cmd_pos(){
    switch(sys->current_mode){
    case MODE_IDLE: // 空闲状态       
        // printf("current mode: IDLE.\n");
        break;

    case MODE_ALARM: // 系统报错            
        // printf("current mode: ALARM.\n");
        handle_alarm_cmd();
        break;

    case MODE_HOMING: // 回零  
        // printf("current mode: HOMING.\n");         
        handle_homing_cmd();
        break;

    case MODE_MDI:  // MDI
    case MODE_MEM:  // 自动
       //  printf("current mode: AUTO.\n");
        handle_run_cmd();
        break;

    // 更新：【手动模式】伍臣发
    case MODE_JOG: // 手动
        // printf("current mode: JOG.\n");
        handle_jog_cmd();
        break;

    // 更新：【手轮模式】张哲
    case MODE_MPG: // 手轮
        // printf("current mode: MPG.\n");
        // handle_mpg_cmd();
        break;
    
    // 更新：【报警模块】翟亚非
    default: // error      
        // printf("current mode: UNKNOWN.\n");
        break;
    }
}

// 报警状态逻辑
static void handle_alarm_cmd(){
    switch(sys->nc_event){
        case SYS_EVENT_NONE:
            break;

        case SYS_EVENT_RESET:
            // 如果急停未重置
            if(sys->is_estop_actived){
                printf("warning: please reset estop at first.\n");
                break;
            }
            // 状态复位、数据重置
            tp_reset();
            // 切换到IDLE模式
            sys_reset();
            break;
        
        case SYS_EVENT_RERUN:
        case SYS_EVENT_PAUSE:
        case SYS_EVENT_X:
        case SYS_EVENT_X_NEG:
        case SYS_EVENT_Y:
        case SYS_EVENT_Y_NEG:
        case SYS_EVENT_Z:
        case SYS_EVENT_Z_NEG:
            // 报警状态不支持其他操作
            printf("warning: alarm mode is not support this operation.\n");
            break;

        default:
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

// 调试：模拟点动
static void handle_jog_cmd(){
    switch(sys->nc_event){
        case SYS_EVENT_NONE:
        case SYS_EVENT_RESET:
        case SYS_EVENT_RERUN:
        case SYS_EVENT_PAUSE:
            break;

        case SYS_EVENT_X:      // +X 正向点动
            hal->target_pos.x += 100;
            break;

        case SYS_EVENT_X_NEG:  // -X 负向点动
            hal->target_pos.x -= 100;
            break;

        case SYS_EVENT_Y:      // +Y 正向点动
            hal->target_pos.y += 100;
            break;

        case SYS_EVENT_Y_NEG:  // -Y 负向点动
            hal->target_pos.y -= 100;
            break;

        case SYS_EVENT_Z:      // +Z 正向点动
            hal->target_pos.z += 100;
            break;

        case SYS_EVENT_Z_NEG:  // -Z 负向点动
            hal->target_pos.z -= 100;
            break;

        default:
            break;
    }
    sys->nc_event = SYS_EVENT_NONE;
}

// todo：检查系统错误
static void check_for_fault(){
    
}

// todo：更新hal数据
static void output_to_hal(){
    hal->current_vel = hal->target_vel;
    hal->x_current_vel = hal->x_target_vel;
    hal->y_current_vel = hal->y_target_vel;
    hal->z_current_vel = hal->z_target_vel;
    // hal->interp_pin = fabs(POS_Distance(&hal->target_pos, &hal->current_pos));
    hal->target_vel = hal->interp_pin / params->cycle_time; // 总目标速度
    hal->x_target_vel = fabs(hal->target_pos.x - hal->current_pos.x) / params->cycle_time; // x轴目标速度
    hal->y_target_vel = fabs(hal->target_pos.y - hal->current_pos.y) / params->cycle_time; // y轴目标速度
    hal->z_target_vel = fabs(hal->target_pos.z - hal->current_pos.z) / params->cycle_time; // z轴目标速度
    // 调试：假设目标位置已下发
    hal->current_pos = hal->target_pos;
    printf("X%f Y%f Z%f\n",hal->current_pos.x,hal->current_pos.y,hal->current_pos.z);
   // Hal_Driver_Set_Target_Pos(x_axis_driver,)
}

static void update_status(){
    write_all_data_file(log_file);
}

/*!
* @brief 轨迹规划（emcmotController）
* @param 
* @return 0：成功；-1：失败
*/
void emcmotTrajPlan(){

    // 处理探针信号
    // process_probe_inputs();

    // 检查系统是否有异常
    check_for_fault();
    
    // 遍历每个通道
    for(int i = 0; i < params->chan_num; i++){
        // 判断运动模式
        set_operating_mode();

        // 根据运动模式计算运动位置
        get_cmd_pos();

        // 笛卡尔坐标转换成关节坐标
        // kinematics_inverse();

        // 更新：【行程超限位】张哲
        // 预先检查插补位置是否超软限位
        // check_for_soft_limit();

        // 更新：【报警模块】翟亚飞
        // 如果预测到超出软限位，如何处理

        // 更新：【误差补偿】伍臣发
        // 误差补偿
        // compute_screw_comp();

        // 更新抽象层数据
        output_to_hal();

        // 上传运动状态数据
        update_status();
    }

}


/*!
* @brief 路径规划（解释器译码，平滑处理）
* @param [in] nc 文件
* @param [out] tq 平滑后的插补指令循环队列
* @return 0：成功；-1：失败
*/
void emcmotPathPlan(){
    // 遍历多个通道
    for(int i = 0; i < params->chan_num; i++){
        // 解释器译码
        // dcd_produce_blk();

        // 预处理，添加block
        chan_add_block();

        // 更新：【异常点处理】张哲
        // 异常点处理
        // chan_block_filter();

        // 更新：【拐角平滑】伍臣发
        // 如果开启拐角平滑：圆弧、贝塞尔、hermit
        // chan_corner_smooth();
        
        // 预留：NURBS平滑
        // 如果开启NURBS平滑
        // if(emcmotStatus->nurbs_smooth){
        //      chan_nurbs_fitting();
        // }
    }
}
