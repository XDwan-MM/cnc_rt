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
#include "nurbsfit.h"
#include "circle_smooth.h"

static void block_cal_pos(PLAN_BLOCK* parser_block, PLAN_BLOCK* current_block, POS* start_pos);
static void block_cal_vel(PLAN_BLOCK* current_block);
static void block_cal_angle(PLAN_BLOCK* current_block, VECTOR* pre_end_vector);
static void block_cal_circle_vector(PLAN_BLOCK* current_block);
static double block_cal_circle_length(PLAN_BLOCK* current_block);
static double block_cal_nurbs_length(PLAN_BLOCK* current_block);
static void block_deal_run_row(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block);
static void block_deal_parser_block(PLAN_BLOCK* parser_block, PLAN_BLOCK* pretreat_block);
static void block_deal_outlier(PLAN_BLOCK* pretreat_block, PLAN_BLOCK* treat_block);
static GNC_RES block_unif_by_nurbs();
static GNC_RES block_unif_by_treat();
static GNC_RES block_fixturnning_point();
// 圆弧平滑入口，使用明确的无参数函数原型。
static GNC_RES block_unif_by_circle_smooth(void);


//cy 异常点处理
static void block_cal_curvature(PLAN_BLOCK* current_block, POS* pre_start_pos, POS* pre_end_pos, POS* current_pos);
static int block_cal_segment(PLAN_BLOCK* pretreat_block, double seg_theta_thre,PLAN_QUEUE* temp);
static void block_fix_pos(PLAN_BLOCK* pretreat_block, double l_theta_thre ,double theta_thre, PLAN_QUEUE* temp);
static bool is_length_valid(PLAN_BLOCK* parser_block);
static void block_deal_nurbs_enable(PLAN_BLOCK* pretreat_block,PLAN_QUEUE* temp);
static int blocknum = 1;//未复位

/// @brief 将译码结果转换成插补段
GNC_RES chan_add_block(){
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* parser_block;

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

/// @brief 预处理：剔除异常点、滤波等
GNC_RES pretreat_block()
{
    PLAN_BLOCK* pretreat_block;
    PLAN_BLOCK* treat_block;

    //结束逻辑：
    if ((rt_data->chan_state[0][CHAN_IS_PARSERING] == false)&&queue_is_empty(parser_queue))
    {
        if(queue_get_size(pretreat_queue)==1)
        {
            treat_block = queue_peak(pretreat_queue);
            queue_push(treat_queue, treat_block);
            queue_pop(pretreat_queue);
            return GNC_OPERATOR_SUCCESS;
        }

    }

    // 每次最多处理 PRE_TREAT_SIZE
    for(int i = 0; i < PRE_TREAT_SIZE; i++){
        // 如果一致性队列已满，提前结束
        if(queue_is_full(treat_queue)) return GNC_DATA_FULL;
        // 如果预处理队列无数据，提前结束
        if(queue_get_size(pretreat_queue)<=1) return GNC_DATA_EMPTY;

        pretreat_block = queue_peak(pretreat_queue);
        if(pretreat_block == NULL) return GNC_DATA_NULL;
        treat_block = queue_peak_tail(treat_queue);
        if(treat_block == NULL) return GNC_DATA_NULL;
        memcpy(treat_block, pretreat_block, sizeof(PLAN_BLOCK));
        // TODO:处理异常点
        block_deal_outlier(pretreat_block, treat_block);
        
    }
    return GNC_CAL_SUCCESS;
}

/// @brief 一致性处理：nubrs、贝塞尔、hermit、圆弧过渡等平滑处理
GNC_RES unif_block()
{   
    GNC_RES res = GNC_CAL_SUCCESS;
    // 根据类型选择拟合方案
    switch (rt_data->chan_data[0].unif_type)
    {
        case UNIF_NURBS:
            res = block_fixturnning_point();
            res = block_unif_by_nurbs();               // nurbs拟合
            break;
        case UNIF_CIRCLE:
            res = block_unif_by_circle_smooth();
            break;
        default:
            res = block_unif_by_treat();         // 不做处理，treat传到unif队列
            break;

    }
    return res;
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
        // if(pretreat_block->line_number>=45)
        // {
        //     printf("ss\n");
        // }
        // 构造成段
        pretreat_block->max_vel = pretreat_block->feed_vel = parser_block->feed_vel;
        block_cal_pos(parser_block, pretreat_block, &pretreat_queue->pre_pos);
        // 计算长度
        block_cal_length(pretreat_block);
        // 计算速度
        block_cal_vel(pretreat_block);
        // 计算切矢
        block_cal_vector(pretreat_block);
        // 计算两段衔接夹角
        block_cal_angle(pretreat_block, &pretreat_queue->pre_vec);

        // 计算起点曲率
        block_cal_curvature(pretreat_block, &pretreat_queue->pre_startpos, &pretreat_queue->pre_pos,&pretreat_block->end_pos);
        // 保存信息
        pretreat_queue->pre_pos = pretreat_block->end_pos;
        pretreat_queue->pre_vec = pretreat_block->end_vec;
        pretreat_queue->pre_startpos = pretreat_block->start_pos;

        break;
    }
}

/// @brief 处理异常点
static void block_deal_outlier(PLAN_BLOCK* pretreat_block, PLAN_BLOCK* treat_block){
    switch (pretreat_block->mode)
    {
    // 处理空包和非移动指令
    case PMC_MODE_NULL:
    case PMC_MODE_M_CMD:
    case PMC_MODE_PAUSE:
        queue_pop(pretreat_queue);
        queue_soft_push(treat_queue);
        break;
    // 处理移动指令 
    default:
        // 长度小于阈值不放入队列
        if(!is_length_valid(pretreat_block)){
            queue_pop(pretreat_queue);
            // 更新下一段起点，起始方向，终点方向，长度
            PLAN_BLOCK* next_block;
            next_block=queue_peak(pretreat_queue);
            next_block->start_pos=pretreat_block->end_pos;
            next_block->start_vec= next_block->end_vec = VECTOR_Unit(&next_block->start_pos, &next_block->end_pos);
            block_cal_length(next_block);
        }else{
            queue_pop(pretreat_queue);
            queue_soft_push(treat_queue);

        }
        break;
    }
}

/// @brief nurbs拟合

static GNC_RES block_fixturnning_point()
{
    // 逆行点修复：
    PLAN_BLOCK *treat_block;
    PLAN_BLOCK *fix_block;
    PLAN_QUEUE *fix_queue_temp;
    fix_queue_temp = fix_queue;
    for (int i = 0; i < PRE_TREAT_SIZE; i++)
    {
        // 如果一致性队列已满，提前结束
        if (queue_is_full(fix_queue))
            return GNC_DATA_FULL;
        // 如果处理队列无数据，提前结束
        if (queue_is_empty(treat_queue))
            return GNC_DATA_EMPTY;
        
        treat_block = queue_peak(treat_queue);
        if(treat_block == NULL) return GNC_DATA_NULL;

        switch (treat_block->mode)
        {
        // 处理空包和非移动指令
        case PMC_MODE_NULL:
        case PMC_MODE_M_CMD:
        case PMC_MODE_PAUSE:
            fix_block = queue_peak_tail(fix_queue);
            if(fix_block == NULL) return GNC_DATA_NULL;
            memcpy(fix_block, treat_block, sizeof(PLAN_BLOCK));
            queue_pop(treat_queue);
            queue_soft_push(fix_queue);
            break;
        // 处理移动指令
        default:
            fix_block = queue_peak_tail(fix_queue);
            if(fix_block == NULL) return GNC_DATA_NULL;
            memcpy(fix_block, treat_block, sizeof(PLAN_BLOCK));
            // 判断是否分段
            block_cal_segment(fix_block, rt_data->chan_data[0].seg_theta_thre,fix_queue_temp);
            // 处理nurbs拟合
            block_deal_nurbs_enable(fix_block,fix_queue_temp);

            queue_pop(treat_queue);
            queue_soft_push(fix_queue);
            fix_queue->pre_pos = fix_block->end_pos;
            fix_queue->pre_vec = fix_block->end_vec;
            fix_queue->pre_startpos = fix_block->start_pos;
            blocknum++;
        }
    }
    return GNC_CAL_SUCCESS;
}

static GNC_RES block_unif_by_nurbs()
{
    PLAN_BLOCK *treat_block;

    // 未处理其译码结束时，nurbs队列还在等待新block
    // static int startflag = 1;

    if ((queue_is_empty(fix_queue)))
        return GNC_OPERATOR_SUCCESS;

    // 如果前瞻队列已满，提前结束
    if (queue_is_full(unif_queue))
        return GNC_OPERATOR_SUCCESS;
    
    //结束逻辑：
    if ((rt_data->chan_state[0][CHAN_IS_PARSERING] == false)&&queue_is_empty(treat_queue))
    {
        if(queue_get_size(fix_queue)==1)
        {
            treat_block = queue_peak(fix_queue);
            queue_push(unif_queue, treat_block);
            queue_pop(fix_queue);
            return GNC_OPERATOR_SUCCESS;

        }

    }

    treat_block = queue_peak(fix_queue);
    if (treat_block == NULL)
        return GNC_DATA_NULL;
    // if(queue_peak(fix_queue)->line_number>=141||queue_peak(fix_queue)->line_number==674 )
    // {
    //     printf("ss\n");
    // }
    switch (treat_block->mode)
    {
    // 处理空包和非移动指令
    case PMC_MODE_NULL:
    case PMC_MODE_M_CMD:
    case PMC_MODE_PAUSE:
        // 如果前瞻队列已满，提前结束
        if (queue_is_full(unif_queue))
            break;
        queue_push(unif_queue, treat_block);
        queue_pop(fix_queue);
        break;
    // 处理移动指令
    default:

        // 处理遗留未入队的nurbs队列， 防止treat队列满了导致两段nurbs和为一起。
        if (queue_peak(fix_queue)->type != NURBS)
        {
            if (!(queue_is_empty(nurbs_temp_queue)))
            {
                if (queue_peak(nurbs_temp_queue)->type == LINE)
                {
                    for (int i = 0; i < PRE_TREAT_SIZE; i++)
                    {
                        // 如果预处理队列无数据，提前结束
                        if (queue_is_empty(nurbs_temp_queue))
                            break;
                        // 如果前瞻队列已满，提前结束
                        if (queue_is_full(unif_queue))
                            break;
                        treat_block = queue_peak(nurbs_temp_queue);
                        treat_block->type = LINE;
                        queue_push(unif_queue, treat_block);
                        queue_pop(nurbs_temp_queue);
                    }
                    return GNC_OPERATOR_SUCCESS;
                }
                else
                {
                    treat_block = queue_peak(nurbs_temp_queue);
                    int nurbs_queue_size = queue_get_size(nurbs_temp_queue);
                    if (nurbs_queue_size >= segmentnum)
                    {
                        if (cal_nurbs_controlpoint(nurbs_temp_queue, treat_block, rt_data->chan_data[0].nurbs_fiterror, rt_data->chan_data[0].nurbs_sagittaerror))
                        {
                            LOG(LOG_LEVEL_INFO, "nurbs拟合/拟合失败，在%d段出现", treat_block->line_number);
                            return GNC_OPERATOR_SUCCESS;
                        }
                        // nurbs段重算速度与长度
                        treat_block->length = treat_block->nurbs_data.length;
                        double min_feed = MIN3(rt_data->axis_data[0].max_feed_vel, rt_data->axis_data[1].max_feed_vel, rt_data->axis_data[2].max_feed_vel);
                        double vn = vp_cal_nurbs_max_vel(treat_block, rt_data->chan_data[0].path_tolerance, rt_data->ncu_data.cycle);
                        treat_block->max_vel = treat_block->feed_vel = MIN3(treat_block->feed_vel, min_feed, vn);
                        // nurbs段重算切矢与上一段连接角度
                        block_cal_vector(treat_block);
                        block_cal_angle(treat_block, &queue_peak_pre_tail(unif_queue)->end_vec);  
                        // nurbs段重算与下一段连接角度
                        block_cal_angle(queue_peak(fix_queue), &treat_block->end_vec);  
                        queue_push(unif_queue, treat_block);
                        return GNC_OPERATOR_SUCCESS;
                    }
                    else
                    {
                        for (int i = 0; i < PRE_TREAT_SIZE; i++)
                        {
                            if (queue_is_empty(nurbs_temp_queue))
                                break;
                            treat_block = queue_peak(nurbs_temp_queue);
                            treat_block->type = LINE;
                            if (queue_is_full(unif_queue))
                                break;
                            queue_push(unif_queue, treat_block);
                            queue_pop(nurbs_temp_queue);
                        }
                        return GNC_OPERATOR_SUCCESS;
                    }
                }
            }
            else
            {
                // 拷贝 fix_queue 的数据
                for (int i = 0; i < PRE_TREAT_SIZE; i++)
                {
                    if (queue_is_full(unif_queue))
                        break;
                    if (queue_get_size(fix_queue) <= 1)
                        break;
                    treat_block = queue_peak(fix_queue);
                    if (treat_block->type == NURBS)
                    {
                        break;
                    }
                    queue_push(unif_queue, treat_block);
                    queue_pop(fix_queue);
                }
                return GNC_OPERATOR_SUCCESS;
            }
        }
        else
        {
            if (!(queue_is_empty(nurbs_temp_queue)))
            {
                if (queue_peak(nurbs_temp_queue)->type == LINE)
                {
                    for (int i = 0; i < PRE_TREAT_SIZE; i++)
                    {
                        // 如果预处理队列无数据，提前结束
                        if (queue_is_empty(nurbs_temp_queue))
                            break;
                        // 如果前瞻队列已满，提前结束
                        if (queue_is_full(unif_queue))
                            break;
                        treat_block = queue_peak(nurbs_temp_queue);
                        treat_block->type = LINE;
                        queue_push(unif_queue, treat_block);
                        queue_pop(nurbs_temp_queue);
                    }
                    return GNC_OPERATOR_SUCCESS;
                }
                else
                {
                    treat_block = queue_peak(nurbs_temp_queue);
                    int nurbs_queue_size = queue_get_size(nurbs_temp_queue);
                    if (nurbs_queue_size >= maxsegmentnum)
                    {
                        if (cal_nurbs_controlpoint(nurbs_temp_queue, treat_block, rt_data->chan_data[0].nurbs_fiterror, rt_data->chan_data[0].nurbs_sagittaerror))
                        {
                            LOG(LOG_LEVEL_INFO, "nurbs拟合/拟合失败，在%d段出现", treat_block->line_number);
                            return GNC_OPERATOR_SUCCESS;
                        }
                         // nurbs段重算速度与长度
                        treat_block->length = treat_block->nurbs_data.length;
                        double min_feed = MIN3(rt_data->axis_data[0].max_feed_vel, rt_data->axis_data[1].max_feed_vel, rt_data->axis_data[2].max_feed_vel);
                        double vn = vp_cal_nurbs_max_vel(treat_block, rt_data->chan_data[0].path_tolerance, rt_data->ncu_data.cycle);
                        treat_block->max_vel = treat_block->feed_vel = MIN3(treat_block->feed_vel, min_feed, vn);
                        // nurbs段重算切矢与上一段连接角度
                        block_cal_vector(treat_block);
                        block_cal_angle(treat_block, &queue_peak_pre_tail(unif_queue)->end_vec);  
                        // nurbs段重算与下一段连接角度
                        block_cal_angle(queue_peak(fix_queue), &treat_block->end_vec);  

                        queue_push(unif_queue, treat_block);
                        return GNC_OPERATOR_SUCCESS;
                    }
                }
            }
            for (int i = 0; i < PRE_TREAT_SIZE; i++)
            {
                if (queue_is_full(unif_queue))
                    break;
                if (queue_get_size(fix_queue) <= 1)
                    break;

                treat_block = queue_peak(fix_queue);

                // 到了不是nurbs队列的段，看nurbs缓存，大于分段点则拟合，否则将缓存转换为line
                if (treat_block->isContinue == 0)
                {
                    int nurbs_queue_size = queue_get_size(nurbs_temp_queue);
                    if (nurbs_queue_size >= segmentnum)
                    {
                        PLAN_BLOCK *nurbs_temp_block = queue_peak(nurbs_temp_queue);
                        if (cal_nurbs_controlpoint(nurbs_temp_queue, nurbs_temp_block, rt_data->chan_data[0].nurbs_fiterror, rt_data->chan_data[0].nurbs_sagittaerror))
                        {
                            LOG(LOG_LEVEL_INFO, "nurbs拟合/拟合失败，在%d段出现", nurbs_temp_block->line_number);
                            return GNC_OPERATOR_SUCCESS;
                        }
                         // nurbs段重算速度与长度
                        nurbs_temp_block->length = nurbs_temp_block->nurbs_data.length;
                        double min_feed = MIN3(rt_data->axis_data[0].max_feed_vel, rt_data->axis_data[1].max_feed_vel, rt_data->axis_data[2].max_feed_vel);
                        double vn = vp_cal_nurbs_max_vel(nurbs_temp_block, rt_data->chan_data[0].path_tolerance, rt_data->ncu_data.cycle);
                        nurbs_temp_block->max_vel = nurbs_temp_block->feed_vel = MIN3(nurbs_temp_block->feed_vel, min_feed, vn);
                        // nurbs段重算切矢与上一段连接角度
                        block_cal_vector(nurbs_temp_block);
                        block_cal_angle(nurbs_temp_block, &queue_peak_pre_tail(unif_queue)->end_vec);  
                        // nurbs段重算与下一段连接角度
                        block_cal_angle(queue_peak(fix_queue), &nurbs_temp_block->end_vec);  
                       
                        queue_push(unif_queue, nurbs_temp_block);
                        // 如果前瞻队列已满，提前结束
                        if (queue_is_full(unif_queue))
                            break;
                        queue_push(unif_queue, treat_block);
                        queue_pop(fix_queue);
                        break;
                    }
                    else
                    {
                        for (int i = 0; i < PRE_TREAT_SIZE; i++)
                        {
                            // 如果预处理队列无数据，提前结束
                            if (queue_is_empty(nurbs_temp_queue))
                                break;
                            treat_block = queue_peak(nurbs_temp_queue);
                            treat_block->type = LINE;
                            // 如果前瞻队列已满，提前结束
                            if (queue_is_full(unif_queue))
                                break;
                            queue_push(unif_queue, treat_block);
                            queue_pop(nurbs_temp_queue);
                        }
                        return GNC_OPERATOR_SUCCESS;
                    }
                }

                queue_push(nurbs_temp_queue, treat_block);
                queue_pop(fix_queue);
            }
            return GNC_OPERATOR_SUCCESS;
        }
    }
}

// 对相邻直线进行圆弧平滑。
// 只有译码结束且上游队列全部排空，才允许提交最后一段。
// 队列复位、删除及平滑模式切换，必须与本规划任务串行执行。
static GNC_RES block_unif_by_circle_smooth(void)
{
    CORNER_SMOOTH_CONFIG cfg;
    CORNER_SMOOTH_CONTROLLER_MAP map;

    // 初始化几何参数，并使用通道配置的路径允许偏差。
    circle_smooth_default_config(&cfg);
    circle_smooth_default_map(&map);
    cfg.path_tolerance = rt_data->chan_data[0].path_tolerance;

    for (int i = 0; i < PRE_TREAT_SIZE; ++i)
    {
        // 需同时检查两个上游队列。
        bool flush = !rt_data->chan_state[0][CHAN_IS_PARSERING] && queue_is_empty(parser_queue) && queue_is_empty(pretreat_queue);

        // 每次处理一对相邻段，仅进行几何平滑和队列传递。
        GNC_RES res = circle_smooth_queue_step(treat_queue, unif_queue, flush, &cfg, &map);
        if (res != GNC_OPERATOR_SUCCESS)
        {
            // 等待后继段、输出空间不足或参数错误时，保留未提交的数据。
            return res;
        }
    }

    return GNC_CAL_SUCCESS;
}

static GNC_RES block_unif_by_treat()
{
    PLAN_BLOCK* treat_block;
    PLAN_BLOCK* unif_block;

    for(int i = 0; i < PRE_TREAT_SIZE; i++){
        // 如果一致性队列已满，提前结束
        if(queue_is_full(unif_queue)) return GNC_DATA_FULL;
        // 如果处理队列无数据，提前结束
        if(queue_is_empty(treat_queue)) return GNC_DATA_EMPTY;

        treat_block = queue_peak(treat_queue);
        if(treat_block == NULL) return GNC_DATA_NULL;
        unif_block = queue_peak_tail(unif_queue);
        if(unif_block == NULL) return GNC_DATA_NULL;

        memcpy(unif_block, treat_block, sizeof(PLAN_BLOCK));
        queue_pop(treat_queue);
        queue_soft_push(unif_queue);
    }
    return GNC_CAL_SUCCESS;
}


static bool is_length_valid(PLAN_BLOCK* parser_block){
    if((parser_block->length != 0) && (parser_block->length <= rt_data->chan_data[0].l_min_thre))
    {
        return false;
    }
    return true;
} 

static void block_deal_nurbs_enable(PLAN_BLOCK* pretreat_block, PLAN_QUEUE *temp){
    if (rt_data->chan_data[0].nurbs_enable)
    {
        // 逆行点修复
        if(blocknum > 3){
            block_fix_pos(pretreat_block, rt_data->chan_data[0].l_theta_thre, rt_data->chan_data[0].theta_thre, temp);
        }
        // 修改为nurbs类型
        if(pretreat_block->isContinue && pretreat_block->type == LINE) pretreat_block->type = NURBS;  
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
        case NURBS:
            current_block->max_acc = MIN3(rt_data->axis_data[0].max_feed_acc, rt_data->axis_data[1].max_feed_acc, rt_data->axis_data[2].max_feed_acc);
            current_block->max_jerk = MIN3(rt_data->axis_data[0].max_feed_jk, rt_data->axis_data[1].max_feed_jk, rt_data->axis_data[2].max_feed_jk);
            // nurbs曲线最大速度计算
            double vn = vp_cal_nurbs_max_vel(current_block, rt_data->chan_data[0].path_tolerance, rt_data->ncu_data.cycle);
            current_block->max_vel = current_block->feed_vel = MIN3(current_block->feed_vel, min_feed, vn);
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
        case NURBS:
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

// 根据位置计算起点曲率
static void block_cal_curvature(PLAN_BLOCK* current_block, POS* pre_start_pos, POS* pre_end_pos, POS* current_pos)
{
    VECTOR AB = POS_VECTOR(pre_start_pos, pre_end_pos);
    VECTOR BC = POS_VECTOR(pre_end_pos, current_pos);
    VECTOR AC = POS_VECTOR(pre_start_pos, current_pos);
    double denominator = POS_Norm(AB)*POS_Norm(BC)*POS_Norm(AC);
    if (denominator < 1e-8)
    {
        current_block->curvature = 0;
    }
    else
    {
        double curvature = 2 * CrossVector(&AB, &BC) / denominator;
        current_block->curvature = curvature;
    }
}

//新增长度比
static int block_cal_segment(PLAN_BLOCK* pretreat_block, double seg_theta_thre,PLAN_QUEUE *temp){
    if ((pretreat_block->angle_rad>(seg_theta_thre*PI/180.0))||(pretreat_block->angle_rad==0))
    {   
        pretreat_block->isContinue=0;
        return 0;
    }
    if(blocknum>1)
    if (fabs((queue_peak_pre_tail(temp)->length)/pretreat_block->length)>20|| fabs((queue_peak_pre_tail(temp)->length)/pretreat_block->length)<0.05)
    {
        pretreat_block->isContinue=0;
        return 0;
    }
    if(pretreat_block->type==LINE)
    {    
        pretreat_block->isContinue=1;
        return 1;
    }
    else 
    {
        pretreat_block->isContinue=0;
        return 0;
    }
}

static void block_fix_pos(PLAN_BLOCK* pretreat_block, double l_theta_thre, double theta_thre, PLAN_QUEUE *temp){
    PLAN_BLOCK* last_pretreat_block=queue_peak_pre_tail(temp);
    PLAN_BLOCK* lastlast_pretreat_block=queue_peak_prepre_tail(temp);
    if((last_pretreat_block->angle_rad!=0)&&(pretreat_block->angle_rad!=0))
    if((last_pretreat_block->length<l_theta_thre)&&(last_pretreat_block->angle_rad>(theta_thre*PI/180))&&(pretreat_block->angle_rad>(theta_thre*PI/180)))
    {
        VECTOR v24= VECTOR_Unit(&last_pretreat_block->start_pos,&pretreat_block->end_pos);//24
        VECTOR v12= VECTOR_Unit(&lastlast_pretreat_block->start_pos,&last_pretreat_block->start_pos);//12
        double angle124=cal_two_vectors_angle(&v12,&v24);
        if(angle124<last_pretreat_block->angle_rad)
        {
            last_pretreat_block->end_pos.x=(pretreat_block->end_pos.x+ last_pretreat_block->start_pos.x)/2;
            last_pretreat_block->end_pos.y=(pretreat_block->end_pos.y+ last_pretreat_block->start_pos.y)/2;
            last_pretreat_block->end_pos.z=(pretreat_block->end_pos.z+ last_pretreat_block->start_pos.z)/2;
            pretreat_block->start_pos=last_pretreat_block->end_pos;
        }
        block_cal_vector(last_pretreat_block);
        block_cal_vector(pretreat_block);
        block_cal_length(last_pretreat_block);
        block_cal_length(pretreat_block);
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
        case HELIX_CCW:block_cal_circle_length(current_block);
            current_block->helical_data.screw_rad = atan2(current_block->helical_data.screw_offset, current_block->circular_data.rad * current_block->circular_data.radius);
            return sqrt(pow(current_block->circular_data.rad * current_block->circular_data.radius, 2) + pow(current_block->helical_data.screw_offset, 2));
        default:
            return 0;
    }
}

static double block_cal_nurbs_length(PLAN_BLOCK *current_block)
{
    if (current_block->nurbs_data.control_point != 0)
    {
        int num_samples = 1000;
        double u_vals[1000];
        for (int i = 0; i < num_samples; i++)
        {
            u_vals[i] = (double)i / (num_samples - 1);
        }
        current_block->length  = getlength(u_vals, num_samples,current_block->nurbs_data.control_point, 3, current_block->nurbs_data.U, current_block->nurbs_data.num,current_block->nurbs_data.w);
    }
    else
    {
        setAlarm(ALARM_CH, ALARM_ERROR, 2);
        LOG(LOG_LEVEL_ERR, "尚未进行nurbs拟合/拟合失败，在%d段出现", current_block->line_number);

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
        case NURBS:
            current_block->length = block_cal_nurbs_length(current_block);
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
            break;
        case NURBS:
            // TODO: 计算切矢
            POS start_vec, end_vec;
            nurbs_curve_derivative(1, current_block->nurbs_data.control_point, current_block->nurbs_data.w, current_block->nurbs_data.U,current_block->nurbs_data.num, current_block->nurbs_data.k, &end_vec);
            nurbs_curve_derivative(0, current_block->nurbs_data.control_point, current_block->nurbs_data.w, current_block->nurbs_data.U,current_block->nurbs_data.num, current_block->nurbs_data.k, &end_vec);
            POS_Divide(&end_vec, POS_Norm(end_vec));
            POS_Divide(&start_vec, POS_Norm(start_vec));
            current_block->end_vec = end_vec;
            current_block->start_vec = start_vec;
            break;
        default:
            break;
    }
}
