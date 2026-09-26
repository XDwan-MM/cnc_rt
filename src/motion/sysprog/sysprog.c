/********************************************************************
 * @file        sysprog.c
 * @brief       系统程序：登记 + 就绪 + 独立缓冲 + 生命周期收口（实现）
 ********************************************************************/

#include "sysprog.h"
#include "global.h"
#include <stdio.h>
#include <string.h>

/*! 内置生产者。**只在这里登记**，避免每个场景都要去找一处 init 挂钩：
 *  新增场景 = 生产者文件里导出描述符 + 下面 GNC_RES sysprog_init() 里加一行。 */
extern const SYS_PROG_DESC tapreturn_sysprog_desc;      // src/motion/rerun/tapreturn.c

#define SYSPROG_DESC_MAX   (8)

static const SYS_PROG_DESC *g_descs[SYSPROG_DESC_MAX];
static int g_desc_count = 0;

static char g_content[SYSPROG_CONTENT_MAX];
static SYS_PROG_ID g_id = SYS_PROG_NONE;
static SYS_PROG_STATE g_state = SYSPROG_INACTIVE;

static void sysprog_reset(void);

static const SYS_PROG_DESC *find_desc(SYS_PROG_ID id)
{
    for (int i = 0; i < g_desc_count; ++i)
    {
        if (g_descs[i] != NULL && g_descs[i]->id == id) return g_descs[i];
    }
    return NULL;
}

static const char *name_of(SYS_PROG_ID id)
{
    const SYS_PROG_DESC *desc = find_desc(id);
    return (desc != NULL) ? desc->name : "未登记";
}

GNC_RES sysprog_init(void)
{
    // 幂等：重复 init 时先清空登记表，否则会因"同一 id 重复登记"而失败
    for (int i = 0; i < SYSPROG_DESC_MAX; ++i) g_descs[i] = NULL;
    g_desc_count = 0;
    sysprog_reset();

    if (!sysprog_register(&tapreturn_sysprog_desc))
    {
        LOG(LOG_LEVEL_ERR, "系统程序登记失败（登记表或描述符异常）");
        return GNC_DATA_FULL;
    }
    return GNC_OPERATOR_SUCCESS;
}

bool sysprog_register(const SYS_PROG_DESC *desc)
{
    if (desc == NULL || desc->id == SYS_PROG_NONE || desc->build == NULL) return false;
    // 先到先得：同 id 重复登记不静默覆盖（覆盖会让"谁在跑"变得不可追）
    if (find_desc(desc->id) != NULL) return false;
    if (g_desc_count >= SYSPROG_DESC_MAX) return false;
    g_descs[g_desc_count++] = desc;
    return true;
}

bool sysprog_arm(SYS_PROG_ID id)
{
    const SYS_PROG_DESC *desc = find_desc(id);
    if (desc == NULL)
    {
        LOG(LOG_LEVEL_ERR, "系统程序%d未登记，拒绝就绪", (int)id);
        return false;
    }

    if (g_state != SYSPROG_INACTIVE)
    {
        LOG(LOG_LEVEL_WARN, "已有系统程序[%s]就绪/进行中，拒绝[%s]",
            name_of(g_id), desc->name);
        return false;
    }
    if (desc->guard != NULL && !desc->guard())
    {
        LOG(LOG_LEVEL_WARN, "系统程序[%s]前置条件不满足，拒绝就绪", desc->name);
        return false;
    }
    // 顺序不可换：参数必须早于**内容被译码器取走**。内容会随 MDI 启动被 parser 取用，
    // 而译码器有预读缓冲 —— 设晚了只表现为莫名其妙的译码报警（见 tapreturn.h 的说明）。
    if (desc->prepare != NULL) desc->prepare();
    if (!desc->build(g_content, (int)sizeof(g_content)))
    {
        // 生成失败 → 回滚 prepare：绝不留下"参数已就绪但没内容"的中间态
        if (desc->cleanup != NULL) desc->cleanup();
        g_content[0] = '\0';
        LOG(LOG_LEVEL_ERR, "系统程序[%s]生成失败，已回滚", desc->name);
        return false;
    }
    g_id = id;
    g_state = SYSPROG_ARMED;
    LOG(LOG_LEVEL_INFO, "系统程序[%s]已就绪: %s", desc->name, g_content);
    return true;
}

void sysprog_cleanup(void)
{

    const SYS_PROG_DESC *desc = find_desc(g_id);
    if (desc != NULL && desc->cleanup != NULL) desc->cleanup();
    if (g_state != SYSPROG_INACTIVE)
    {
        LOG(LOG_LEVEL_INFO, "系统程序[%s]已收口", desc != NULL ? desc->name : "未登记");
    }
    g_state = SYSPROG_INACTIVE;
    g_id = SYS_PROG_NONE;
    g_content[0] = '\0';
}

void sysprog_mark_consumed(void)
{
    // 已运行过 ≠ 结束：内容仍可取用（与"MDI 缓冲里那一行还在"的既有行为一致），
    // 状态只用于 HMI 回显与后续按场景的完成判据。
    if (g_state == SYSPROG_ARMED) g_state = SYSPROG_ACTIVE;
}

bool sysprog_content_ready(void)
{
    return (g_state != SYSPROG_INACTIVE) && (g_content[0] != '\0');
}

bool sysprog_get_content(char *buf, int buf_len)
{
    if (buf == NULL || buf_len <= 0) return false;
    if (!sysprog_content_ready()) return false;
    size_t len = strlen(g_content);
    if ((int)(len + 1) > buf_len) return false;
    memcpy(buf, g_content, len + 1);
    return true;
}

bool sysprog_is_armed(void)
{
    return (g_state == SYSPROG_ARMED);
}

bool sysprog_active(SYS_PROG_ID id)
{
    return (g_state != SYSPROG_INACTIVE) && (g_id == id);
}

SYS_PROG_ID sysprog_running_id(void)
{
    return (g_state != SYSPROG_INACTIVE) ? g_id : SYS_PROG_NONE;
}

SYS_PROG_STATE sysprog_state(void)
{
    return g_state;
}

void sysprog_reset(void)
{
    // 生产路径请用 sysprog_cleanup()：这里**不调** desc->cleanup()，
    // 会把 prepare 留下的参数变成残留。
    g_id = SYS_PROG_NONE;
    g_state = SYSPROG_INACTIVE;
    g_content[0] = '\0';
}
