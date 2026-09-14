#include "trace_publisher.h"

#include "global.h"
#include "rt_data.h"
#include <tracedata.h>

#include <string.h>

static uint64_t s_nextBlockId[SYS_CHAN_NUM];
static CncTraceBlock s_activeBlock[SYS_CHAN_NUM];
static int s_activeValid[SYS_CHAN_NUM];

static CncTraceSharedData *trace_shared(int channel)
{
    if (var_data == NULL || channel < 0 || channel >= SYS_CHAN_NUM)
    {
        return NULL;
    }
    return &var_data->trace_channel[channel];
}

static int trace_type(INTERP_TYPE type)
{
    switch (type)
    {
    case RAPID: return CNC_TRACE_RAPID;
    case LINE: return CNC_TRACE_LINE;
    case CIRCLE_CW: return CNC_TRACE_CIRCLE_CW;
    case CIRCLE_CCW: return CNC_TRACE_CIRCLE_CCW;
    case HELIX_CW: return CNC_TRACE_HELIX_CW;
    case HELIX_CCW: return CNC_TRACE_HELIX_CCW;
    case UNKNOWN: return CNC_TRACE_UNKNOWN;
    default: return CNC_TRACE_OTHER;
    }
}

static void copy_pos(double target[CNC_TRACE_AXIS_COUNT], const POS *source)
{
    target[0] = source->x;
    target[1] = source->y;
    target[2] = source->z;
}

static void invalidate_current(CncTraceSharedData *shared)
{
    uint32_t sequence = __atomic_load_n(&shared->current.sequence, __ATOMIC_RELAXED);
    sequence &= ~1U;
    __atomic_store_n(&shared->current.sequence, sequence + 1U, __ATOMIC_RELEASE);
    shared->current.valid = 0;
    __atomic_store_n(&shared->current.sequence, sequence + 2U, __ATOMIC_RELEASE);
}

void trace_publisher_begin_program(int channel)
{
    CncTraceSharedData *shared = trace_shared(channel);
    if (shared == NULL)
    {
        return;
    }

    s_nextBlockId[channel] = 0;
    s_activeValid[channel] = 0;
    invalidate_current(shared);
    __atomic_store_n(&shared->completedWriteSequence, 0, __ATOMIC_RELEASE);
    __atomic_add_fetch(&shared->generation, 1, __ATOMIC_RELEASE);
}

void trace_publisher_reset(int channel)
{
    CncTraceSharedData *shared = trace_shared(channel);
    if (shared == NULL)
    {
        return;
    }

    s_activeValid[channel] = 0;
    invalidate_current(shared);
}

void trace_publisher_activate(int channel, const PLAN_BLOCK *block, int isMotionBlock)
{
    CncTraceSharedData *shared = trace_shared(channel);
    if (shared == NULL)
    {
        return;
    }
    if (block == NULL || !isMotionBlock)
    {
        s_activeValid[channel] = 0;
        invalidate_current(shared);
        return;
    }

    CncTraceBlock *active = &s_activeBlock[channel];
    memset(active, 0, sizeof(*active));
    active->blockId = ++s_nextBlockId[channel];
    active->type = trace_type(block->type);
    active->lineNumber = block->line_number;
    active->toolNumber = var_chan[channel][VAR_CHAN_TOOL_USE].value.i;
    active->plane = block->circular_data.plane;
    active->direction = block->circular_data.dir;
    copy_pos(active->startPos, &block->start_pos);
    copy_pos(active->endPos, &block->end_pos);
    copy_pos(active->centerPos, &block->circular_data.center_pos);
    active->radius = block->circular_data.radius;
    active->startAngle = block->circular_data.statr_rad;
    active->sweepAngle = block->circular_data.rad;
    active->helixOffset = block->helical_data.screw_offset;
    s_activeValid[channel] = 1;

    trace_publisher_publish_current(channel);
}

void trace_publisher_publish_current(int channel)
{
    CncTraceSharedData *shared = trace_shared(channel);
    if (shared == NULL || !s_activeValid[channel])
    {
        return;
    }

    uint32_t sequence = __atomic_load_n(&shared->current.sequence, __ATOMIC_RELAXED);
    sequence &= ~1U;
    __atomic_store_n(&shared->current.sequence, sequence + 1U, __ATOMIC_RELEASE);
    shared->current.valid = 1;
    shared->current.block = s_activeBlock[channel];
    shared->current.currentPos[0] = rt_data->axis_data[AXIS_X].wcs_cmd_pos_cur;
    shared->current.currentPos[1] = rt_data->axis_data[AXIS_Y].wcs_cmd_pos_cur;
    shared->current.currentPos[2] = rt_data->axis_data[AXIS_Z].wcs_cmd_pos_cur;
    __atomic_store_n(&shared->current.sequence, sequence + 2U, __ATOMIC_RELEASE);
}

void trace_publisher_complete(int channel)
{
    CncTraceSharedData *shared = trace_shared(channel);
    if (shared == NULL || !s_activeValid[channel])
    {
        return;
    }

    const uint64_t publishSequence =
        __atomic_load_n(&shared->completedWriteSequence, __ATOMIC_RELAXED) + 1;
    CncTraceCompletedSlot *slot =
        &shared->completed[(publishSequence - 1) % CNC_TRACE_QUEUE_CAPACITY];

    __atomic_store_n(&slot->publishSequence, 0, __ATOMIC_RELEASE);
    slot->block = s_activeBlock[channel];
    __atomic_store_n(&slot->publishSequence, publishSequence, __ATOMIC_RELEASE);
    __atomic_store_n(&shared->completedWriteSequence, publishSequence, __ATOMIC_RELEASE);
    s_activeValid[channel] = 0;
    invalidate_current(shared);
}
