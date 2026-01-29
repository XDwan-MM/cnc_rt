#include "alarmmanager.h"
#include "semaphorelocker.h"
#include <QDebug>

///////////////////////////////////////////////////////////////////

/**
 * 初始化循环队列
 * @param queue 指向循环队列的指针
 */
void initQueue(AlarmHisDatas *queue) {
    if (queue == NULL) return;
    queue->front = 0;
    queue->rear = 0;
}

/**
 * 判断队列是否为空
 * @param queue 循环队列
 * @return 空返回true，非空返回false
 */
bool isEmpty(AlarmHisDatas *queue) {
    if (queue == NULL) return true;
    return queue->front == queue->rear;
}

/**
 * 判断队列是否已满
 * @param queue 循环队列
 * @return 满返回true，未满返回false
 */
bool isFull(AlarmHisDatas *queue) {
    if (queue == NULL) return true;
    return (queue->rear + 1) % ALARM_HIS_COUNT_MAX == queue->front;
}

/**
 * 入队操作
 * @param queue 指向循环队列的指针
 * @param value 要入队的元素值
 * @return 入队成功返回true，失败（队列满）返回false
 */
bool enqueue(AlarmHisDatas *queue, AlarmHisDataDef value) {
    if (queue == NULL || isFull(queue)) {
        printf("队列已满，无法入队：%d\n", value);
        return false;
    }
    queue->datas[queue->rear] = value;          // 元素放入队尾位置
    queue->rear = (queue->rear + 1) % ALARM_HIS_COUNT_MAX;  // 队尾下标循环后移
//    printf("入队成功：%d\n", value.id);
    return true;
}

/**
 * 出队操作
 * @param queue 指向循环队列的指针
 * @param outValue 用于接收出队元素的指针（出队失败时值无意义）
 * @return 出队成功返回true，失败（队列空）返回false
 */
bool dequeue(AlarmHisDatas *queue, AlarmHisDataDef *outValue) {
    if (queue == NULL || isEmpty(queue) || outValue == NULL) {
        printf("队列为空，无法出队\n");
        return false;
    }
    *outValue = queue->datas[queue->front];     // 取出队头元素
    queue->front = (queue->front + 1) % ALARM_HIS_COUNT_MAX;  // 队头下标循环后移
//    printf("出队成功：%d\n", *outValue->id);
    return true;
}

/**
 * 获取队头元素（不出队）
 * @param queue 循环队列
 * @param outValue 用于接收队头元素的指针
 * @return 获取成功返回true，失败返回false
 */
bool getFront(AlarmHisDatas *queue, AlarmHisDataDef *outValue) {
    if (queue == NULL || isEmpty(queue) || outValue == NULL) {
        printf("队列为空，无队头元素\n");
        return false;
    }
    *outValue = queue->datas[queue->front];
    return true;
}

/**
 * 获取队列当前元素个数
 * @param queue 循环队列
 * @return 元素个数（队列异常返回0）
 */
int getSize(AlarmHisDatas *queue) {
    if (queue == NULL) return 0;
    // 循环计算长度：(队尾 - 队头 + 总容量) % 总容量
    return (queue->rear - queue->front + ALARM_HIS_COUNT_MAX) % ALARM_HIS_COUNT_MAX;
}

/**
 * 打印队列中所有元素（从队头到队尾）
 * @param queue 循环队列
 */
void printQueue(AlarmHisDatas *queue) {
    if (queue == NULL || isEmpty(queue)) {
        printf("队列为空\n");
        return;
    }
    printf("队列元素（队头→队尾）：");
    int i = queue->front;
    while (i != queue->rear) {
        printf("%d ", queue->datas[i]);
        i = (i + 1) % ALARM_HIS_COUNT_MAX;
    }
    printf("\n");
}


///////////////////////////////////////////////////////////////////

AlarmManager::AlarmManager(AlarmDataBuffer &buffer)
    : m_alms(buffer.alms)
    , m_hiss(buffer.hiss)
    , _sem("AlarmDataSem", 1, QSystemSemaphore::Open)
{
    for (int i = 0; i < ALARM_COUNT_MAX; ++i)
    {
        m_emptyAlms.datas[i].id = -1;
        m_emptyAlms.datas[i].type = -1;
        m_emptyAlms.datas[i].level = -1;
    }
    m_emptyAlms.count = 0;
    m_alms = m_emptyAlms;

    m_hiss.front = 0;
    m_hiss.rear = 0;
}

AlarmManager::~AlarmManager() {
}

int AlarmManager::setAlarm(int type, int level, int id)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type >= ALARM_TYPE_ALL)
    {
        return ALARM_MANAGER_TYPE_FAILED;
    }
    if (level < 0 || level >= ALARM_LEVEL_ALL)
    {
        return ALARM_MANAGER_LEVEL_FAILED;
    }
    SemaphoreLocker locker(_sem);

    if (m_alms.count >= ALARM_COUNT_MAX)
    {
        return ALARM_MANAGER_COUNT_MAX_FAILED;
    }
    for (int i = 0; i < m_alms.count; ++i)
    {
        if (m_alms.datas[i].type == type && m_alms.datas[i].level == level &&m_alms.datas[i].id == id)
        {
            return ALARM_MANAGER_REPEAT_ERROR;
        }
    }

    m_alms.datas[m_alms.count].id = id;
    m_alms.datas[m_alms.count].type = (int)type;
    m_alms.datas[m_alms.count].level = (int)level;
    m_alms.datas[m_alms.count].start = time(NULL);
    m_alms.count++;

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::delAlarm(int type, int level, int id)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type >= ALARM_TYPE_ALL)
    {
        return ALARM_MANAGER_TYPE_FAILED;
    }
    if (level < 0 || level >= ALARM_LEVEL_ALL)
    {
        return ALARM_MANAGER_LEVEL_FAILED;
    }
    SemaphoreLocker locker(_sem);

    int idx = -1;
    // 获取目标索引
    for (int i = 0; i < m_alms.count; ++i)
    {
        if (m_alms.datas[i].type == type && m_alms.datas[i].level == level
            && m_alms.datas[i].id == id)
        {
            idx = i;

            AlarmHisDataDef his;
            his.id = m_alms.datas[i].id;
            his.type = m_alms.datas[i].type;
            his.level = m_alms.datas[i].level;
            his.start = m_alms.datas[i].start;
            his.end = time(NULL);
            enqueue(&m_hiss, his);
        }
    }
    if (idx < 0)
    {
        return ALARM_MANAGER_NOT_FIND_TARGET;
    }

    // 移动值
    for (int i = idx; i < m_alms.count; ++i)
    {
        if (i == m_alms.count - 1)
        {
            m_alms.datas[i] = {-1};
        }
        else
        {
            m_alms.datas[i] = m_alms.datas[i + 1];
        }
    }
    m_alms.count--;

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::cleanAllAlarm(int type, int level)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type > ALARM_TYPE_ALL)
    {
        return ALARM_MANAGER_TYPE_FAILED;
    }
    if (level < 0 || level > ALARM_LEVEL_ALL)
    {
        return ALARM_MANAGER_LEVEL_FAILED;
    }
    SemaphoreLocker locker(_sem);

    // 消除指定报警
    if (type == int(ALARM_TYPE_ALL) && level == int(ALARM_LEVEL_ALL))
    {
        for (int i = 0; i < ALARM_COUNT_MAX; ++i)
        {
            if (m_alms.datas[i].id < 0)
            {
                break;
            }
            emptyData(m_alms.datas[i]);
        }
        m_alms.count = 0;
    }
    else if (type == int(ALARM_TYPE_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].id < 0)
            {
                break;
            }
            if (m_alms.datas[i].level == level)
            {
                emptyData(m_alms.datas[i]);
            }
        }
    }
    else if (level == int(ALARM_LEVEL_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].id < 0)
            {
                break;
            }
            if (m_alms.datas[i].type == type)
            {
                emptyData(m_alms.datas[i]);
            }
        }
    }
    else
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].id < 0)
            {
                break;
            }
            if (m_alms.datas[i].type == type && m_alms.datas[i].level == level)
            {
                emptyData(m_alms.datas[i]);
            }
        }
    }

    // 重新排列
    AlarmDatas emptyAlms = m_emptyAlms;
    for (int i = 0; i < ALARM_COUNT_MAX; ++i)
    {
        if (m_alms.datas[i].id >= 0)
        {
            emptyAlms.datas[emptyAlms.count] = m_alms.datas[i];
            ++emptyAlms.count;
        }
    }
    m_alms = emptyAlms;

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::getAlarmCount(int type, int level, int *count)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type > ALARM_TYPE_ALL)
    {
        return ALARM_MANAGER_TYPE_FAILED;
    }
    if (level < 0 || level > ALARM_LEVEL_ALL)
    {
        return ALARM_MANAGER_LEVEL_FAILED;
    }
    if (count == nullptr)
    {
        return ALARM_MANAGER_NULL_POINTER;
    }
    SemaphoreLocker locker(_sem);

    int num = 0;
    if (type == int(ALARM_TYPE_ALL) && level == int(ALARM_LEVEL_ALL))
    {
        num = m_alms.count;
    }
    else if (type == int(ALARM_TYPE_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].level == level)
            {
                ++num;
            }
        }
    }
    else if (level == int(ALARM_LEVEL_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].type == type)
            {
                ++num;
            }
        }
    }
    else
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].type == type && m_alms.datas[i].level == level)
            {
                ++num;
            }
        }
    }
    *count = num;

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::getAlarmId(int type, int level, int index, int *id)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    if (type < 0 || type > ALARM_TYPE_ALL)
    {
        return ALARM_MANAGER_TYPE_FAILED;
    }
    if (level < 0 || level > ALARM_LEVEL_ALL)
    {
        return ALARM_MANAGER_LEVEL_FAILED;
    }
    SemaphoreLocker locker(_sem);

    if (index < 0 || index >= m_alms.count)
    {
        return ALARM_MANAGER_INDEX_ERROR;
    }

    *id = -2;
    if (type == int(ALARM_TYPE_ALL) && level == int(ALARM_LEVEL_ALL))
    {
        *id = m_alms.datas[index].id;
    }
    else if (type == int(ALARM_TYPE_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].level == level)
            {
                --index;
            }
            if (index < 0)
            {
                *id = m_alms.datas[i].id;
                break;
            }
        }
        if (index >= 0)
        {
            return ALARM_MANAGER_NOT_FIND_TARGET;
        }
    }
    else if (level == int(ALARM_LEVEL_ALL))
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].type == type)
            {
                --index;
            }
            if (index < 0)
            {
                *id = m_alms.datas[i].id;
                break;
            }
        }
        if (index >= 0)
        {
            return ALARM_MANAGER_NOT_FIND_TARGET;
        }
    }
    else
    {
        for (int i = 0; i < m_alms.count; ++i)
        {
            if (m_alms.datas[i].type == type && m_alms.datas[i].level == level)
            {
                --index;
            }
            if (index < 0)
            {
                *id = m_alms.datas[i].id;
                break;
            }
        }
        if (index >= 0)
        {
            return ALARM_MANAGER_NOT_FIND_TARGET;
        }
    }

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::getAlarmData(int index, int *id, int *type, int *level, time_t *start)
{
    if (!_isShmInit)
    {
        return ALARM_MANAGER_SHM_INIT_FAILED;
    }
    SemaphoreLocker locker(_sem);

    if (index < 0 || index >= m_alms.count)
    {
        return ALARM_MANAGER_INDEX_ERROR;
    }

    *id = m_alms.datas[index].id;
    *type = m_alms.datas[index].type;
    *level = m_alms.datas[index].level;
    *start = m_alms.datas[index].start;

    return ALARM_MANAGER_SUCCESS;
}

int AlarmManager::getAlarmHisCount(int *count)
{
    *count = getSize(&m_hiss);
    return 0;
}

int AlarmManager::takeAlarmHisFront(int *id, int *type, int *level, time_t *start, time_t *end)
{
    AlarmHisDataDef his;
    bool tr = dequeue(&m_hiss, &his);
    if (!tr)
    {
        return -1;
    }
    *id = his.id;
    *type = his.type;
    *level = his.level;
    *start = his.start;
    *end = his.end;
    return 0;
}

void AlarmManager::emptyData(AlarmDataDef &data)
{
    AlarmHisDataDef his;
    his.id = data.id;
    his.type = data.type;
    his.level = data.level;
    his.start = data.start;
    his.end = time(NULL);
    enqueue(&m_hiss, his);

    data.id = -1;
    data.type = -1;
    data.level = -1;
}
