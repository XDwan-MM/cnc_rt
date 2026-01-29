#include "shmapi.h"
#include "shmmanager.h"

bool shmInit() {
    return ShmManager::instance().init();
}
