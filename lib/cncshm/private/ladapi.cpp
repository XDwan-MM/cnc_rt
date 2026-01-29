#include "ladapi.h"
#include "ladmanager.h"
#include "shmmanager.h"

LadData* getLadDataPtr() {
    return ShmManager::instance().ladManager()->getLadDataPtr();
}
