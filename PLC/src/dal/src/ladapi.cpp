#include "ladapi.h"
#include "ladmanager.h"
#include "shmmanager.h"

// LadRow* getLadRowPtr(int row) {
//     return ShmManager::instance().ladManager()->getLadRowPtr(row);
// }

LadData* getLadDataPtr() {
    return ShmManager::instance().ladManager()->getLadDataPtr();
}
