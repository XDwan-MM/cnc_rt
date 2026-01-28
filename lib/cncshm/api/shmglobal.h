#ifndef SHM_GLOBAL_H
#define SHM_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SHM_LIBRARY)
#define SHM_EXPORT Q_DECL_EXPORT
#else
#define SHM_EXPORT Q_DECL_IMPORT
#endif

#endif // SHM_GLOBAL_H
