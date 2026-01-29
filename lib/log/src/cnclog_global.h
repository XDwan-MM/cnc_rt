#ifndef CNCLOG_GLOBAL_H
#define CNCLOG_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CNCLOGMAIN_LIBRARY)
#  define CNCLOGMAIN_EXPORT Q_DECL_EXPORT
#else
#  define CNCLOGMAIN_EXPORT Q_DECL_IMPORT
#endif

#endif // CNCLOG_GLOBAL_H
