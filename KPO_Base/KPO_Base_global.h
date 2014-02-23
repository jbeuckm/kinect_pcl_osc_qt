#ifndef KPO_BASE_GLOBAL_H
#define KPO_BASE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(KPO_BASE_LIBRARY)
#  define KPO_BASESHARED_EXPORT Q_DECL_EXPORT
#else
#  define KPO_BASESHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // KPO_BASE_GLOBAL_H
