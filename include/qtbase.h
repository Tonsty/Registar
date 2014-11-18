#ifndef QTBASE_H
#define QTBASE_H

#include<QtCore/QMetaType>

#include "pclbase.h"

Q_DECLARE_METATYPE(registar::CloudDataPtr);
Q_DECLARE_METATYPE(registar::CloudDataConstPtr);

Q_DECLARE_METATYPE(QList<bool>);
Q_DECLARE_METATYPE(Eigen::Matrix4f);

#endif
