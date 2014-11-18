#ifndef QTBASE_H
#define QTBASE_H

#include<QtCore/QMetaType>

#include "pclbase.h"

namespace registar
{
	Q_DECLARE_METATYPE(CloudDataPtr);
	Q_DECLARE_METATYPE(CloudDataConstPtr);

	Q_DECLARE_METATYPE(QList<bool>);
	Q_DECLARE_METATYPE(Eigen::Matrix4f);
}

#endif
