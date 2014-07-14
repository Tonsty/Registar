#ifndef CLOUDIO_H
#define CLOUDIO_H

#include <QtCore/QObject>

#include "pclbase.h"

class CloudIO
{
public:
	CloudIO();
	virtual ~CloudIO();

	static bool importCloudData(const QString &fileName, CloudDataPtr &cloudData);
	static bool exportCloudData(const QString &fileName, CloudDataPtr &cloudData);

	static bool importTransformation(const QString &fileName, Eigen::Matrix4f &transformation);
	static bool exportTransformation(const QString &fileName, Eigen::Matrix4f &transformation);

	static bool importBoundaries(const QString &fileName, BoundariesPtr &boundaries);
	static bool exportBoundaries(const QString &fileName, BoundariesPtr &boundaries);
};

#endif
