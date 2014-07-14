#ifndef BOUNDARYESTIMATION_H
#define BOUNDARYESTIMATION_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

class BoundaryEstimation
{
public:

	BoundaryEstimation();
	virtual ~BoundaryEstimation();

	static void filter(CloudDataPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers);
	static void filter(CloudDataPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_filtered);
};

#endif