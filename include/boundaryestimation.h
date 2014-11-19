#ifndef BOUNDARYESTIMATION_H
#define BOUNDARYESTIMATION_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class BoundaryEstimation
	{
	public:

		BoundaryEstimation();
		virtual ~BoundaryEstimation();

		static void filter(CloudDataConstPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers);
		static void filter(CloudDataConstPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_filtered);
	};
}

#endif