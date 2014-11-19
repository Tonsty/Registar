#ifndef EUCLIDEANCLUSTEREXTRACTION_H
#define EUCLIDEANCLUSTEREXTRACTION_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class EuclideanClusterExtraction
	{
	public:

		EuclideanClusterExtraction();
		virtual ~EuclideanClusterExtraction();

		//static void filter(CloudDataPtr cloudData, QVariantMap parameters, std::vector<CloudDataPtr> cloudData_filtereds);
		static void filter(CloudDataConstPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers);
		static void filter(CloudDataConstPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);

	};
}

#endif