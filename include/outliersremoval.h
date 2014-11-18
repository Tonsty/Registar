#ifndef OUTLIERSREMOVAL_H
#define OUTLIERSREMOVAL_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class OutliersRemoval
	{
	public:

		OutliersRemoval();
		virtual ~OutliersRemoval();

		static void filter(CloudDataPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers);
		static void filter(CloudDataPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);
	};
}

#endif