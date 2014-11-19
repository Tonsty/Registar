#ifndef MOVINGLEASTSQUARES_H
#define MOVINGLEASTSQUARES_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class MovingLeastSquares
	{

	public:
		MovingLeastSquares();
		virtual ~MovingLeastSquares();

		static void filter(CloudDataConstPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);
	};
}

#endif
