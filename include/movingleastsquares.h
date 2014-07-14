#ifndef MOVINGLEASTSQUARES_H
#define MOVINGLEASTSQUARES_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

class MovingLeastSquares
{
	
public:
	MovingLeastSquares();
	virtual ~MovingLeastSquares();

	static void filter(CloudDataPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);
};

#endif
