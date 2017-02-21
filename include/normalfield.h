#ifndef NORMALFILED_H
#define NORMALFILED_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class NormalField
	{
	public:
		NormalField();
		virtual ~NormalField();

		static void filter(CloudDataConstPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);
	};
}


#endif