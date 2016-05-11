#ifndef TANG2014_H
#define TANG2014_H

#include <QtCore/QVariantMap>

#include "Tang2014/globalregistration.h"

namespace registar {

	class Tang2014 
	{
	public:
		static tang2014::Transformations startRegistration(tang2014::ScanPtrs scanPtrs, QVariantMap parameters);
	};

};

#endif