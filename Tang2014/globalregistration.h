#ifndef GLOBALREGISTRATION_H
#define GLOBALREGISTRATION_H

#include "common.h"
#include "link.h"
#include "loop.h"
#include "scan.h"
#include "pairregistration.h"

class GlobalRegistration
{
public:
	GlobalRegistration(ScanPtrs _scanPtrs, Links _links, Loops _loops) : scanPtrs(_scanPtrs), links(_links), loops(_loops) {}
	~GlobalRegistration() {}

	void startRegistration();

	void initialTransformations();
	void buildKdTreePtrs();
	void initialPairRegistration();
	void incrementalLoopRefine();
	void globalPairRefine();
	void globalRefine();

	ScanPtrs scanPtrs;
	Links links;
	Loops loops;

	KdTreePtrs kdTreePtrs;
	Transformations transformations;

	PairRegistrationPtrMap pairRegistrationPtrMap;
};

#endif
