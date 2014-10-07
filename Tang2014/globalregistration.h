#ifndef GLOBALREGISTRATION_H
#define GLOBALREGISTRATION_H

#include "common.h"
#include "link.h"
#include "loop.h"
#include "scan.h"
#include "pairregistration.h"
#include "graph.h"

class GlobalRegistration
{
public:
	GlobalRegistration(ScanPtrs _scanPtrs, Links _links, Loops _loops) : scanPtrs(_scanPtrs), links(_links), loops(_loops) {}
	~GlobalRegistration() {}

	void startRegistration();

	void initialTransformations();
	void buildKdTreePtrs();
	void initialPairRegistration();

	void initialGraph();
	void incrementalLoopRefine();

	float loopEstimateConsistencyError( GraphLoop* _graphLoop );
	void loopRefine( GraphLoop* _graphLoop );
	
	void globalPairRefine();
	void globalRefine(unsigned int _interationNum);

	ScanPtrs scanPtrs;
	Links links;
	Loops loops;

	KdTreePtrs kdTreePtrs;
	Transformations transformations;

	PairRegistrationPtrMap pairRegistrationPtrMap;

	Graph graph;
};

#endif
