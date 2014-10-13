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

	struct Parameters
	{
		PairRegistration::Parameters pr_para;

		bool doInitialPairRegistration;
		bool doIncrementalLoopRefine;
		bool doGlobalRefine;
		unsigned int globalIterationNum_max;
		unsigned int globalIterationNum_min;
	} para;

	GlobalRegistration(ScanPtrs _scanPtrs = ScanPtrs(), Links _links = Links(), Loops _loops = Loops()) : scanPtrs(_scanPtrs), links(_links), loops(_loops) {}

	~GlobalRegistration() {}

	void setParameters(GlobalRegistration::Parameters _para){ para = _para; }

	void startRegistration();

	void initialTransformations();
	void buildKdTreePtrs();

	void initialPairRegistration();

	void initialGraph();
	void incrementalLoopRefine();

	float loopEstimateConsistencyError( GraphLoop* _graphLoop );
	void loopRefine( GraphLoop* _graphLoop );

	GraphEdge* createGraphEdge(GraphVertex *_vertex1, GraphVertex *_vertex2);
	Transformation GraphVertexDecompose(GraphVertex* currentVertex, Transformation lastTransformation, GraphVertex* lastVertex, 
		std::vector<GraphVertex*> &resultVertices, Transformations &resultTransformations, bool baseVertexOnly);
	
	void globalPairRefine();
	void globalRefine(unsigned int _iterationNum_max, unsigned int _iterationNum_min);

	ScanPtrs scanPtrs;
	Links links;
	Loops loops;

	KdTreePtrs kdTreePtrs;
	Transformations transformations;

	PairRegistrationPtrMap pairRegistrationPtrMap;

	Graph graph;
};

#endif
