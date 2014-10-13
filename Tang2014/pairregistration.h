#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include "common.h"
#include "scan.h"
#include "link.h"

#include <map>

class PairRegistration
{
public:

	enum MatchMethod
	{
		POINT_TO_POINT, POINT_TO_PLANE
	};

	enum SolveMethod
	{
		UMEYAMA, SVD
	};

	struct Parameters
	{
		MatchMethod mMethod;
		SolveMethod sMethod;
		bool distanceTest;
		float distThreshold;
		bool angleTest;
		float angleThreshold;
		bool boundaryTest;
		bool biDirection;
		unsigned int iterationNum_max;
		unsigned int iterationNum_min;
	} para;


	PairRegistration(ScanPtr _target, ScanPtr _source) : target(_target), source(_source) {}
	~PairRegistration() {}

	void startRegistration();

	struct PointPair
	{
		Point targetPoint;
		Point sourcePoint;
	};
	typedef std::vector< PointPair, Eigen::aligned_allocator<PointPair> > PointPairs;

	void generatePointPairs(PointsPtr _sbuffer, Transformation _transformation, PointPairs &_s2t);
	void generatePointPairs(PointsPtr _buffer, Transformation _transformation, PointPairs &_s2t, PointPairs &_t2s);

	void generateFinalPointPairs(Transformation _transformation);

	Transformation solveRegistration(PointPairs &_s2t, Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tgt);

	inline void setKdTree(KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree)
	{
		targetKdTree = _targetKdTree;
		sourceKdTree = _sourceKdTree;
	}

	inline void setParameter(PairRegistration::Parameters _para) { para = _para; }
	inline void setTransformation(Transformation _transformation) { transformation = _transformation; }

	ScanPtr target;
	ScanPtr source;

	Transformation transformation;

	KdTreePtr targetKdTree;
	KdTreePtr sourceKdTree;

	PointPairs final_s2t;
	static Transformation solveRegistration(PointPairs &_s2t, PairRegistration::SolveMethod _sMethod);

	typedef boost::shared_ptr<PairRegistration> Ptr;
};

typedef PairRegistration::Ptr PairRegistrationPtr;
typedef std::map<Link,PairRegistrationPtr,LinkComp> PairRegistrationPtrMap;

#endif

