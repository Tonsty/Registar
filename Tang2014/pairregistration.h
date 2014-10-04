#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include "common.h"
#include "scan.h"
#include "link.h"

#include <map>

class PairRegistration
{
public:
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

	Transformation solveRegistration(PointPairs &_s2t, Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tgt);

	inline void setKdTree(KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree)
	{
		targetKdTree = _targetKdTree;
		sourceKdTree = _sourceKdTree;
	}

	enum MatchMethod
	{
		POINT_TO_POINT, POINT_TO_PLANE
	};
	MatchMethod mMethod;
	enum SolveMethod
	{
		UMEYAMA, SVD
	};
	SolveMethod sMethod;

	inline void setParameter(PairRegistration::MatchMethod _mMethod, PairRegistration::SolveMethod _sMethod, Transformation _transformation,
		bool _distanceTest, float _distanceThreshold, bool _angleTest, float _angleThreshold, bool _boundaryTest, bool _biDirection, unsigned int _interationNum)
	{
		mMethod = _mMethod;
		sMethod = _sMethod;
		transformation = _transformation;
		distanceTest = _distanceTest;
		distThreshold = _distanceThreshold;
		angleTest = _angleTest;
		angleThreshold = _angleThreshold;
		boundaryTest = _boundaryTest;
		biDirection = _biDirection;
		interationNum = _interationNum;
	}

	ScanPtr target;
	ScanPtr source;

	Transformation transformation;

	KdTreePtr targetKdTree;
	KdTreePtr sourceKdTree;

	bool distanceTest;
	float distThreshold;
	bool angleTest;
	float angleThreshold;
	bool boundaryTest;
	bool biDirection;
	unsigned int interationNum;

	typedef boost::shared_ptr<PairRegistration> Ptr;
};

typedef PairRegistration::Ptr PairRegistrationPtr;
typedef std::map<Link,PairRegistrationPtr,LinkComp> PairRegistrationPtrMap;

#endif

