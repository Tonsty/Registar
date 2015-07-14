#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include "common.h"
#include "scan.h"
#include "link.h"

#include <map>

namespace Tang2014
{
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

		static void generatePointPairs(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, 
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _sbuffer, Transformation _transformation, PairRegistration::Parameters _para, PointPairs &_s2t);

		static void generatePointPairs(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree, 
								std::vector<int> &_targetCandidateIndices, std::vector<int> &_targetCandidateIndices_temp,
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _buffer, Transformation _transformation, PairRegistration::Parameters _para, PointPairs &_s2t, PointPairs &_t2s);

		virtual void generateFinalPointPairs(Transformation _transformation);

		Transformation solveRegistration(PointPairs &_s2t, Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tgt);

		inline void setKdTree(KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree)
		{
			targetKdTree = _targetKdTree;
			sourceKdTree = _sourceKdTree;
		}

		inline void setParameter(const PairRegistration::Parameters &_para) { para = _para; }
		inline void setTransformation(const Transformation &_transformation) { transformation = _transformation; }

		ScanPtr target;
		ScanPtr source;

		Transformation transformation;

		KdTreePtr targetKdTree;
		KdTreePtr sourceKdTree;

		void initiateCandidateIndices();
		std::vector<int> targetCandidateIndices;
		std::vector<int> targetCandidateIndices_temp;
		std::vector<int> sourceCandidateIndices;
		std::vector<int> sourceCandidateIndices_temp;

		PointPairs final_s2t;
		static Transformation solveRegistration(PointPairs &_s2t, PairRegistration::SolveMethod _sMethod);

		typedef boost::shared_ptr<PairRegistration> Ptr;
	};

	typedef PairRegistration::Ptr PairRegistrationPtr;
	typedef std::map<Link,PairRegistrationPtr,LinkComp> PairRegistrationPtrMap;

	#ifdef _OPENMP

	class PairRegistrationOMP : public PairRegistration
	{
	public:

		PairRegistrationOMP(ScanPtr _target, ScanPtr _source, unsigned int _threads = 0) : PairRegistration(_target, _source), threads(_threads) {}
		~PairRegistrationOMP() {}

	    inline void setNumberOfThreads (unsigned int _threads = 0) { threads = _threads; }

	    void startRegistrationOMP();

		static void generatePointPairsOMP(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, 
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _sbuffer, const Transformation &_transformation, PairRegistration::Parameters _para, PointPairs &_s2t, unsigned int _threads);

		static void generatePointPairsOMP(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree, 
								std::vector<int> &_targetCandidateIndices, std::vector<int> &_targetCandidateIndices_temp,
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _buffer, const Transformation &_transformation, PairRegistration::Parameters _para, PointPairs &_s2t, PointPairs &_t2s, unsigned int _threads);

		virtual void generateFinalPointPairs(Transformation _transformation);

		unsigned int threads;

		typedef boost::shared_ptr<PairRegistrationOMP> Ptr;
	};

	typedef PairRegistrationOMP::Ptr PairRegistrationOMPPtr;

	#endif
}

#endif

