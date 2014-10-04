#include "globalregistration.h"

#include "SRoMCPS.h"
#include "../include/mathutilities.h"

void GlobalRegistration::startRegistration()
{
	buildKdTreePtrs();
	initialTransformations();	
	initialPairRegistration();
	incrementalLoopRefine();
	globalRefine();
}

void GlobalRegistration::buildKdTreePtrs()
{
	for (int i = 0; i < scanPtrs.size(); ++i)
	{
		KdTreePtr kdTreePtr(new KdTree);
		kdTreePtr->setInputCloud(scanPtrs[i]->pointsPtr);
		kdTreePtrs.push_back(kdTreePtr);
	}
}

void GlobalRegistration::initialTransformations()
{
	for (int i = 0; i < scanPtrs.size(); ++i)
	{
		transformations.push_back(Transformation::Identity());
	}
}

void GlobalRegistration::initialPairRegistration()
{
	for (int i = 0; i < links.size(); ++i)
	{
		Link link = links[i];
		ScanIndex a = link.a;
		ScanIndex b = link.b;
		PairRegistrationPtr pairReigstrationPtr(new PairRegistration(scanPtrs[a], scanPtrs[b]));
		pairReigstrationPtr->setKdTree(kdTreePtrs[a], kdTreePtrs[b]);
		pairReigstrationPtr->setParameter(PairRegistration::POINT_TO_PLANE, PairRegistration::UMEYAMA, Transformation::Identity(), true, 0.12f, true, 45.0f, true, true, 1);
		pairReigstrationPtr->startRegistration();
		std::pair<Link, PairRegistrationPtr> pairLP(link, pairReigstrationPtr);
		pairRegistrationPtrMap.insert(pairLP);
	}
}

void GlobalRegistration::incrementalLoopRefine()
{
	for (int i = 0; i < loops.size(); ++i)
	{

	}
}

void GlobalRegistration::globalPairRefine()
{

}

void GlobalRegistration::globalRefine()
{
	PointsPtr buffer(new Points);
	PairRegistration::PointPairs s2t, t2s;

	sromcps::ScanIndexPairs sipairs;
	std::vector<sromcps::PointPairWithWeights> ppairwwss;
	int M = scanPtrs.size();

	sromcps::PointPairWithWeights buffer1;
	for (int i = 0; i < links.size(); ++i)
	{
		Link link = links[i];
		ScanIndex a = link.a;
		ScanIndex b = link.b;

		sipairs.push_back(sromcps::ScanIndexPair(a,b));

		buffer->clear();
		s2t.clear();
		t2s.clear();
		Transformation transformation = transformations[b] * transformations[a].inverse();
		PairRegistrationPtr pairRegistrationPtr = pairRegistrationPtrMap[link];
		pairRegistrationPtr->generatePointPairs(buffer, transformation, s2t, t2s);

		for (int j = 0; j < s2t.size(); ++j) s2t[j].sourcePoint = transformPointWithNormal(s2t[j].sourcePoint, transformation.inverse());
		for (int j = 0; j < t2s.size(); ++j) t2s[j].sourcePoint = transformPointWithNormal(t2s[j].sourcePoint, transformation);

		buffer1.clear();
		for (int j = 0; j < s2t.size(); ++j)
		{
			sromcps::PointPairWithWeight temp;
			temp.w = 1.0;
			temp.ppair.first = s2t[j].targetPoint.getVector3fMap().cast<sromcps::Scalar>(); 
			temp.ppair.second = s2t[j].sourcePoint.getVector3fMap().cast<sromcps::Scalar>();
			buffer1.push_back(temp);
		}
		for (int j = 0; j < t2s.size(); ++j)
		{
			sromcps::PointPairWithWeight temp;
			temp.w = 1.0;
			temp.ppair.first = t2s[j].sourcePoint.getVector3fMap().cast<sromcps::Scalar>(); 
			temp.ppair.second = t2s[j].targetPoint.getVector3fMap().cast<sromcps::Scalar>();
			buffer1.push_back(temp);
		}		
		ppairwwss.push_back(buffer1);
	}

	sromcps::SRoMCPS sromcps_globalrefine(sipairs, ppairwwss, M);

}

	// Transformation resultTransformation, resultTransformation0, resultTransformation1;
	// resultTransformation0 = Transformation::Identity();
	// resultTransformation0.block<3,3>(0,0) = sromcps_globalrefine.R.block<3,3>(0,0).cast<float>();
	// resultTransformation0.block<3,1>(0,3) = sromcps_globalrefine.T.block<3,1>(0,0).cast<float>();
	// resultTransformation1 = Transformation::Identity();
	// resultTransformation1.block<3,3>(0,0) = sromcps_globalrefine.R.block<3,3>(0,3).cast<float>();
	// resultTransformation1.block<3,1>(0,3) = sromcps_globalrefine.T.block<3,1>(3,0).cast<float>();

	// resultTransformation = resultTransformation1 * resultTransformation0.inverse();
	// std::cout << resultTransformation << std::endl;

