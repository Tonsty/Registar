#include "../include/pairwiseregistration.h"

PairwiseRegistration::PairwiseRegistration(RegistrationData *target, RegistrationData *source, 
	QObject *parent) : QObject(parent)
{
	this->target = target;
	this->source = source;

	// method = POINT_TO_POINT;
	// distanceThreshold = std::numeric_limits<float>::max();
	// normalAngleThreshold = 180.0f;
	// boundaryTest = false;
	// allowScaling = false;
	// biDirectional = false;
}

PairwiseRegistration::~PairwiseRegistration() {}


static void preCorrespondences(RegistrationData *target, RegistrationData *source,
	CorrspondencesComputationData &correspondencesComputationData,
	CorrspondencesComputationParameters &corrspondencesComputationParameters,
	Correspondences &correspondences)
{
	
}

Eigen::Matrix4f PairwiseRegistration::registAr(Correspondences &correspondences, bool allowScaling)
{

}

Eigen::Matrix4f PairwiseRegistration::icp(RegistrationData *target, RegistrationData *source, 
	Eigen::Matrix4f initialTransformation, Method method, float distanceThreshold, 
	float normalAngleThreshold, bool boundaryTest, bool biDirectional, 
	bool allowScaling, int iterationNumber)
{
	PairwiseRegistrationComputationData pairwiseRegistrationComputationData;
	initializeComputationData(target, source, pairwiseRegistrationComputationData);
	Correspondences correspondences;
	for (int i = 0; i < iterationNumber; ++i)
	{
		preCorrespondences(pairwiseRegistrationComputationData, initialTransformation, method, distanceThreshold, 
			normalAngleThreshold, boundaryTest, biDirectional, correspondences);
		initialTransformation = registAr(correspondences, allowScaling);
	}
	return initialTransformation;
}