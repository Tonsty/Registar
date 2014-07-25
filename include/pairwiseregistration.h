#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include <vector>
#include <Eigen/Dense>
#include <QtCore/QObject>

#include "pclbase.h"
#include "registrationdatamanager.h"

struct Correspondence
{
	PointType targetPoint;
	PointType sourcePoint;
};

typedef std::vector<Correspondence> Correspondences;

struct CorrspondencesComputationData
{
	CloudDataPtr cloudData_target_dynamic;
	CloudDataPtr cloudData_source_dynamic;

	pcl::CorrespondencesPtr correspondences;
	pcl::CorrespondencesPtr correspondences_temp;

	pcl::CorrespondencesPtr correspondences_inverse;
	pcl::CorrespondencesPtr correspondences_temp_inverse;
};

struct CorrspondencesComputationParameters
{
	enum Method
	{
		POINT_TO_POINT, POINT_TO_PLANE, POINT_TO_MLSSURFACE
	};
	Method method;
	float distanceThreshold;
	float normalAngleThreshold;
	bool boundaryTest;
	bool biDirectional;
};

struct PairwiseRegistrationComputationData
{
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src;
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_tgt;

	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src_inverse;
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_tgt_inverse;

	std::vector<float> squareErrors;
	std::vector<float> squareErrors_inverse;
	std::vector<float> squareErrors_total;

	float rmsError;
	float rmsError_inverse;
	float rmsError_total;
};

class PairwiseRegistration : public QObject
{
	Q_OBJECT

public:

	PairwiseRegistration(RegistrationData *target, RegistrationData *source, QObject *parent = 0);
	virtual ~PairwiseRegistration();

	Eigen::Matrix4f transformation;

	RegistrationData *target;
	RegistrationData *source;

	static void preCorrespondences(RegistrationData *target, RegistrationData *source,
		CorrspondencesComputationData &correspondencesComputationData,
		CorrspondencesComputationParameters &corrspondencesComputationParameters,
		Correspondences &correspondences);

	static Eigen::Matrix4f registAr(Correspondences &correspondences, 
		PairwiseRegistrationComputationData &pairwiseRegistrationComputationData, bool allowScaling);

	static Eigen::Matrix4f icp(RegistrationData *target, RegistrationData *source, 
		CorrspondencesComputationParameters &corrspondencesComputationParameters, 
		bool allowScaling, int iterationNumber);
};


#endif 