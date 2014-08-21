#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include <vector>
#include <Eigen/Dense>
#include <QtCore/QObject>

#include "pclbase.h"
#include "registrationdatamanager.h"

#include <QtCore/QVariantMap>  ///////////////////
class CloudVisualizer;   ///////////////////////

struct Correspondence
{
	PointType targetPoint;
	PointType sourcePoint;
};

typedef std::vector<Correspondence, Eigen::aligned_allocator<Correspondence> > Correspondences;

struct CorrespondencesComputationData
{
	CloudData cloudData_source_dynamic;
	pcl::Correspondences pcl_correspondences;
	pcl::Correspondences pcl_correspondences_temp;
};

enum CorrespondenceComputationMethod
{
	POINT_TO_POINT, POINT_TO_PLANE, POINT_TO_MLSSURFACE
};
struct CorrespondencesComputationParameters
{
	CorrespondenceComputationMethod method;
	float distanceThreshold;
	float normalAngleThreshold;
	bool boundaryTest;
	bool biDirectional;
};

struct PairwiseRegistrationComputationData
{
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src;
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_tgt;
};

enum PairwiseRegistrationComputationMethod
{
	SVD, UMEYAMA
};
struct PairwiseRegistrationComputationParameters
{
	PairwiseRegistrationComputationMethod method;
	bool allowScaling;
};

class PairwiseRegistration : public QObject
{
	Q_OBJECT

public:

	PairwiseRegistration(RegistrationData *target, RegistrationData *source, 
		QString registrationName, QObject *parent = 0);
	virtual ~PairwiseRegistration();

	Eigen::Matrix4f transformation;

	RegistrationData *target;
	RegistrationData *source;

	static void preCorrespondences(RegistrationData *target, RegistrationData *source,
		Eigen::Matrix4f initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
		Correspondences &correspondences, CorrespondencesComputationData &correspondencesComputationData);

	static Eigen::Matrix4f registAr(Correspondences &correspondences, 
		PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, 
		PairwiseRegistrationComputationData &pairwiseRegistrationComputationData);

	static Eigen::Matrix4f icp(RegistrationData *target, RegistrationData *source, 
		Eigen::Matrix4f initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
		PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, int iterationNumber);

	static QString generateName(QString targetName, QString sourceName);

	CloudVisualizer *cloudVisualizer;  //////////////////////////////

	float rmsError_total;  /////////////////////////////////////////////
	std::vector<float> squareErrors_total; /////////////////////////////

	void reinitialize();  //////////////////////////////////////////////
	void process(QVariantMap parameters); //////////////////////////////////
	void initializeTransformation(Eigen::Matrix4f transformation);///////////////////////
	bool correspondencesOK;/////////////////////////////////////////////////////////////////
	Eigen::Matrix4f transformation_inverse;////////////////////////////////////////////////////
	void estimateRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber); //////////////////////////
	void estimateVirtualRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber); /////////////////////

	static void computeSquareErrors(Correspondences &correspondences, std::vector<float> &squareErrors_total, float &rmsError_total);////////////////////
	void renderErrorMap();////////////////////////////////////////////////////////////
};

class PairwiseRegistrationManager : public QObject
{
	Q_OBJECT

public:
	PairwiseRegistrationManager(QObject *parent = 0);
	virtual ~PairwiseRegistrationManager();
	PairwiseRegistration *addPairwiseRegistration(RegistrationData *target, RegistrationData *source, 
		QString registrationName);
	void removePairwiseRegistration(QString registrationName);
	PairwiseRegistration *getPairwiseRegistration(QString registrationName);
};


#endif 