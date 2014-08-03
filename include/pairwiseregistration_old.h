#ifndef PAIRWISEREGISTRATION_OLD_H
#define PAIRWISEREGISTRATION_OLD_H

#include <QtGui/QWidget>
#include <QtCore/QVariantMap>
#include <pcl/features/boundary.h>

#include "pclbase.h"
#include "cloud.h"

class CloudVisualizer;

class PairwiseRegistration : public QObject
{
	Q_OBJECT

public:
	enum Method
	{
		POINT_POINT, POINT_PLANE, POINT_MLSSURFACE
	};

	PairwiseRegistration(Cloud *cloud_target, Cloud *cloud_source, QObject *parent = 0);
	~PairwiseRegistration();

	Cloud* cloud_target;
	Cloud* cloud_source;

	CloudDataPtr cloudData_target;
	CloudDataPtr cloudData_source;

	CloudDataPtr cloudData_target_dynamic;
	CloudDataPtr cloudData_source_dynamic;

	KdTreePtr tree_target;
	KdTreePtr tree_source;

	BoundariesPtr boundaries_target;
	BoundariesPtr boundaries_source;

	pcl::CorrespondencesPtr correspondences;
	pcl::CorrespondencesPtr correspondences_temp;

	pcl::CorrespondencesPtr correspondences_inverse;
	pcl::CorrespondencesPtr correspondences_temp_inverse;

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

	Eigen::Matrix4f transformation;
	Eigen::Matrix4f transformation_inverse;

	Method method;
	float distanceThreshold;
	float normalAngleThreshold;
	bool boundaryTest;
	bool allowScaling;
	bool biDirectional;

	CloudVisualizer *cloudVisualizer;

	bool correspondencesOK;

	void initialize();
	void reinitialize();

	void initializeTransformation(Eigen::Matrix4f transformation);
	void buildNearestCorrespondences(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse);
	void filterCorrespondencesByDistanceAndNormal(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse);
	void filterCorrespondencesByBoundaries(
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse);
	void refineCorrespondences(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt_inverse);
	void preCorrespondences();
	void icp();
	void icp(int iterationNumber);

	//void buildNearestCorrespondences_GPU();
	void estimateRMSErrorByTransformation(Eigen::Matrix4f transformation, float &error1, int &ovlNumber1);
	void estimateVirtualRMSErrorByTransformation(Eigen::Matrix4f transformation, float &error2, int &ovlNumber2);

	void computeSquareErrors(
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt_inverse,
		std::vector<float> &squareErrors,
		std::vector<float> &squareErrors_inverse,
		std::vector<float> &squareErrors_total,
		float &rmsError, float &rmsError_inverse, float &rmsError_total);

	void exportTransformation();

	void setCloudVisualizer(CloudVisualizer *cloudVisualizer);
	void showBoundaries();
	void showCorrespondences();
	void renderErrorMap();
	void process(QVariantMap parameters);

	

};


class PairwiseRegistrationManager : public QObject
{
	Q_OBJECT

public:
	PairwiseRegistrationManager(QObject *parent = 0);
	~PairwiseRegistrationManager();

	PairwiseRegistration* addPairwiseRegistration(Cloud *cloud_target, Cloud *cloud_source);
	void removePairwiseRegistration(QString cloudName_target, QString cloudName_source);
	PairwiseRegistration* getPairwiseRegistration(QString cloudName_target, QString cloudName_source);
};

#endif