#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include <QtGui/QWidget>
#include <QtCore/QVariantMap>

#ifndef Q_MOC_RUN
#include <pcl/features/boundary.h>
#include <pcl/gpu/octree/octree.hpp>
// #include "pclbase.h"
#include "cloud.h"
#endif

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

	pcl::search::KdTree<PointType>::Ptr tree_target;
	pcl::search::KdTree<PointType>::Ptr tree_source;

	Method method;

	float distanceThreshold;
	float normalAngleThreshold;

	pcl::PointCloud<pcl::Boundary>::Ptr boundaries_target;
	pcl::PointCloud<pcl::Boundary>::Ptr boundaries_source;

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

	bool correspondencesOK;
	bool boundaryTest;
	bool allowScaling;

	CloudVisualizer *cloudVisualizer;

	void initialize();
	void reinitialize();

	void initializeTransformation(const Eigen::Matrix4f &transformation);
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
	void estimateRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &error1, int &ovlNumber1);
	void estimateVirtualRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &error2, int &ovlNumber2);

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
	PairwiseRegistration* getPairwiseRegistration(QString cloudName_target, QString cloudName_source);
};

#endif