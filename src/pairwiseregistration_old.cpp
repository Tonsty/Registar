#include <QtCore/QDebug>
#include <pcl/features/boundary.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/distances.h>

//#include "../include/qtbase.h"
#include "../include/cloudvisualizer.h"
#include "../include/pairwiseregistration.h" 

PairwiseRegistration::PairwiseRegistration(Cloud *cloud_target, Cloud *cloud_source, QObject *parent) : QObject(parent)
{
	this->cloud_target = cloud_target;
	this->cloud_source = cloud_source;
	initialize();
}

void PairwiseRegistration::initialize()
{
	cloudData_target.reset(new CloudData);
	pcl::transformPointCloudWithNormals(*cloud_target->getCloudData(), *cloudData_target, cloud_target->getTransformation());
	cloudData_source.reset(new CloudData);
	pcl::transformPointCloudWithNormals(*cloud_source->getCloudData(), *cloudData_source, cloud_source->getTransformation());

	boundaries_target = cloud_target->getBoundaries();
	boundaries_source = cloud_source->getBoundaries();

	tree_target.reset(new pcl::search::KdTree<PointType>);
	tree_target->setInputCloud(cloudData_target);
	tree_source.reset(new pcl::search::KdTree<PointType>);
	tree_source->setInputCloud(cloudData_source);

	cloudData_target_dynamic.reset(new CloudData);
	cloudData_source_dynamic.reset(new CloudData);
	initializeTransformation(Eigen::Matrix4f::Identity());

	correspondences.reset(new pcl::Correspondences);
	correspondences_temp.reset(new pcl::Correspondences);

	correspondences_inverse.reset(new pcl::Correspondences);
	correspondences_temp_inverse.reset(new pcl::Correspondences);

	method = POINT_POINT;

	distanceThreshold = 0.5f;
	normalAngleThreshold = 45.0f;

	transformation = Eigen::Matrix4f::Identity();
	transformation_inverse = Eigen::Matrix4f::Identity();

	correspondencesOK = false;
	boundaryTest = false;
	allowScaling = false;

}

void PairwiseRegistration::reinitialize()
{
	pcl::transformPointCloudWithNormals(*cloud_target->getCloudData(), *cloudData_target, cloud_target->getTransformation());
	pcl::transformPointCloudWithNormals(*cloud_source->getCloudData(), *cloudData_source, cloud_source->getTransformation());

	tree_target->setInputCloud(cloudData_target);
	tree_source->setInputCloud(cloudData_source);

	initializeTransformation(Eigen::Matrix4f::Identity());

	boundaries_target = cloud_target->getBoundaries();
	boundaries_source = cloud_source->getBoundaries();

	transformation = Eigen::Matrix4f::Identity();
	transformation_inverse = Eigen::Matrix4f::Identity();

	squareErrors.clear();

	correspondences->clear();
	correspondences_temp->clear();

	correspondences_inverse->clear();
	correspondences_temp_inverse->clear();

	correspondencesOK = false;
	boundaryTest = false;
	allowScaling = false;

	if(cloudVisualizer) cloudVisualizer->removeCloud("target");
	if(cloudVisualizer) cloudVisualizer->removeCloud("source");
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudBoundaries_target");
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudBoundaries_source");
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudCorrespondences_target");
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudCorrespondences_source");

	if(cloudVisualizer) cloudVisualizer->addCloud(cloudData_target, "target");
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudData_source_dynamic, "source");
}

void PairwiseRegistration::initializeTransformation(Eigen::Matrix4f transformation)
{
	this->transformation = transformation;
	pcl::transformPointCloudWithNormals(*cloudData_source, *cloudData_source_dynamic, this->transformation);

	this->transformation_inverse = this->transformation.inverse();
	pcl::transformPointCloudWithNormals(*cloudData_target, *cloudData_target_dynamic, this->transformation_inverse);
}

PairwiseRegistration::~PairwiseRegistration()
{
}

void PairwiseRegistration::setCloudVisualizer(CloudVisualizer *cloudVisualizer)
{
	this->cloudVisualizer = cloudVisualizer;
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudData_target, "target");
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudData_source_dynamic, "source");
	//if(cloudVisualizer)  cloudVisualizer->addCloud(cloudData_target, "target");
	//if(cloudVisualizer)  cloudVisualizer->addCloud(cloudData_source_dynamic, "source");
}

void PairwiseRegistration::showBoundaries()
{
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudBoundaries_target");
	CloudDataPtr cloudBoundaries_target(new CloudData);
	for (int i = 0; i < boundaries_target->size(); ++i)
	{
		if((*boundaries_target)[i].boundary_point != 0)
		{
			PointType point = (*cloudData_target)[i];
			point.r = 255;
			point.g = 255;
			point.b = 0;
			cloudBoundaries_target->push_back(point);
		}
	}
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudBoundaries_target, "cloudBoundaries_target", 254.0f, 254.0f, 0.0f);

	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudBoundaries_source");
	CloudDataPtr cloudBoundaries_source(new CloudData);
	for (int i = 0; i < boundaries_source->size(); ++i)
	{
		if((*boundaries_source)[i].boundary_point != 0)
		{
			PointType point = (*cloudData_source_dynamic)[i];
			point.r = 255;
			point.g = 255;
			point.b = 0;
			cloudBoundaries_source->push_back(point);
		}
	}
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudBoundaries_source, "cloudBoundaries_source", 254.0f, 254.0f, 0.0f);
}

void PairwiseRegistration::buildNearestCorrespondences(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse)
{
	correspondences_temp->clear();
	for (int i = 0; i < cloudData_source_dynamic->size(); ++i)
	{
		PointType point = (*cloudData_source_dynamic)[i];
		int K = 1;
		std::vector<int> indices(K);
		std::vector<float> sqr_distances(K);
		if(tree_target->nearestKSearch(point, K, indices, sqr_distances) > 0)
		{
			pcl::Correspondence temp;
			temp.index_query = i;
			temp.index_match = indices[0];
			// temp.distance = sqrtf(sqr_distances[0]);
			temp.distance = sqr_distances[0];
			correspondences_temp->push_back(temp);
		}
	}
	correspondences->swap(*correspondences_temp);

	correspondences_temp_inverse->clear();
	for (int i = 0; i < cloudData_target_dynamic->size(); ++i)
	{
		PointType point = (*cloudData_target_dynamic)[i];
		int K = 1;
		std::vector<int> indices(K);
		std::vector<float> sqr_distances(K);
		if(tree_source->nearestKSearch(point, K, indices, sqr_distances) > 0)
		{
			pcl::Correspondence temp;
			temp.index_query = i;
			temp.index_match = indices[0];
			// temp.distance = sqrtf(sqr_distances[0]);
			temp.distance = sqr_distances[0];
			correspondences_temp_inverse->push_back(temp);
		}
	}
	correspondences_inverse->swap(*correspondences_temp_inverse);

}

void PairwiseRegistration::filterCorrespondencesByDistanceAndNormal(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse)
{
	float NAthreshold = cosf(normalAngleThreshold / 180.0f * M_PI);
	float sqr_distanceThreshold = distanceThreshold * distanceThreshold;

	correspondences_temp->clear();
	for (int i = 0; i < correspondences->size(); ++i)
	{
		if((*correspondences)[i].distance < sqr_distanceThreshold)
		{
			int index_query = (*correspondences)[i].index_query;
			int index_match = (*correspondences)[i].index_match;

			Eigen::Vector3f query_normal = (*cloudData_source_dynamic)[index_query].getNormalVector3fMap();
			Eigen::Vector3f match_normal = (*cloudData_target)[index_match].getNormalVector3fMap();
			query_normal.normalize();
			match_normal.normalize();

			if(query_normal.dot(match_normal) > NAthreshold)
			{
				correspondences_temp->push_back((*correspondences)[i]);
			}
		}
	}
	correspondences->swap(*correspondences_temp);

	correspondences_temp_inverse->clear();
	for (int i = 0; i < correspondences_inverse->size(); ++i)
	{
		if((*correspondences_inverse)[i].distance < sqr_distanceThreshold)
		{
			int index_query = (*correspondences_inverse)[i].index_query;
			int index_match = (*correspondences_inverse)[i].index_match;

			Eigen::Vector3f query_normal = (*cloudData_target_dynamic)[index_query].getNormalVector3fMap();
			Eigen::Vector3f match_normal = (*cloudData_source)[index_match].getNormalVector3fMap();
			query_normal.normalize();
			match_normal.normalize();

			if(query_normal.dot(match_normal) > NAthreshold)
			{
				correspondences_temp_inverse->push_back((*correspondences_inverse)[i]);
			}
		}
	}
	correspondences_inverse->swap(*correspondences_temp_inverse);
}

void PairwiseRegistration::filterCorrespondencesByBoundaries(
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_temp,
		pcl::CorrespondencesPtr correspondences_inverse,
		pcl::CorrespondencesPtr correspondences_temp_inverse)
{
	correspondences_temp->clear();
	for (int i = 0; i < correspondences->size(); ++i)
	{
		int index_match = (*correspondences)[i].index_match;
		if((*boundaries_target)[index_match].boundary_point == 0)
		{
			correspondences_temp->push_back((*correspondences)[i]);
		}
	}
	correspondences->swap(*correspondences_temp);

	correspondences_temp_inverse->clear();
	for (int i = 0; i < correspondences_inverse->size(); ++i)
	{
		int index_match = (*correspondences_inverse)[i].index_match;
		if((*boundaries_source)[index_match].boundary_point == 0)
		{
			correspondences_temp_inverse->push_back((*correspondences_inverse)[i]);
		}
	}
	correspondences_inverse->swap(*correspondences_temp_inverse);
}

void PairwiseRegistration::refineCorrespondences(
		CloudDataPtr cloudData_target_dynamic,
		CloudDataPtr cloudData_source_dynamic,
		pcl::CorrespondencesPtr correspondences, 
		pcl::CorrespondencesPtr correspondences_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt_inverse)
{
	cloud_src.resize(Eigen::NoChange, correspondences->size());
	cloud_tgt.resize(Eigen::NoChange, correspondences->size());

	cloud_src_inverse.resize(Eigen::NoChange, correspondences_inverse->size());
	cloud_tgt_inverse.resize(Eigen::NoChange, correspondences_inverse->size());

	switch(method)
	{
		case POINT_POINT:
		{
			//Original ICP
			for (int i = 0; i < correspondences->size(); ++i)
			{
				int query = (*correspondences)[i].index_query;
				int match = (*correspondences)[i].index_match;

				cloud_src (0, i) = (*cloudData_source_dynamic)[query].x;
				cloud_src (1, i) = (*cloudData_source_dynamic)[query].y;
				cloud_src (2, i) = (*cloudData_source_dynamic)[query].z;

				cloud_tgt (0, i) = (*cloudData_target)[match].x;
				cloud_tgt (1, i) = (*cloudData_target)[match].y;
				cloud_tgt (2, i) = (*cloudData_target)[match].z;
			}

			for (int i = 0; i < correspondences_inverse->size(); ++i)
			{
				int query = (*correspondences_inverse)[i].index_query;
				int match = (*correspondences_inverse)[i].index_match;

				cloud_src_inverse (0, i) = (*cloudData_target_dynamic)[query].x;
				cloud_src_inverse (1, i) = (*cloudData_target_dynamic)[query].y;
				cloud_src_inverse (2, i) = (*cloudData_target_dynamic)[query].z;

				cloud_tgt_inverse (0, i) = (*cloudData_source)[match].x;
				cloud_tgt_inverse (1, i) = (*cloudData_source)[match].y;
				cloud_tgt_inverse (2, i) = (*cloudData_source)[match].z;
			}
			break;		
		}
		case POINT_PLANE:
		{
			//Point-Plane based ICP
			for (int i = 0; i < correspondences->size(); ++i)
			{
				int query = (*correspondences)[i].index_query;
				int match = (*correspondences)[i].index_match;

				cloud_src (0, i) = (*cloudData_source_dynamic)[query].x;
				cloud_src (1, i) = (*cloudData_source_dynamic)[query].y;
				cloud_src (2, i) = (*cloudData_source_dynamic)[query].z;

				Eigen::Vector3f target_normal = (*cloudData_target)[match].getNormalVector3fMap();
				Eigen::Vector3f source_point = (*cloudData_source_dynamic)[query].getVector3fMap();
				Eigen::Vector3f target_point = (*cloudData_target)[match].getVector3fMap();

				target_normal.normalize();
				target_point = source_point - (source_point - target_point).dot(target_normal) * target_normal;

				cloud_tgt (0, i) = target_point.x();
				cloud_tgt (1, i) = target_point.y();
				cloud_tgt (2, i) = target_point.z();
			}

			for (int i = 0; i < correspondences_inverse->size(); ++i)
			{
				int query = (*correspondences_inverse)[i].index_query;
				int match = (*correspondences_inverse)[i].index_match;

				cloud_src_inverse (0, i) = (*cloudData_target_dynamic)[query].x;
				cloud_src_inverse (1, i) = (*cloudData_target_dynamic)[query].y;
				cloud_src_inverse (2, i) = (*cloudData_target_dynamic)[query].z;

				Eigen::Vector3f source_normal = (*cloudData_source)[match].getNormalVector3fMap();
				Eigen::Vector3f target_point = (*cloudData_target_dynamic)[query].getVector3fMap();
				Eigen::Vector3f source_point = (*cloudData_source)[match].getVector3fMap();

				source_normal.normalize();
				source_point = target_point - (target_point - source_point).dot(source_normal) * source_normal;

				cloud_tgt_inverse (0, i) = source_point.x();
				cloud_tgt_inverse (1, i) = source_point.y();
				cloud_tgt_inverse (2, i) = source_point.z();
			}
			break;
		}
		case POINT_MLSSURFACE:
		{
			break;
		}
	}	
}

void PairwiseRegistration::preCorrespondences()
{
	if (correspondencesOK == false)
	{
		buildNearestCorrespondences(
			cloudData_target_dynamic, cloudData_source_dynamic, 
			correspondences, correspondences_temp, 
			correspondences_inverse, correspondences_temp_inverse);
		filterCorrespondencesByDistanceAndNormal(
			cloudData_target_dynamic, cloudData_source_dynamic, 
			correspondences, correspondences_temp, 
			correspondences_inverse, correspondences_temp_inverse);
		if(boundaryTest && boundaries_target != NULL && boundaries_source != NULL)
		{
			filterCorrespondencesByBoundaries(
			correspondences, correspondences_temp,
			correspondences_inverse, correspondences_temp_inverse);
		}

		refineCorrespondences(
			cloudData_target_dynamic, cloudData_source_dynamic, 
			correspondences, correspondences_inverse,
			cloud_src, cloud_tgt, 
			cloud_src_inverse, cloud_tgt_inverse);
		correspondencesOK = true;
	}
}

void PairwiseRegistration::computeSquareErrors(
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src_inverse,
		Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt_inverse,
		std::vector<float> &squareErrors,
		std::vector<float> &squareErrors_inverse,
		std::vector<float> &squareErrors_total,
		float &rmsError, float &rmsError_inverse, float &rmsError_total)
{
	squareErrors.resize(cloud_src.cols());
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_diff (3, cloud_src.cols() );
	cloud_diff = cloud_tgt - cloud_src;
	for (int i = 0; i < cloud_src.cols(); ++i)
		squareErrors[i] = cloud_diff.block(0,i,3,1).squaredNorm();

	squareErrors_inverse.resize(cloud_src_inverse.cols());
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_diff_inverse (3, cloud_src_inverse.cols() );
	cloud_diff_inverse = cloud_tgt_inverse - cloud_src_inverse;
	for (int i = 0; i < cloud_src_inverse.cols(); ++i)
		squareErrors_inverse[i] = cloud_diff_inverse.block(0,i,3,1).squaredNorm();

	squareErrors_total.clear();
	squareErrors_total.insert(
		squareErrors_total.end(), 
		squareErrors.begin(), 
		squareErrors.end());
	squareErrors_total.insert(
		squareErrors_total.end(),
		squareErrors_inverse.begin(),
		squareErrors_inverse.end());

	float mean = 0.0f;
	for (int i = 0; i < squareErrors.size(); ++i) mean += squareErrors[i];
	mean /= squareErrors.size();
	rmsError = sqrtf(mean);

	float mean_inverse = 0.0f;
	for(int i = 0; i < squareErrors_inverse.size(); ++i) mean_inverse += squareErrors_inverse[i];
	mean_inverse /= squareErrors_inverse.size();
	rmsError_inverse = sqrt(mean_inverse);

	float mean_total = 0.0f;
	for (int i = 0; i < squareErrors_total.size(); ++i) mean_total += squareErrors_total[i];
	mean_total /= squareErrors_total.size();
	rmsError_total = sqrtf(mean_total);

	// qDebug() << "Correspondences number is " << QString::number(cloud_src.cols());
	// qDebug() << "RMS Error is " << QString::number(mean);
}

void PairwiseRegistration::showCorrespondences()
{
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudCorrespondences_target");
	CloudDataPtr cloudCorrespondences_target(new CloudData);
	if(cloudVisualizer) cloudVisualizer->removeCloud("cloudCorrespondences_source");
	CloudDataPtr cloudCorrespondences_source(new CloudData);

	for (int i = 0; i < correspondences->size(); ++i)
	{
		int index_query = (*correspondences)[i].index_query;
		PointType point_query = (*cloudData_source_dynamic)[index_query];
		int index_match = (*correspondences)[i].index_match;
		PointType point_match = (*cloudData_target)[index_match];
		point_query.r = 0;
		point_query.g = 255;
		point_query.b = 0;
		point_match.r = 0;
		point_match.g = 255;
		point_match.b = 0;
		cloudCorrespondences_source->push_back(point_query);
		cloudCorrespondences_target->push_back(point_match);
	}
	for (int i = 0; i < correspondences_inverse->size(); ++i)
	{
		int index_query = (*correspondences_inverse)[i].index_query;
		PointType point_query = (*cloudData_target)[index_query];
		int index_match = (*correspondences_inverse)[i].index_match;
		PointType point_match = (*cloudData_source_dynamic)[index_match];
		point_query.r = 0;
		point_query.g = 255;
		point_query.b = 0;
		point_match.r = 0;
		point_match.g = 255;
		point_match.b = 0;
		cloudCorrespondences_source->push_back(point_query);
		cloudCorrespondences_target->push_back(point_match);
	}

	if(cloudVisualizer) cloudVisualizer->addCloud(cloudCorrespondences_target, "cloudCorrespondences_target", 0.0f, 254.0f, 0.0f);
	if(cloudVisualizer) cloudVisualizer->addCloud(cloudCorrespondences_source, "cloudCorrespondences_source", 0.0f, 254.0f, 0.0f);
}

void PairwiseRegistration::icp()
{
	// Eigen::Matrix4f transformation_matrix = pcl::umeyama (cloud_src, cloud_tgt, true );
	// pcl::transformPointCloudWithNormals(*cloudData_source_dynamic, *cloudData_source_dynamic, transformation_matrix);
	// transformation = transformation_matrix * transformation;
	// Eigen::Matrix4f transformation_matrix_inverse = pcl::umeyama (cloud_src_inverse, cloud_tgt_inverse, true );
	// pcl::transformPointCloudWithNormals(*cloudData_target_dynamic, *cloudData_target_dynamic, transformation_matrix_inverse);
	// transformation_inverse = transformation_matrix_inverse * transformation_inverse;

	Eigen::Matrix<float, 4, Eigen::Dynamic> inversed_src = Eigen::MatrixXf::Ones(4, cloud_tgt_inverse.cols());
	Eigen::Matrix<float, 4, Eigen::Dynamic> inversed_tgt = Eigen::MatrixXf::Ones(4, cloud_src_inverse.cols());
	inversed_src.block(0, 0, 3, cloud_tgt_inverse.cols()) = cloud_tgt_inverse;
	inversed_tgt.block(0, 0, 3, cloud_src_inverse.cols()) = cloud_src_inverse;
	inversed_src = transformation * inversed_src;
	inversed_tgt = transformation * inversed_tgt;

	Eigen::Matrix<float, 3, Eigen::Dynamic> src, tgt;
	src.resize(Eigen::NoChange, cloud_src.cols() + inversed_src.cols());
	tgt.resize(Eigen::NoChange, cloud_tgt.cols() + inversed_tgt.cols());
	src.block(0, 0, 3, cloud_src.cols()) = cloud_src;
	src.block(0, cloud_src.cols(), 3, inversed_src.cols()) = inversed_src.block(0, 0, 3, inversed_src.cols());
	tgt.block(0, 0, 3, cloud_tgt.cols()) = cloud_tgt;
	tgt.block(0, cloud_tgt.cols(), 3, inversed_tgt.cols()) = inversed_tgt.block(0, 0, 3, inversed_tgt.cols());

	Eigen::Matrix4f transformation_matrix = pcl::umeyama (src, tgt, allowScaling );
	transformation = transformation_matrix * transformation;
	pcl::transformPointCloudWithNormals(*cloudData_source, *cloudData_source_dynamic, transformation);

	transformation_inverse = transformation.inverse();
	pcl::transformPointCloudWithNormals(*cloudData_target, *cloudData_target_dynamic, transformation_inverse);
}

void PairwiseRegistration::icp(int iterationNumber)
{
	for (int i = 0; i < iterationNumber; ++i)
	{
		qDebug() << QString::number(i) << " icp iteration...";
		preCorrespondences(); 
		icp();
		correspondencesOK = false;
	}
	qDebug() << QString::number(iterationNumber) << " icp iteration(s) completed!";

	//cloudVisualizer->updateCloud(cloudData_source_dynamic, "source", 254.0f, 127.0f, 0.0f);
	//cloudVisualizer->updateCloud(cloudData_target_dynamic, "target");
}


void PairwiseRegistration::estimateRMSErrorByTransformation(Eigen::Matrix4f transformation, float &error1, int &ovlNumber1)
{
	CloudDataPtr cloudData_target_dynamic(new CloudData);
	CloudDataPtr cloudData_source_dynamic(new CloudData);

	pcl::copyPointCloud(*cloudData_target, *cloudData_target_dynamic);
	pcl::transformPointCloudWithNormals(*cloudData_source, *cloudData_source_dynamic, transformation);

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
	pcl::CorrespondencesPtr correspondences_temp(new pcl::Correspondences);
	pcl::CorrespondencesPtr correspondences_inverse(new pcl::Correspondences);
	pcl::CorrespondencesPtr correspondences_temp_inverse(new pcl::Correspondences);
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

	buildNearestCorrespondences(
		cloudData_target_dynamic, cloudData_source_dynamic, 
		correspondences, correspondences_temp, 
		correspondences_inverse, correspondences_temp_inverse);
	filterCorrespondencesByDistanceAndNormal(
		cloudData_target_dynamic, cloudData_source_dynamic, 
		correspondences, correspondences_temp, 
		correspondences_inverse, correspondences_temp_inverse);
	if(boundaryTest && boundaries_target != NULL && boundaries_source != NULL)
	filterCorrespondencesByBoundaries(
		correspondences, correspondences_temp,
		correspondences_inverse, correspondences_temp_inverse);
	refineCorrespondences(
		cloudData_target_dynamic, cloudData_source_dynamic, 
		correspondences,  correspondences_inverse,
		cloud_src, cloud_tgt, 
		cloud_src_inverse, cloud_tgt_inverse);
	computeSquareErrors(
		cloud_src, cloud_tgt, 
		cloud_src_inverse, cloud_tgt_inverse,
		squareErrors, squareErrors_inverse, squareErrors_total, 
		rmsError, rmsError_inverse, rmsError_total);

	error1 = rmsError_total;
	ovlNumber1 = squareErrors_total.size();
}

void PairwiseRegistration::estimateVirtualRMSErrorByTransformation(Eigen::Matrix4f transformation, float &error2, int &ovlNumber2)
{
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src_virtual(cloud_src);
	Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src_inverse_virtual(cloud_src_inverse);

	Eigen::Matrix4f transformation_temp = transformation * this->transformation.inverse();
	for (int i = 0; i < cloud_src.cols(); ++i)
		cloud_src_virtual.block(0, i, 3, 1) = transformation_temp.block(0, 0, 3, 3) * cloud_src.block(0, i, 3, 1) + transformation_temp.block(0, 3, 3, 1);

	Eigen::Matrix4f transformation_temp_inverse = transformation.inverse() * this->transformation;
	for (int i = 0; i < cloud_src_inverse.cols(); ++i)
		cloud_src_inverse_virtual.block(0, i, 3, 1) = transformation_temp_inverse.block(0, 0, 3, 3) * cloud_src_inverse.block(0, i, 3, 1) + transformation_temp_inverse.block(0, 3, 3, 1);

	std::vector<float> squareErrors_virtual;
	std::vector<float> squareErrors_inverse_virtual;
	std::vector<float> squareErrors_total_virtual;
	float rmsError_virtual;
	float rmsError_inverse_virtual;
	float rmsError_total_virtual;

	computeSquareErrors(
		cloud_src_virtual, cloud_src, 
		cloud_src_inverse_virtual, cloud_src_inverse,
		squareErrors_virtual, squareErrors_inverse_virtual, squareErrors_total_virtual, 
		rmsError_virtual, rmsError_inverse_virtual, rmsError_total_virtual);

	error2 = rmsError_total_virtual;
	ovlNumber2 = squareErrors_total_virtual.size();
}

void PairwiseRegistration::exportTransformation()
{
	cloud_source->setRegistrationTransformation(transformation * cloud_source->getTransformation());
	//transformation = Eigen::Matrix4f::Identity();
}

void PairwiseRegistration::process(QVariantMap parameters)
{
	QString command = parameters["command"].toString();

	if (command == "Export")
	{	
		exportTransformation();
		return;
	}

	//method = (Method)parameters["method"].toInt();
	allowScaling = parameters["allowScaling"].toBool();
	if(correspondencesOK == false)
	{
		distanceThreshold = parameters["distanceThreshold"].toFloat();
		normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
		method = (Method)parameters["method"].toInt();
		boundaryTest = parameters["boundaryTest"].toBool();
	}
	else
	{
		if (distanceThreshold != parameters["distanceThreshold"].toFloat() || normalAngleThreshold != parameters["normalAngleThreshold"].toFloat() 
			|| method != (Method)parameters["method"].toInt() || boundaryTest != parameters["boundaryTest"].toBool())
		{
			// qDebug() << QString::number(distanceThreshold);
			// qDebug() << QString::number(normalAngleThreshold);
			// qDebug() << QString::number(method);
			qDebug() << boundaryTest;
			distanceThreshold = parameters["distanceThreshold"].toFloat();
			normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
			method = (Method)parameters["method"].toInt();
			boundaryTest = parameters["boundaryTest"].toBool();
			// qDebug() << QString::number(distanceThreshold);
			// qDebug() << QString::number(normalAngleThreshold);
			// qDebug() << QString::number(method);
			qDebug() << boundaryTest;
			correspondencesOK = false;
			qDebug() << "correspondences parameters changed";			
		}
	}

	if (command == "Pre-Correspondences")
	{
		preCorrespondences();
		computeSquareErrors(
			cloud_src, cloud_tgt, 
			cloud_src_inverse, cloud_tgt_inverse,
			squareErrors, squareErrors_inverse, squareErrors_total, 
			rmsError, rmsError_inverse, rmsError_total);
		renderErrorMap();
	}
	else if (command == "ICP")
	{
		icp(parameters["icpNumber"].toInt());
		preCorrespondences();
		computeSquareErrors(
			cloud_src, cloud_tgt, 
			cloud_src_inverse, cloud_tgt_inverse, 
			squareErrors, squareErrors_inverse, squareErrors_total,
			rmsError, rmsError_inverse, rmsError_total);
		renderErrorMap();
	}

}

void PairwiseRegistration::renderErrorMap()
{
	Eigen::Vector3f red(250, 0, 0);
	Eigen::Vector3f green(0, 250, 0);
	Eigen::Vector3f blue(0, 0, 250);
	float redError = 0.005f;
    float greenError = 0.0001f ;
    float blueError = 0.0f;

	for (int i = 0; i < cloudData_target->size(); ++i)
	{
		(*cloudData_target)[i].r = 0;
		(*cloudData_target)[i].g = 0;
		(*cloudData_target)[i].b = 250;
	}
	for (int i = 0; i < cloudData_source_dynamic->size(); ++i)
	{
		(*cloudData_source_dynamic)[i].r = 0;
		(*cloudData_source_dynamic)[i].g = 0;	
		(*cloudData_source_dynamic)[i].b = 250;
	}

	qDebug() << QString::number(correspondences_inverse->size());
	qDebug() << QString::number(correspondences->size());

    for (int i = 0; i < correspondences_inverse->size(); ++i)
    {
    	int index = (*correspondences_inverse)[i].index_query;
    	//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors_inverse[i]);
    	float error = sqrtf(squareErrors_inverse[i]);
    	if (error >= redError ) 
    	{
    		(*cloudData_target)[index].r = red[0];
    		(*cloudData_target)[index].g = red[1];
    		(*cloudData_target)[index].b = red[2];
    		continue;
    	}
    	if (error >= greenError)
    	{
    		float fraction = (error - greenError) / (redError - greenError);
    		Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
    		(*cloudData_target)[index].r = color[0];
    		(*cloudData_target)[index].g = color[1];
      		(*cloudData_target)[index].b = color[2];  		
    		continue;
    	}
    	if (error >= blueError)
    	{
    		float fraction = (error - blueError) / (greenError - blueError);
    		Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
    		(*cloudData_target)[index].r = color[0];
    		(*cloudData_target)[index].g = color[1];
      		(*cloudData_target)[index].b = color[2];  
    		continue;
    	}
    }

    for (int i = 0; i < correspondences->size(); ++i)
    {
    	int index = (*correspondences)[i].index_query;
    	//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors[i]);
    	float error = sqrtf(squareErrors[i]);
    	if (error >= redError) 
    	{
    		(*cloudData_source_dynamic)[index].r = red[0];
    		(*cloudData_source_dynamic)[index].g = red[1];
    		(*cloudData_source_dynamic)[index].b = red[2];    		
    		continue;
    	}
    	if (error >= greenError)
    	{
    		float fraction = (error - greenError) / (redError - greenError);
    		Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
    		(*cloudData_source_dynamic)[index].r = color[0];
    		(*cloudData_source_dynamic)[index].g = color[1];   
    		(*cloudData_source_dynamic)[index].b = color[2]; 		
    		continue;
    	}
    	if (error >= blueError)
    	{
    		float fraction = (error - blueError) / (greenError - blueError);
    		Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
    		(*cloudData_source_dynamic)[index].r = color[0];
    		(*cloudData_source_dynamic)[index].g = color[1];   
    		(*cloudData_source_dynamic)[index].b = color[2]; 
    		continue;
    	}
    }

	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_target, "target");
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_source_dynamic, "source");
}

PairwiseRegistrationManager::PairwiseRegistrationManager(QObject *parent) : QObject(parent){}

PairwiseRegistrationManager::~PairwiseRegistrationManager(){}

PairwiseRegistration* PairwiseRegistrationManager::addPairwiseRegistration(Cloud *cloud_target, Cloud *cloud_source)
{
	PairwiseRegistration *pairwiseRegistration = new PairwiseRegistration(cloud_target, cloud_source, this);
	pairwiseRegistration->setObjectName(cloud_target->getCloudName() + "<-" + cloud_source->getCloudName());
	return pairwiseRegistration;
}

void PairwiseRegistrationManager::removePairwiseRegistration(QString cloudName_target, QString cloudName_source)
{
	PairwiseRegistration *pairwiseRegistration = findChild<PairwiseRegistration*>(cloudName_target + "<-" + cloudName_source);
	if(pairwiseRegistration) delete pairwiseRegistration;
}

PairwiseRegistration* PairwiseRegistrationManager::getPairwiseRegistration(QString cloudName_target, QString cloudName_source)
{
	return findChild<PairwiseRegistration*>(cloudName_target + "<-" + cloudName_source);
}