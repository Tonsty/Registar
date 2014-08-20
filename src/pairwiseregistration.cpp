#include <pcl/common/transforms.h>

#include "../include/mathutilities.h"
#include "../include/pairwiseregistration.h"

#include "../include/cloudvisualizer.h"//////////////////////

PairwiseRegistration::PairwiseRegistration(RegistrationData *target, RegistrationData *source, 
	QString registrationName, QObject *parent) : QObject(parent)
{
	this->target = target;
	this->source = source;

	this->setObjectName(registrationName);

	// method = POINT_TO_POINT;
	// distanceThreshold = std::numeric_limits<float>::max();
	// normalAngleThreshold = 180.0f;
	// boundaryTest = false;
	// allowScaling = false;
	// biDirectional = false;
}

PairwiseRegistration::~PairwiseRegistration() {}


void PairwiseRegistration::preCorrespondences(RegistrationData *target, RegistrationData *source,
	Eigen::Matrix4f initialTransformation, CorrspondencesComputationParameters &corrspondencesComputationParameters, 
	Correspondences &correspondences, CorrspondencesComputationData &correspondencesComputationData)
{
	if(corrspondencesComputationParameters.biDirectional)
	{
		preCorrespondences(target, source, initialTransformation, 
			corrspondencesComputationParameters, correspondences, correspondencesComputationData);

		int inverse_correspondences_start = correspondences.size();

		preCorrespondences(source, target, initialTransformation.inverse(),
			corrspondencesComputationParameters, correspondences, correspondencesComputationData);

		for (int i = inverse_correspondences_start; i < correspondences.size(); ++i)
		{
			Correspondence correspondence = correspondences[i];
			Correspondence correspondence_inverse;
			correspondence_inverse.sourcePoint = transformPointWithNormal(correspondence.targetPoint, initialTransformation);
			correspondence_inverse.targetPoint = transformPointWithNormal(correspondence.sourcePoint, initialTransformation);
			correspondences[i] = correspondence_inverse;
		}
	}
	else
	{
		CloudData &cloudData_target = *target->cloudData;
		KdTreePtr tree_target = target->kdTree;
		BoundariesPtr boundaries_target = target->boundaries;
		CloudData &cloudData_source = *source->cloudData;

		CloudData &cloudData_source_dynamic = correspondencesComputationData.cloudData_source_dynamic;		
		pcl::transformPointCloudWithNormals(cloudData_source, cloudData_source_dynamic, initialTransformation);

		pcl::Correspondences &pcl_correspondences = correspondencesComputationData.pcl_correspondences;
		pcl::Correspondences &pcl_correspondences_temp = correspondencesComputationData.pcl_correspondences_temp;

		pcl_correspondences_temp.clear();
		for (int i = 0; i < cloudData_source_dynamic.size(); ++i)
		{
			PointType point = cloudData_source_dynamic[i];
			int K = 1;
			std::vector<int> indices(K);
			std::vector<float> distance2s(K);
			if (tree_target->nearestKSearch(point, K, indices, distance2s) > 0)
			{
				pcl::Correspondence temp;
				temp.index_query = i;
				temp.index_match = indices[0];
				//temp.distance = sqrtf(distance2s[0]);
				temp.distance = distance2s[0];
				pcl_correspondences_temp.push_back(temp);
			}
		}
		pcl_correspondences.swap(pcl_correspondences_temp);

		float normalAngleThreshold = corrspondencesComputationParameters.normalAngleThreshold;
		float NAthreshold = cosf(normalAngleThreshold / 180.0f * M_PI);
		float distanceThreshold = corrspondencesComputationParameters.distanceThreshold;
		float distanceThreshold2 = distanceThreshold * distanceThreshold;
		pcl_correspondences_temp.clear();
		for (int i = 0; i < pcl_correspondences.size(); ++i)
		{
			if (pcl_correspondences[i].distance < distanceThreshold2)
			{
				int index_query = pcl_correspondences[i].index_query;
				int index_match = pcl_correspondences[i].index_match;

				Eigen::Vector3f query_normal = cloudData_source_dynamic[index_query].getNormalVector3fMap();
				Eigen::Vector3f match_normal = cloudData_target[index_match].getNormalVector3fMap();
				query_normal.normalize();
				match_normal.normalize();

				if (query_normal.dot(match_normal) > NAthreshold)
					pcl_correspondences_temp.push_back(pcl_correspondences[i]);
			}
		}
		pcl_correspondences.swap(pcl_correspondences_temp);

		pcl_correspondences_temp.clear();
		for (int i = 0; i < pcl_correspondences_temp.size(); ++i)
		{
			int index_match = pcl_correspondences[i].index_match;
			if ((*boundaries_target)[index_match].boundary_point == 0)
				pcl_correspondences_temp.push_back(pcl_correspondences[i]);
		}
		pcl_correspondences.swap(pcl_correspondences_temp);

		CorrespondenceComputationMethod method = corrspondencesComputationParameters.method;
		switch(method)
		{
			case POINT_TO_POINT:
			{
				//Original ICP
				for (int i = 0; i < pcl_correspondences.size(); ++i)
				{
					int query = pcl_correspondences[i].index_query;
					int match = pcl_correspondences[i].index_match;

					Correspondence correspondence_temp;
					correspondence_temp.sourcePoint = cloudData_source_dynamic[query];
					correspondence_temp.targetPoint = cloudData_target[match];
					correspondences.push_back(correspondence_temp);
				}
				break;
			}
			case POINT_TO_PLANE:
			{
				//Point-Plane based ICP
				for (int i = 0; i < pcl_correspondences.size(); ++i)
				{
					int query = pcl_correspondences[i].index_query;
					int match = pcl_correspondences[i].index_match;

					Correspondence correspondence_temp;
					correspondence_temp.sourcePoint = cloudData_source_dynamic[query];

					Eigen::Vector3f target_normal = cloudData_target[match].getNormalVector3fMap();
					Eigen::Vector3f source_point = cloudData_source_dynamic[query].getVector3fMap();
					Eigen::Vector3f target_point = cloudData_target[match].getVector3fMap();

					target_normal.normalize();
					target_point = source_point - (source_point - target_point).dot(target_normal) * target_normal;

					correspondence_temp.targetPoint = cloudData_target[match];
					correspondence_temp.targetPoint.getVector3fMap() = target_point;

					correspondences.push_back(correspondence_temp);
				}
				break;
			}
			case POINT_TO_MLSSURFACE:
			{
				break;
			}
		}
	}
}

Eigen::Matrix4f PairwiseRegistration::registAr(Correspondences &correspondences, 
	PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters,
	PairwiseRegistrationComputationData &pairwiseRegistrationComputationData)
{
	Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_src = pairwiseRegistrationComputationData.cloud_src;
	Eigen::Matrix<float, 3, Eigen::Dynamic> &cloud_tgt = pairwiseRegistrationComputationData.cloud_tgt;

	cloud_src.resize(Eigen::NoChange, correspondences.size());
	cloud_tgt.resize(Eigen::NoChange, correspondences.size());

	for (int i = 0; i < correspondences.size(); ++i)
	{
		cloud_src(0, i) = correspondences[i].sourcePoint.x;
		cloud_src(1, i) = correspondences[i].sourcePoint.y;
		cloud_src(2, i) = correspondences[i].sourcePoint.z;

		cloud_tgt(0, i) = correspondences[i].targetPoint.x;
		cloud_tgt(1, i) = correspondences[i].targetPoint.y;
		cloud_tgt(2, i) = correspondences[i].targetPoint.z;
	}

    Eigen::Matrix4f transformation_matrix;
	switch(pairwiseRegistrationComputationParameters.method)
	{
		case UMEYAMA:
		{
			transformation_matrix = pcl::umeyama (cloud_src, cloud_tgt, 
				pairwiseRegistrationComputationParameters.allowScaling);
			break;
		}
		case SVD:
		{
			transformation_matrix = Eigen::Matrix4f::Identity();
			break;
		}
	}
	return transformation_matrix;
}

Eigen::Matrix4f PairwiseRegistration::icp(RegistrationData *target, RegistrationData *source, 
	Eigen::Matrix4f initialTransformation, CorrspondencesComputationParameters &corrspondencesComputationParameters, 
	PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, int iterationNumber)
{
	CorrspondencesComputationData correspondencesComputationData;
	PairwiseRegistrationComputationData pairwiseRegistrationComputationData;
	Correspondences correspondences;
	for (int i = 0; i < iterationNumber; ++i)
	{
		correspondences.clear();
		preCorrespondences(target, source, initialTransformation, 
			corrspondencesComputationParameters, correspondences, correspondencesComputationData);
		initialTransformation = registAr(correspondences, pairwiseRegistrationComputationParameters, 
			pairwiseRegistrationComputationData);
	}
	return initialTransformation;
}

QString PairwiseRegistration::generateName(QString targetName, QString sourceName)
{
	return targetName + "<-" + sourceName;
}

void PairwiseRegistration::reinitialize(){}  /////////////////
void PairwiseRegistration::process(QVariantMap parameters){} /////////////////
void PairwiseRegistration::initializeTransformation(Eigen::Matrix4f transformation){}///////////////////////
void PairwiseRegistration::estimateRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber){} //////////////////////////
void PairwiseRegistration::estimateVirtualRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber){} /////////////////////

PairwiseRegistrationManager::PairwiseRegistrationManager(QObject *parent) {}

PairwiseRegistrationManager::~PairwiseRegistrationManager() {}

PairwiseRegistration *PairwiseRegistrationManager::addPairwiseRegistration(RegistrationData *target, 
	RegistrationData *source, QString registrationName)
{
	return new PairwiseRegistration(target, source, registrationName, this);
}

void PairwiseRegistrationManager::removePairwiseRegistration(QString registrationName)
{
	PairwiseRegistration *pairwiseRegistration = findChild<PairwiseRegistration*>(registrationName);
	if(pairwiseRegistration) delete pairwiseRegistration;
}

PairwiseRegistration *PairwiseRegistrationManager::getPairwiseRegistration(QString registrationName)
{
	return findChild<PairwiseRegistration*>(registrationName);
}