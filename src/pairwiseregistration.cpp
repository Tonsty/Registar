#include <QtCore/QDebug>
#include <QtCore/QStringList>
#include <pcl/common/transforms.h>

#include "../include/mathutilities.h"
#include "../include/pairwiseregistration.h"

using namespace registar;

PairwiseRegistration::PairwiseRegistration(RegistrationData *target, RegistrationData *source, 
	QString registrationName, QObject *parent) : QObject(parent)
{
	this->target = target;
	this->source = source;

	this->setObjectName(registrationName);

	transformation = Eigen::Matrix4f::Identity();

	freezed = false;

	errorPrecomputed = false;
	rmsError_total = 0.0f;
	squareErrors_total.clear();

	// method = POINT_TO_POINT;
	// distanceThreshold = std::numeric_limits<float>::max();
	// normalAngleThreshold = 180.0f;
	// boundaryTest = false;
	// allowScaling = false;
	// biDirectional = false;
}

PairwiseRegistration::~PairwiseRegistration() {}

void PairwiseRegistration::initialize()
{
	transformation = Eigen::Matrix4f::Identity();

	freezed = false;

	errorPrecomputed = false;
	rmsError_total = 0.0f;
	squareErrors_total.clear();
}

void PairwiseRegistration::initializeTransformation(const Eigen::Matrix4f &transformation)
{
	this->transformation = transformation;

	freezed = false;
	
	errorPrecomputed = false;
	rmsError_total = 0.0f;
	squareErrors_total.clear();
}

void PairwiseRegistration::process(QVariantMap parameters) {}

void PairwiseRegistration::estimateRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &rmsError, int &ovlNumber)
{
	CorrespondencesComputationParameters correspondencesComputationParameters;

	correspondencesComputationParameters.method = POINT_TO_PLANE;
	correspondencesComputationParameters.distanceThreshold = 0.12f;
	correspondencesComputationParameters.normalAngleThreshold = 45.0f;
	correspondencesComputationParameters.boundaryTest = true;
	correspondencesComputationParameters.biDirectional = true;

	CorrespondencesComputationData correspondencesComputationData;
	Correspondences correspondences;
	CorrespondenceIndices correspondenceIndices;
	int inverseStartIndex;
	preCorrespondences(target, source, transformation, 
		correspondencesComputationParameters, correspondences, 
		correspondenceIndices, inverseStartIndex, correspondencesComputationData);

	float rmsError_total;
	std::vector<float> squareErrors_total;
	computeSquareErrors(correspondences, squareErrors_total, rmsError_total);

	rmsError = rmsError_total;
	ovlNumber = squareErrors_total.size();
}

void PairwiseRegistration::estimateVirtualRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &rmsError, int &ovlNumber)
{
	CorrespondencesComputationParameters correspondencesComputationParameters;

	correspondencesComputationParameters.method = POINT_TO_PLANE;
	correspondencesComputationParameters.distanceThreshold = 0.12f;
	correspondencesComputationParameters.normalAngleThreshold = 45.0f;
	correspondencesComputationParameters.boundaryTest = true;
	correspondencesComputationParameters.biDirectional = true;	

	CorrespondencesComputationData correspondencesComputationData;
	Correspondences correspondences;
	CorrespondenceIndices correspondenceIndices;
	int inverseStartIndex;
	preCorrespondences(target, source, this->transformation, 
		correspondencesComputationParameters, correspondences, 
		correspondenceIndices, inverseStartIndex, correspondencesComputationData);

	for (int i = 0; i < inverseStartIndex; ++i)
	{
		correspondences[i].targetPoint = transformPointWithNormal(correspondences[i].sourcePoint, transformation * this->transformation.inverse());
	}
	for (int i = inverseStartIndex; i < correspondences.size(); ++i)
	{
		correspondences[i].sourcePoint = transformPointWithNormal(correspondences[i].targetPoint, this->transformation.inverse());
		correspondences[i].targetPoint = transformPointWithNormal(correspondences[i].targetPoint, transformation.inverse());
	}

	float rmsError_total;
	std::vector<float> squareErrors_total;
	computeSquareErrors(correspondences, squareErrors_total, rmsError_total);

	rmsError = rmsError_total;
	ovlNumber = squareErrors_total.size();
} 

void PairwiseRegistration::preCorrespondences(RegistrationData *target, RegistrationData *source,
	const Eigen::Matrix4f &initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
	Correspondences &correspondences, CorrespondenceIndices &correspondenceIndices, 
	int &inverseStartIndex, CorrespondencesComputationData &correspondencesComputationData)
{
	if(correspondencesComputationParameters.biDirectional)
	{
		CorrespondencesComputationParameters correspondencesComputationParameters_temp = correspondencesComputationParameters;
		correspondencesComputationParameters_temp.biDirectional = false;

		int inverseStartIndex_temp1;
		preCorrespondences(target, source, initialTransformation, 
			correspondencesComputationParameters_temp, correspondences, 
			correspondenceIndices, inverseStartIndex_temp1, correspondencesComputationData);

		inverseStartIndex = correspondences.size();

		int inverseStartIndex_temp2;
		preCorrespondences(source, target, initialTransformation.inverse(),
			correspondencesComputationParameters_temp, correspondences, 
			correspondenceIndices, inverseStartIndex_temp2, correspondencesComputationData);

		for (int i = inverseStartIndex; i < correspondences.size(); ++i)
		{
			Correspondence correspondence = correspondences[i];
			Correspondence correspondence_inverse;
			correspondence_inverse.sourcePoint = transformPointWithNormal(correspondence.targetPoint, initialTransformation);
			correspondence_inverse.targetPoint = transformPointWithNormal(correspondence.sourcePoint, initialTransformation);
			correspondences[i] = correspondence_inverse;

			CorrespondenceIndex correspondenceIndex = correspondenceIndices[i];
			CorrespondenceIndex correspondenceIndex_inverse;
			correspondenceIndex_inverse.sourceIndex = correspondenceIndex.targetIndex;
			correspondenceIndex_inverse.targetIndex = correspondenceIndex.sourceIndex;
			correspondenceIndices[i] = correspondenceIndex_inverse;
		}
	}
	else
	{
		CloudData &cloudData_target = *target->cloudData;
		KdTreePtr tree_target = target->kdTree;
		BoundariesConstPtr boundaries_target = target->boundaries;
		CloudData &cloudData_source = *source->cloudData;

		CloudData &cloudData_source_dynamic = correspondencesComputationData.cloudData_source_dynamic;		
		pcl::transformPointCloudWithNormals(cloudData_source, cloudData_source_dynamic, initialTransformation);

		pcl::Correspondences &pcl_correspondences = correspondencesComputationData.pcl_correspondences;
		pcl::Correspondences &pcl_correspondences_temp = correspondencesComputationData.pcl_correspondences_temp;

		bool use_omp = true;
		if ( use_omp )
		{
			int _threads = omp_get_num_procs();
			std::cout << _threads << " threads" << std::endl;
			std::vector<pcl::Correspondences> pcl_correspondences_temp_in_threads(_threads);

			pcl_correspondences_temp.clear();
			#pragma omp parallel for schedule (dynamic,1000) num_threads (_threads)
			for (int i = 0; i < cloudData_source_dynamic.size(); ++i)
			{
				int tn = omp_get_thread_num();

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
					pcl_correspondences_temp_in_threads[tn].push_back(temp);
				}
			}
			for (int i = 0; i < pcl_correspondences_temp_in_threads.size(); i++)
			{
				pcl_correspondences_temp.insert(pcl_correspondences_temp.end(), pcl_correspondences_temp_in_threads[i].begin(), pcl_correspondences_temp_in_threads[i].end());
				pcl_correspondences_temp_in_threads[i].clear();
			}
			pcl_correspondences.swap(pcl_correspondences_temp);

			//std::cerr << pcl_correspondences.size() << std::endl;

			float normalAngleThreshold = correspondencesComputationParameters.normalAngleThreshold;
			float NAthreshold = cosf(normalAngleThreshold / 180.0f * M_PI);
			float distanceThreshold = correspondencesComputationParameters.distanceThreshold;
			float distanceThreshold2 = distanceThreshold * distanceThreshold;
			pcl_correspondences_temp.clear();
			#pragma omp parallel for schedule (dynamic,1000) num_threads (_threads)
			for (int i = 0; i < pcl_correspondences.size(); ++i)
			{
				int tn = omp_get_thread_num();

				if (pcl_correspondences[i].distance < distanceThreshold2)
				{
					int index_query = pcl_correspondences[i].index_query;
					int index_match = pcl_correspondences[i].index_match;

					Eigen::Vector3f query_normal = cloudData_source_dynamic[index_query].getNormalVector3fMap();
					Eigen::Vector3f match_normal = cloudData_target[index_match].getNormalVector3fMap();
					query_normal.normalize();
					match_normal.normalize();

					if (query_normal.dot(match_normal) > NAthreshold)
						pcl_correspondences_temp_in_threads[tn].push_back(pcl_correspondences[i]);
				}
			}
			for (int i = 0; i < pcl_correspondences_temp_in_threads.size(); i++)
			{
				pcl_correspondences_temp.insert(pcl_correspondences_temp.end(), pcl_correspondences_temp_in_threads[i].begin(), pcl_correspondences_temp_in_threads[i].end());
				pcl_correspondences_temp_in_threads[i].clear();
			}
			pcl_correspondences.swap(pcl_correspondences_temp);

			//std::cerr << pcl_correspondences.size() << std::endl;

			if (correspondencesComputationParameters.boundaryTest)
			{
				pcl_correspondences_temp.clear();
				#pragma omp parallel for schedule (dynamic,1000) num_threads (_threads)
				for (int i = 0; i < pcl_correspondences.size(); ++i)
				{
					int tn = omp_get_thread_num();
					int index_match = pcl_correspondences[i].index_match;
					if ((*boundaries_target)[index_match].boundary_point == 0)
						pcl_correspondences_temp_in_threads[tn].push_back(pcl_correspondences[i]);
				}
				for (int i = 0; i < pcl_correspondences_temp_in_threads.size(); i++)
				{
					pcl_correspondences_temp.insert(pcl_correspondences_temp.end(), pcl_correspondences_temp_in_threads[i].begin(), pcl_correspondences_temp_in_threads[i].end());
					pcl_correspondences_temp_in_threads[i].clear();
				}
				pcl_correspondences.swap(pcl_correspondences_temp);
			}

			//std::cerr << pcl_correspondences.size() << std::endl;

			CorrespondenceComputationMethod method = correspondencesComputationParameters.method;
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

						CorrespondenceIndex correspondenceIndex_temp;
						correspondenceIndex_temp.sourceIndex = query;
						correspondenceIndex_temp.targetIndex = match;
						correspondenceIndices.push_back(correspondenceIndex_temp);
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


						CorrespondenceIndex correspondenceIndex_temp;
						correspondenceIndex_temp.sourceIndex = query;
						correspondenceIndex_temp.targetIndex = match;
						correspondenceIndices.push_back(correspondenceIndex_temp);
					}
					break;
				}
			case POINT_TO_MLSSURFACE:
				{
					break;
				}
			}
		}
		else
		{
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

			//std::cerr << pcl_correspondences.size() << std::endl;

			float normalAngleThreshold = correspondencesComputationParameters.normalAngleThreshold;
			float NAthreshold = cosf(normalAngleThreshold / 180.0f * M_PI);
			float distanceThreshold = correspondencesComputationParameters.distanceThreshold;
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

			//std::cerr << pcl_correspondences.size() << std::endl;

			if (correspondencesComputationParameters.boundaryTest)
			{
				pcl_correspondences_temp.clear();
				for (int i = 0; i < pcl_correspondences.size(); ++i)
				{
					int index_match = pcl_correspondences[i].index_match;
					if ((*boundaries_target)[index_match].boundary_point == 0)
						pcl_correspondences_temp.push_back(pcl_correspondences[i]);
				}
				pcl_correspondences.swap(pcl_correspondences_temp);
			}

			//std::cerr << pcl_correspondences.size() << std::endl;

			CorrespondenceComputationMethod method = correspondencesComputationParameters.method;
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

						CorrespondenceIndex correspondenceIndex_temp;
						correspondenceIndex_temp.sourceIndex = query;
						correspondenceIndex_temp.targetIndex = match;
						correspondenceIndices.push_back(correspondenceIndex_temp);
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


						CorrespondenceIndex correspondenceIndex_temp;
						correspondenceIndex_temp.sourceIndex = query;
						correspondenceIndex_temp.targetIndex = match;
						correspondenceIndices.push_back(correspondenceIndex_temp);
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
}

Eigen::Matrix4f PairwiseRegistration::registAr(Correspondences &correspondences, 
	PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters,
	PairwiseRegistrationComputationData &pairwiseRegistrationComputationData)
{
	if ( correspondences.size() <= 0)
	{
		return Eigen::Matrix4f::Identity();
	}
	else
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
}

Eigen::Matrix4f PairwiseRegistration::icp(RegistrationData *target, RegistrationData *source, 
	const Eigen::Matrix4f &initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
	PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, int iterationNumber)
{
	CorrespondencesComputationData correspondencesComputationData;
	PairwiseRegistrationComputationData pairwiseRegistrationComputationData;
	Correspondences correspondences;
	CorrespondenceIndices correspondenceIndices;
	int inverseStartIndex;
	Eigen::Matrix4f transformation_temp = initialTransformation;
	for (int i = 0; i < iterationNumber; ++i)
	{
		correspondences.clear();
		preCorrespondences(target, source, transformation_temp, 
			correspondencesComputationParameters, correspondences, 
			correspondenceIndices, inverseStartIndex, correspondencesComputationData);
		transformation_temp = registAr(correspondences, pairwiseRegistrationComputationParameters, 
			pairwiseRegistrationComputationData) * transformation_temp;
	}
	return transformation_temp;
}

void PairwiseRegistration::computeSquareErrors(Correspondences &correspondences, std::vector<float> &squareErrors_total, float &rmsError_total)
{
	squareErrors_total.clear();
	for (int i = 0; i < correspondences.size(); ++i)
	{
		Eigen::Vector3f xyz_target = correspondences[i].targetPoint.getVector3fMap();
		Eigen::Vector3f xyz_source = correspondences[i].sourcePoint.getVector3fMap();

		squareErrors_total.push_back((xyz_target - xyz_source).squaredNorm());
	}

	float mean_total = 0.0f;
	for (int i = 0; i < squareErrors_total.size(); ++i) mean_total += squareErrors_total[i];
	mean_total /= squareErrors_total.size();
	rmsError_total = sqrtf(mean_total);
}

PairwiseRegistrationManager::PairwiseRegistrationManager(QObject *parent) {}

PairwiseRegistrationManager::~PairwiseRegistrationManager() {}

PairwiseRegistration *PairwiseRegistrationManager::addPairwiseRegistration(RegistrationData *target, RegistrationData *source, QString registrationName)
{
	return new PairwiseRegistration(target, source, registrationName, this);
}

void PairwiseRegistrationManager::addPairwiseRegistration(PairwiseRegistration *pairwiseRegistration)
{
	pairwiseRegistration->setParent(this);
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

QList<PairwiseRegistration*> PairwiseRegistrationManager::getAllPairwiseRegistrations()
{
	return findChildren<PairwiseRegistration*>();
}

QStringList PairwiseRegistrationManager::getAllPairwiseRegistrationNames()
{
	QStringList allPairwiseRegistrationNames;
	QList<PairwiseRegistration*> allPairwiseRegistrations = getAllPairwiseRegistrations();
	for (int i = 0; i < allPairwiseRegistrations.size(); ++i)
	{
		allPairwiseRegistrationNames << allPairwiseRegistrations[i]->objectName();
	}
	return allPairwiseRegistrationNames;
}