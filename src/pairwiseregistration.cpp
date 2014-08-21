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

	transformation = Eigen::Matrix4f::Identity();

	// method = POINT_TO_POINT;
	// distanceThreshold = std::numeric_limits<float>::max();
	// normalAngleThreshold = 180.0f;
	// boundaryTest = false;
	// allowScaling = false;
	// biDirectional = false;
}

PairwiseRegistration::~PairwiseRegistration() {}


void PairwiseRegistration::preCorrespondences(RegistrationData *target, RegistrationData *source,
	Eigen::Matrix4f initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
	Correspondences &correspondences, CorrespondencesComputationData &correspondencesComputationData)
{
	if(correspondencesComputationParameters.biDirectional)
	{
		CorrespondencesComputationParameters correspondencesComputationParameters_temp = correspondencesComputationParameters;
		correspondencesComputationParameters_temp.biDirectional = false;

		preCorrespondences(target, source, initialTransformation, 
			correspondencesComputationParameters_temp, correspondences, correspondencesComputationData);

		int inverse_correspondences_start = correspondences.size();

		preCorrespondences(source, target, initialTransformation.inverse(),
			correspondencesComputationParameters_temp, correspondences, correspondencesComputationData);

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
	Eigen::Matrix4f initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
	PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, int iterationNumber)
{
	CorrespondencesComputationData correspondencesComputationData;
	PairwiseRegistrationComputationData pairwiseRegistrationComputationData;
	Correspondences correspondences;
	for (int i = 0; i < iterationNumber; ++i)
	{
		correspondences.clear();
		preCorrespondences(target, source, initialTransformation, 
			correspondencesComputationParameters, correspondences, correspondencesComputationData);
		initialTransformation = registAr(correspondences, pairwiseRegistrationComputationParameters, 
			pairwiseRegistrationComputationData) * initialTransformation;
	}
	return initialTransformation;
}

QString PairwiseRegistration::generateName(QString targetName, QString sourceName)
{
	return targetName + "<-" + sourceName;
}

void PairwiseRegistration::reinitialize()///////////////////////////////////////////////////////////////////////////////////////////////////////////
{
	transformation = Eigen::Matrix4f::Identity();
	
	if(cloudVisualizer) cloudVisualizer->updateCloud(target->cloudData, "target", 0, 0, 255);
	if(cloudVisualizer) cloudVisualizer->updateCloud(source->cloudData, "source", 255, 0, 0);
}

void PairwiseRegistration::initializeTransformation(Eigen::Matrix4f transformation)//////////////////////////////////////////////////////////////////
{
	this->transformation = transformation;

	CloudDataPtr cloudData_temp(new CloudData);
	pcl::transformPointCloudWithNormals(*source->cloudData, *cloudData_temp, this->transformation);
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_temp, "source", 255, 0, 0);
}

void PairwiseRegistration::process(QVariantMap parameters)//////////////////////////////////////////////////////////////////////////////////////////
{
	QString command = parameters["command"].toString();

	if (command == "Pre-Correspondences")
	{
		CorrespondencesComputationParameters correspondencesComputationParameters;

		correspondencesComputationParameters.method = (CorrespondenceComputationMethod)parameters["method"].toInt();
		correspondencesComputationParameters.distanceThreshold = parameters["distanceThreshold"].toFloat();
		correspondencesComputationParameters.normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
		correspondencesComputationParameters.boundaryTest = parameters["boundaryTest"].toBool();
		correspondencesComputationParameters.biDirectional = true;

		CorrespondencesComputationData correspondencesComputationData;
		Correspondences correspondences;
		preCorrespondences(target, source, transformation, 
			correspondencesComputationParameters, correspondences, correspondencesComputationData);

		computeSquareErrors(correspondences, squareErrors_total, rmsError_total);
		//renderErrorMap();				
	}
	else if (command == "ICP")
	{
		CorrespondencesComputationParameters correspondencesComputationParameters;

		correspondencesComputationParameters.method = (CorrespondenceComputationMethod)parameters["method"].toInt();
		correspondencesComputationParameters.distanceThreshold = parameters["distanceThreshold"].toFloat();
		correspondencesComputationParameters.normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
		correspondencesComputationParameters.boundaryTest = parameters["boundaryTest"].toBool();
		correspondencesComputationParameters.biDirectional = true;

		PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters;

		pairwiseRegistrationComputationParameters.method = UMEYAMA;
		pairwiseRegistrationComputationParameters.allowScaling = parameters["allowScaling"].toBool();
		int iterationNumber = parameters["icpNumber"].toInt();

		transformation = icp(target, source, transformation, 
			correspondencesComputationParameters, pairwiseRegistrationComputationParameters, iterationNumber);

		initializeTransformation(transformation);

		CorrespondencesComputationData correspondencesComputationData;
		Correspondences correspondences;
		preCorrespondences(target, source, transformation, 
			correspondencesComputationParameters, correspondences, correspondencesComputationData);

		computeSquareErrors(correspondences, squareErrors_total, rmsError_total);		
		//renderErrorMap();
	}
}

void PairwiseRegistration::estimateRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber){} //////////////////////////
void PairwiseRegistration::estimateVirtualRMSErrorByTransformation(Eigen::Matrix4f transformation, float &rmsError, int &ovlNumber){} /////////////////////

void PairwiseRegistration::computeSquareErrors(Correspondences &correspondences, std::vector<float> &squareErrors_total, float &rmsError_total)////////////////////
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

void PairwiseRegistration::renderErrorMap()
{
	Eigen::Vector3f red(250, 0, 0);
	Eigen::Vector3f green(0, 250, 0);
	Eigen::Vector3f blue(0, 0, 250);
	// float redError = 0.005f;
 //    float greenError = 0.0001f ;
 //    float blueError = 0.0f;

    CloudDataPtr cloudData_target_temp(new CloudData);
    pcl::copyPointCloud(*target->cloudData, *cloudData_target_temp);

    CloudDataPtr cloudData_source_temp(new CloudData);
    pcl::transformPointCloudWithNormals(*source->cloudData, *cloudData_source_temp, transformation);

	for (int i = 0; i < cloudData_target_temp->size(); ++i) 
	{
		(*cloudData_target_temp)[i].r = 0;
		(*cloudData_target_temp)[i].g = 0;
		(*cloudData_target_temp)[i].b = 250;
		//(*cloudData_target_temp)[i].getRGBVector3i() = blue.cast<int>();
	}
	for (int i = 0; i < cloudData_source_temp->size(); ++i)
	{
		(*cloudData_source_temp)[i].r = 0;
		(*cloudData_source_temp)[i].g = 0;	
		(*cloudData_source_temp)[i].b = 250;
		//(*cloudData_target_temp)[i].getRGBVector3i() = blue.cast<int>();
	} 

	// qDebug() << QString::number(correspondences_inverse->size());
	// qDebug() << QString::number(correspondences->size());

 //    for (int i = 0; i < correspondences_inverse->size(); ++i)
 //    {
 //    	int index = (*correspondences_inverse)[i].index_query;
 //    	//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors_inverse[i]);
 //    	float error = sqrtf(squareErrors_inverse[i]);
 //    	if (error >= redError ) 
 //    	{
 //    		(*cloudData_target)[index].r = red[0];
 //    		(*cloudData_target)[index].g = red[1];
 //    		(*cloudData_target)[index].b = red[2];
 //    		continue;
 //    	}
 //    	if (error >= greenError)
 //    	{
 //    		float fraction = (error - greenError) / (redError - greenError);
 //    		Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
 //    		(*cloudData_target)[index].r = color[0];
 //    		(*cloudData_target)[index].g = color[1];
 //      		(*cloudData_target)[index].b = color[2];  		
 //    		continue;
 //    	}
 //    	if (error >= blueError)
 //    	{
 //    		float fraction = (error - blueError) / (greenError - blueError);
 //    		Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
 //    		(*cloudData_target)[index].r = color[0];
 //    		(*cloudData_target)[index].g = color[1];
 //      		(*cloudData_target)[index].b = color[2];  
 //    		continue;
 //    	}
 //    }

 //    for (int i = 0; i < correspondences->size(); ++i)
 //    {
 //    	int index = (*correspondences)[i].index_query;
 //    	//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors[i]);
 //    	float error = sqrtf(squareErrors[i]);
 //    	if (error >= redError) 
 //    	{
 //    		(*cloudData_source_dynamic)[index].r = red[0];
 //    		(*cloudData_source_dynamic)[index].g = red[1];
 //    		(*cloudData_source_dynamic)[index].b = red[2];    		
 //    		continue;
 //    	}
 //    	if (error >= greenError)
 //    	{
 //    		float fraction = (error - greenError) / (redError - greenError);
 //    		Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
 //    		(*cloudData_source_dynamic)[index].r = color[0];
 //    		(*cloudData_source_dynamic)[index].g = color[1];   
 //    		(*cloudData_source_dynamic)[index].b = color[2]; 		
 //    		continue;
 //    	}
 //    	if (error >= blueError)
 //    	{
 //    		float fraction = (error - blueError) / (greenError - blueError);
 //    		Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
 //    		(*cloudData_source_dynamic)[index].r = color[0];
 //    		(*cloudData_source_dynamic)[index].g = color[1];   
 //    		(*cloudData_source_dynamic)[index].b = color[2]; 
 //    		continue;
 //    	}
 //    }

	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_target_temp, "target");
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_source_temp, "source");
}


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