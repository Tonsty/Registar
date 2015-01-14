#include <pcl/common/transforms.h>

#include "../include/cloudvisualizer.h"
#include "../include/pairwiseregistrationinteractor.h"

using namespace registar;

PairwiseRegistrationInteractor::PairwiseRegistrationInteractor(RegistrationData *target, RegistrationData *source,
	QString registrationName, QObject *parent) : PairwiseRegistration(target, source, registrationName, parent)
{
	cloudVisualizer = NULL;
}

PairwiseRegistrationInteractor::~PairwiseRegistrationInteractor() {}

void PairwiseRegistrationInteractor::initialize()
{
	PairwiseRegistration::initialize();

	if(cloudVisualizer) cloudVisualizer->updateCloud(target->cloudData, "target", 0, 0, 255);
	if(cloudVisualizer) cloudVisualizer->updateCloud(source->cloudData, "source", 255, 0, 0);	
}

void PairwiseRegistrationInteractor::initializeTransformation(const Eigen::Matrix4f &transformation)
{
	PairwiseRegistration::initializeTransformation(transformation);

    CloudDataPtr cloudData_target_temp(new CloudData);
    pcl::copyPointCloud(*target->cloudData, *cloudData_target_temp);
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_target_temp, "target", 0, 0, 255);

	CloudDataPtr cloudData_source_temp(new CloudData);
	pcl::transformPointCloudWithNormals(*source->cloudData, *cloudData_source_temp, transformation);
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_source_temp, "source", 255, 0, 0);	
}

void PairwiseRegistrationInteractor::exportTransformation()
{
	source->cloud->setRegistrationTransformation(transformation * source->cloud->getTransformation());
	//transformation = Eigen::Matrix4f::Identity();
}

void PairwiseRegistrationInteractor::process(QVariantMap parameters)
{
	QString command = parameters["command"].toString();

	if (command == "Pre-Correspondences")
	{
		CorrespondencesComputationParameters correspondencesComputationParameters;

		correspondencesComputationParameters.method = (CorrespondenceComputationMethod)parameters["method"].toInt();
		correspondencesComputationParameters.distanceThreshold = parameters["distanceThreshold"].toFloat();
		correspondencesComputationParameters.normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
		correspondencesComputationParameters.boundaryTest = parameters["boundaryTest"].toBool();
		correspondencesComputationParameters.biDirectional = parameters["biDirectional"].toBool();
		correspondencesComputationParameters.use_scpu = parameters["use_scpu"].toBool();
		correspondencesComputationParameters.use_mcpu = parameters["use_mcpu"].toBool();

		CorrespondencesComputationData correspondencesComputationData;
		Correspondences correspondences;
		CorrespondenceIndices correspondenceIndices;
		int inverseStartIndex;
		preCorrespondences(target, source, transformation, 
			correspondencesComputationParameters, correspondences, 
			correspondenceIndices, inverseStartIndex, correspondencesComputationData);

		computeSquareErrors(correspondences, squareErrors_total, rmsError_total);
		renderErrorMap(correspondenceIndices, inverseStartIndex, squareErrors_total, true);				
	}
	else if (command == "ICP")
	{
		CorrespondencesComputationParameters correspondencesComputationParameters;

		correspondencesComputationParameters.method = (CorrespondenceComputationMethod)parameters["method"].toInt();
		correspondencesComputationParameters.distanceThreshold = parameters["distanceThreshold"].toFloat();
		correspondencesComputationParameters.normalAngleThreshold = parameters["normalAngleThreshold"].toFloat();
		correspondencesComputationParameters.boundaryTest = parameters["boundaryTest"].toBool();
		correspondencesComputationParameters.biDirectional = parameters["biDirectional"].toBool();
		correspondencesComputationParameters.use_scpu = parameters["use_scpu"].toBool();
		correspondencesComputationParameters.use_mcpu = parameters["use_mcpu"].toBool();

		PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters;

		pairwiseRegistrationComputationParameters.method = UMEYAMA;
		pairwiseRegistrationComputationParameters.allowScaling = parameters["allowScaling"].toBool();
		int iterationNumber = parameters["icpNumber"].toInt();

		Eigen::Matrix4f transformation_temp = icp(target, source, transformation, 
			correspondencesComputationParameters, pairwiseRegistrationComputationParameters, iterationNumber);

		initializeTransformation(transformation_temp);

		CorrespondencesComputationData correspondencesComputationData;
		Correspondences correspondences;
		CorrespondenceIndices correspondenceIndices;
		int inverseStartIndex;		
		preCorrespondences(target, source, transformation, 
			correspondencesComputationParameters, correspondences, 
			correspondenceIndices, inverseStartIndex, correspondencesComputationData);

		computeSquareErrors(correspondences, squareErrors_total, rmsError_total);		
		renderErrorMap(correspondenceIndices, inverseStartIndex, squareErrors_total, false);
	}
	else if (command == "Export")
	{
		exportTransformation();
	}
}

void PairwiseRegistrationInteractor::renderErrorMap(CorrespondenceIndices &correspondenceIndices, int &inverseStartIndex, std::vector<float> &squareErrors_total, bool mapping)
{
	Eigen::Vector3f red(250, 0, 0);
	Eigen::Vector3f green(0, 250, 0);
	Eigen::Vector3f blue(0, 0, 250);
    float redError = 0.02f;
    float greenError = 0.002f ;
    float blueError = 0.0f;

	if (correspondenceIndices.size() > inverseStartIndex)
	{
		CloudDataPtr cloudData_target_temp(new CloudData);
		pcl::copyPointCloud(*target->cloudData, *cloudData_target_temp);
		for (int i = 0; i < cloudData_target_temp->size(); ++i) 
		{
			(*cloudData_target_temp)[i].r = 0;
			(*cloudData_target_temp)[i].g = 0;
			(*cloudData_target_temp)[i].b = 250;
			//(*cloudData_target_temp)[i].getRGBVector3i() = blue.cast<int>();
		}
		//qDebug() << QString::number(inverseStartIndex);
		//qDebug() << QString::number(correspondenceIndices.size() - inverseStartIndex);
		if (mapping)
		{
			for (int i = inverseStartIndex; i < correspondenceIndices.size(); ++i)
			{
				int index = correspondenceIndices[i].targetIndex;
				//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors_total[i]);
				float error = sqrtf(squareErrors_total[i]);
				if (error >= redError ) 
				{
					(*cloudData_target_temp)[index].r = red[0];
					(*cloudData_target_temp)[index].g = red[1];
					(*cloudData_target_temp)[index].b = red[2];
					continue;
				}
				if (error >= greenError)
				{
					float fraction = (error - greenError) / (redError - greenError);
					Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
					(*cloudData_target_temp)[index].r = color[0];
					(*cloudData_target_temp)[index].g = color[1];
					(*cloudData_target_temp)[index].b = color[2];  		
					continue;
				}
				if (error >= blueError)
				{
					float fraction = (error - blueError) / (greenError - blueError);
					Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
					(*cloudData_target_temp)[index].r = color[0];
					(*cloudData_target_temp)[index].g = color[1];
					(*cloudData_target_temp)[index].b = color[2];  
					continue;
				}
			}
		}
		if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_target_temp, "target");
	}

	CloudDataPtr cloudData_source_temp(new CloudData);
	pcl::transformPointCloudWithNormals(*source->cloudData, *cloudData_source_temp, transformation);
	if (mapping)
	{
		for (int i = 0; i < cloudData_source_temp->size(); ++i)
		{
			(*cloudData_source_temp)[i].r = 0;
			(*cloudData_source_temp)[i].g = 0;	
			(*cloudData_source_temp)[i].b = 250;
			//(*cloudData_target_temp)[i].getRGBVector3i() = blue.cast<int>();
		} 
		for (int i = 0; i < inverseStartIndex; ++i)
		{
			int index = correspondenceIndices[i].sourceIndex;
			//qDebug() << QString::number(index) << " " <<  QString::number(squareErrors_total[i]);
			float error = sqrtf(squareErrors_total[i]);
			if (error >= redError) 
			{
				(*cloudData_source_temp)[index].r = red[0];
				(*cloudData_source_temp)[index].g = red[1];
				(*cloudData_source_temp)[index].b = red[2];    		
				continue;
			}
			if (error >= greenError)
			{
				float fraction = (error - greenError) / (redError - greenError);
				Eigen::Vector3f color = red * fraction + green * ( 1.0f -fraction);
				(*cloudData_source_temp)[index].r = color[0];
				(*cloudData_source_temp)[index].g = color[1];   
				(*cloudData_source_temp)[index].b = color[2]; 		
				continue;
			}
			if (error >= blueError)
			{
				float fraction = (error - blueError) / (greenError - blueError);
				Eigen::Vector3f color = green * fraction + blue * ( 1.0f -fraction);
				(*cloudData_source_temp)[index].r = color[0];
				(*cloudData_source_temp)[index].g = color[1];   
				(*cloudData_source_temp)[index].b = color[2]; 
				continue;
			}
		}
	}
	else
	{
		for (int i = 0; i < cloudData_source_temp->size(); ++i)
		{
			(*cloudData_source_temp)[i].r = 250;
			(*cloudData_source_temp)[i].g = 0;	
			(*cloudData_source_temp)[i].b = 0;
			//(*cloudData_target_temp)[i].getRGBVector3i() = blue.cast<int>();
		} 
	}
	if(cloudVisualizer) cloudVisualizer->updateCloud(cloudData_source_temp, "source");

}