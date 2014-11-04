#include <QtCore/QFileInfo>

#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>

#include "../include/pclbase.h"
#include "../include/qtbase.h"
#include "../include/cloud.h"
#include "../include/cloudmanager.h"
#include "../include/cloudio.h"
#include "../include/pairwiseregistration.h"
#include "../include/globalregistration.h"
#include "../include/mathutilities.h"

int main(int argc, char **argv)
{
	CloudManager* cloudManager = new CloudManager();
	PairwiseRegistrationManager* pairwiseRegistrationManager = new PairwiseRegistrationManager();
	RegistrationDataManager* registrationDataManager = new RegistrationDataManager();
	//CycleRegistrationManager *cycleRegistrationManager = new CycleRegistrationManager();
	
  	std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
	for (int i = 0; i < p_file_indices_ply.size(); ++i)
	{
		CloudDataPtr cloudData(new CloudData);
		Eigen::Matrix4f transformation;
		BoundariesPtr boundaries(new Boundaries);

		QString fileName = QString(argv[p_file_indices_ply[i]]);
		CloudIO::importCloudData(fileName, cloudData);
		CloudIO::importTransformation(fileName, transformation);
		CloudIO::importBoundaries(fileName, boundaries);
		Cloud* cloud = cloudManager->addCloud(cloudData, Cloud::fromIO, fileName, transformation);
		cloud->setBoundaries(boundaries);
	}

	std::vector< std::vector<int> > overlapRelations( cloudManager->getAllClouds().size() );
	overlapRelations[0].push_back(1);
	overlapRelations[0].push_back(2);
	overlapRelations[0].push_back(5);
	overlapRelations[0].push_back(6);
	overlapRelations[0].push_back(9);

	// overlapRelations[1].push_back(0);
	// overlapRelations[1].push_back(3);
	// overlapRelations[3].push_back(1);
	// overlapRelations[3].push_back(2);
	// overlapRelations[2].push_back(3);
	// overlapRelations[2].push_back(0);
	// overlapRelations[0].push_back(2);

	for (int i = 0; i < overlapRelations.size(); ++i)
	{
		for (int j = 0; j < overlapRelations[i].size(); ++j)
		{
			QString cloudName_target = QString::number(i);
			QString cloudName_source = QString::number(overlapRelations[i][j]);

			QString prName = PairwiseRegistration::generateName(cloudName_target, cloudName_source);
			PairwiseRegistration *pairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(prName);
			QString reversePrName = PairwiseRegistration::generateName(cloudName_source, cloudName_target);
			PairwiseRegistration *reversePairwiseRegistration = pairwiseRegistrationManager->getPairwiseRegistration(reversePrName);

			std::cout << i << " " << overlapRelations[i][j] << std::endl;

			if (pairwiseRegistration == NULL && reversePairwiseRegistration == NULL)
			{
				RegistrationData *registrationData_target = registrationDataManager->getRegistrationData(cloudName_target);
				if ( registrationData_target == NULL)
				{
					Cloud *cloud_target = cloudManager->getCloud(cloudName_target);
					registrationData_target = registrationDataManager->addRegistrationData(cloud_target, cloudName_target);
				}
				RegistrationData *registrationData_source = registrationDataManager->getRegistrationData(cloudName_source);
				if ( registrationData_source == NULL)
				{
					Cloud *cloud_source = cloudManager->getCloud(cloudName_source);
					registrationData_source = registrationDataManager->addRegistrationData(cloud_source, cloudName_source);
				}
				pairwiseRegistration = pairwiseRegistrationManager->addPairwiseRegistration(registrationData_target, registrationData_source, prName);
			}		
		}
	}

	std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > sumErrorCloud(registrationDataManager->getAllRegistrationDatas().size());
	std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr > maxErrorCloud(registrationDataManager->getAllRegistrationDatas().size());

	QList<RegistrationData*> registrationDataList = registrationDataManager->getAllRegistrationDatas();
	for (int i = 0; i < registrationDataList.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZI>);

		pcl::copyPointCloud(*(registrationDataList[i]->cloudData), *temp1);
		pcl::copyPointCloud(*(registrationDataList[i]->cloudData), *temp2);	

		QString dataName = registrationDataList[i]->objectName();

		sumErrorCloud[dataName.toInt()] = temp1;
		maxErrorCloud[dataName.toInt()] = temp2;
	}

	for (int i = 0; i < sumErrorCloud.size(); ++i)
	{
		for (int j = 0; j < sumErrorCloud[i]->size(); ++j)
		{
			(*sumErrorCloud[i])[j].intensity = 0.0f;
		}
	}

	for (int i = 0; i < maxErrorCloud.size(); ++i)
	{
		for (int j = 0; j < maxErrorCloud[i]->size(); ++j)
		{
			(*maxErrorCloud[i])[j].intensity = 0.0f;
		}
	}

	QList<PairwiseRegistration*> pairwiseRegistrationList = pairwiseRegistrationManager->getAllPairwiseRegistrations();
	for (int i = 0; i < pairwiseRegistrationList.size(); ++i)
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
		PairwiseRegistration::preCorrespondences(pairwiseRegistrationList[i]->getTarget(), pairwiseRegistrationList[i]->getSource(), pairwiseRegistrationList[i]->getTransformation(), 
			correspondencesComputationParameters, correspondences, 
			correspondenceIndices, inverseStartIndex, correspondencesComputationData);

		float rmsError_total;
		std::vector<float> squareErrors_total;
		PairwiseRegistration::computeSquareErrors(correspondences, squareErrors_total, rmsError_total);

		std::cout << rmsError_total << std::endl;
		std::cout << squareErrors_total.size() << std::endl;

		int targetErrorCloudIndex = pairwiseRegistrationList[i]->getTarget()->objectName().toInt();
		int sourceErrorCloudIndex = pairwiseRegistrationList[i]->getSource()->objectName().toInt();

		for (int i = inverseStartIndex; i < correspondenceIndices.size(); ++i)
		{
			int index = correspondenceIndices[i].targetIndex;
			float error = sqrtf(squareErrors_total[i]);
			(*sumErrorCloud[targetErrorCloudIndex])[index].intensity += error;
			(*maxErrorCloud[targetErrorCloudIndex])[index].intensity = error > (*maxErrorCloud[targetErrorCloudIndex])[index].intensity ? error : (*maxErrorCloud[targetErrorCloudIndex])[index].intensity;
		}

		for (int i = 0; i < inverseStartIndex; ++i)
		{
	    	int index = correspondenceIndices[i].sourceIndex;
	    	float error = sqrtf(squareErrors_total[i]);

			(*sumErrorCloud[sourceErrorCloudIndex])[index].intensity += error;
			(*maxErrorCloud[sourceErrorCloudIndex])[index].intensity = error > (*maxErrorCloud[targetErrorCloudIndex])[index].intensity ? error : (*maxErrorCloud[targetErrorCloudIndex])[index].intensity;	    	
		}
	}

	for (int i = 0; i < sumErrorCloud.size(); ++i)
	{
		QString cloudName = QString::number(i);
		Cloud* cloud = cloudManager->getCloud(cloudName);

		QString fileName =  cloud->getFileName();
		QFileInfo fileInfo(fileName);
		QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_sumerror.ply";		

		pcl::PLYWriter writer;
		writer.write(newfileName.toStdString(), *sumErrorCloud[i]);		
	}

	for (int i = 0; i < maxErrorCloud.size(); ++i)
	{
		QString cloudName = QString::number(i);
		Cloud* cloud = cloudManager->getCloud(cloudName);

		QString fileName =  cloud->getFileName();
		QFileInfo fileInfo(fileName);
		QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_maxerror.ply";		

		pcl::PLYWriter writer;
		writer.write(newFileName.toStdString(), *maxErrorCloud[i]);		
	}	

	return 0;
}