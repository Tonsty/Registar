 #include <QtCore/QFileInfo>

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

	float angleThreshold = 10.0f;
	pcl::console::parse_argument (argc, argv, "--angle", angleThreshold);
	angleThreshold = angleThreshold / 180 * M_PI;

	float distanceThreshold = 0.01f;
	pcl::console::parse_argument (argc, argv, "--distance", distanceThreshold);

	QList<Cloud*> cloudList = cloudManager->getAllClouds();
	for (int i = 0; i < cloudList.size(); ++i)
	{
		Cloud *cloud = cloudList[i];	
		Eigen::Matrix4f transformation = cloud->getTransformation();
		Eigen::Matrix4f randomRigidTransf = randomRigidTransformation(angleThreshold, distanceThreshold);
		cloud->setTransformation(randomRigidTransf * transformation);
		std::cout << randomRigidTransf << std::endl;
	}

	for (int i = 0; i < cloudList.size(); ++i)
	{
		Cloud *cloud = cloudList[i];		
		QString cloudName = cloud->getCloudName();
		CloudDataPtr cloudData = cloud->getCloudData();
		Eigen::Matrix4f transformation = cloud->getTransformation();
		BoundariesPtr boundaries = cloud->getBoundaries();
		QString fileName =  cloud->getFileName();
		QFileInfo fileInfo(fileName);
		QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_out.ply";
		CloudIO::exportCloudData(newFileName, cloudData);
		CloudIO::exportTransformation(newFileName, transformation);
		CloudIO::exportBoundaries(newFileName, boundaries);

		CloudIO::exportCloudData(newFileName, cloudData);
	}

	return 0;
}