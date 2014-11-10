#include <QtCore/QFileInfo>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

#include "../include/pclbase.h"
#include "../include/qtbase.h"
#include "../include/cloud.h"
#include "../include/cloudmanager.h"
#include "../include/cloudio.h"
#include "../include/pairwiseregistration.h"
#include "../include/globalregistration.h"
#include "../include/mathutilities.h"

#include "../set_color/set_color.h"

int main(int argc, char **argv)
{	
  	std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
   	std::string output_directory = ".";
	pcl::console::parse_argument (argc, argv, "--directory", output_directory);	 

  	std::vector<RGB_> rgbs = generateUniformColors(p_file_indices_ply.size(), 60, 300);	
	
	for (int i = 0; i < p_file_indices_ply.size(); ++i)
	{
		Eigen::Matrix4f transformation;
		QString fileName = QString(argv[p_file_indices_ply[i]]);
		CloudIO::importTransformation(fileName, transformation);
	    pcl::PLYReader reader;
	    pcl::PolygonMesh mesh;
	    reader.read(argv[p_file_indices_ply[i]], mesh);

	    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudData(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	    // for (int i = 0; i < mesh.cloud.fields.size(); ++i)
	    // {
	    // 	std::cout << mesh.cloud.fields[i].name << std::endl;
	    // }

	    pcl::fromPCLPointCloud2(mesh.cloud, *cloudData);

	    pcl::transformPointCloudWithNormals(*cloudData, *cloudData, transformation);

	    for (int j = 0; j < cloudData->size(); ++j)
	    {
	    	(*cloudData)[j].r = rgbs[i].r;
	    	(*cloudData)[j].g = rgbs[i].g;
	    	(*cloudData)[j].b = rgbs[i].b;	    	
	    }

	    pcl::toPCLPointCloud2(*cloudData, mesh.cloud);   

	    // for (int i = 0; i < mesh.cloud.fields.size(); ++i)
	    // {
	    // 	std::cout << mesh.cloud.fields[i].name << std::endl;
	    // }

	    QFileInfo info(argv[p_file_indices_ply[i]]);
	    pcl::io::savePLYFile((QString(output_directory.c_str()) + "/" + info.fileName()).toStdString(), mesh);	
	}

	return 0;
}


// int main(int argc, char **argv)
// {
// 	CloudManager* cloudManager = new CloudManager();
	
//   	std::vector<int> p_file_indices_ply = pcl::console::parse_file_extension_argument (argc, argv, ".ply");	
// 	for (int i = 0; i < p_file_indices_ply.size(); ++i)
// 	{
// 		CloudDataPtr cloudData(new CloudData);
// 		Eigen::Matrix4f transformation;
// 		BoundariesPtr boundaries(new Boundaries);

// 		QString fileName = QString(argv[p_file_indices_ply[i]]);
// 		CloudIO::importCloudData(fileName, cloudData);
// 		CloudIO::importTransformation(fileName, transformation);
// 		CloudIO::importBoundaries(fileName, boundaries);
// 		Cloud* cloud = cloudManager->addCloud(cloudData, Cloud::fromIO, fileName, transformation);
// 		cloud->setBoundaries(boundaries);
// 	}

// 	float angleThreshold = 10.0f;
// 	pcl::console::parse_argument (argc, argv, "--angle", angleThreshold);
// 	angleThreshold = angleThreshold / 180 * M_PI;

// 	float distanceThreshold = 0.01f;
// 	pcl::console::parse_argument (argc, argv, "--distance", distanceThreshold);

// 	QList<Cloud*> cloudList = cloudManager->getAllClouds();
// 	for (int i = 0; i < cloudList.size(); ++i)
// 	{
// 		Cloud *cloud = cloudList[i];	
// 		Eigen::Matrix4f transformation = cloud->getTransformation();
// 		Eigen::Matrix4f randomRigidTransf = randomRigidTransformation(angleThreshold, distanceThreshold);
// 		cloud->setTransformation(randomRigidTransf * transformation);
// 		std::cout << randomRigidTransf << std::endl;
// 	}

// 	for (int i = 0; i < cloudList.size(); ++i)
// 	{
// 		Cloud *cloud = cloudList[i];		
// 		QString cloudName = cloud->getCloudName();
// 		CloudDataPtr cloudData = cloud->getCloudData();
// 		Eigen::Matrix4f transformation = cloud->getTransformation();
// 		BoundariesPtr boundaries = cloud->getBoundaries();
// 		QString fileName =  cloud->getFileName();
// 		QFileInfo fileInfo(fileName);
// 		QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_out.ply";
// 		CloudIO::exportCloudData(newFileName, cloudData);
// 		CloudIO::exportTransformation(newFileName, transformation);
// 		CloudIO::exportBoundaries(newFileName, boundaries);

// 		CloudIO::exportCloudData(newFileName, cloudData);
// 	}

// 	return 0;
// }
