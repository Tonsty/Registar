#include <QtCore/QDebug>
#include <QtCore/QFileInfo>
#include <QtCore/QFile>
#include <QtCore/QStringList>

#include <Eigen/Dense>

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../include/cloudio.h"

int main(int argc, char **argv)
{	
  	std::vector<int> p_file_indices_conf = pcl::console::parse_file_extension_argument (argc, argv, ".conf");
    std::string output_directory = ".";
    pcl::console::parse_argument (argc, argv, "--directory", output_directory); 

  	for (int i = 0; i < p_file_indices_conf.size(); ++i)
  	{
  		QFile conf_file(argv[p_file_indices_conf[i]]);
  		if(conf_file.open(QIODevice::ReadOnly)) 
  		{
  			QTextStream in(&conf_file);
  			while(!in.atEnd()) {
  			    QString line = in.readLine();
  			    //qDebug() << line;
  			    QStringList list = line.split(" ");
  			    if (list[0] == "camera" || list[0] == "mesh") continue;
  			    if (list[0] == "bmesh")
  			    {
  			    	QString ply_file = list[1];
  			    	float tx = list[2].toFloat();
  			    	float ty = list[3].toFloat();
  			    	float tz = list[4].toFloat();
  			    	float qi = list[5].toFloat();
  			    	float qj = list[6].toFloat();
  			    	float qk = list[7].toFloat();
  			    	float ql = list[8].toFloat();
  			    	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  			    	Eigen::Vector3f translation = Eigen::Vector3f(tx, ty, tz);
  			    	Eigen::Quaternionf quaternion = Eigen::Quaternionf(qi, qj, qk, ql);
  			    	transformation.block<3,3>(0,0) = quaternion.toRotationMatrix();
  			    	transformation.block<3,1>(0,3) = translation;

              QFileInfo fileInfo(ply_file);
  			    	CloudIO::exportTransformation(QString(output_directory.c_str()) + "/" + fileInfo.completeBaseName() + ".tf", transformation);
  			    }
  			}
  		}
  	}

    // pcl::PLYReader reader;
    // pcl::PolygonMesh mesh;
    // reader.read("dragonStandRight_0_out.ply", mesh);

    // std::cout << mesh.cloud.height << std::endl;
    // std::cout << mesh.cloud.width << std::endl;

    // QFileInfo info("dragonStandRight_0_out.ply");
    // pcl::io::savePLYFile((info.path() + "/" + info.completeBaseName() + "_transformed.ply").toStdString(), mesh);

  	// pcl::PLYReader reader;
  	// pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;

  	// reader.read("bun000.ply", cloud);

  	// std::cout << cloud << std::endl;

  	// // pcl::PLYWriter writer;
  	// // writer.write("bun000_out_out.ply", cloud);

  	// pcl::PCDWriter writer;
  	// writer.write("bun000.pcd", cloud);


  	// pcl::PLYReader reader;
  	// pcl::PolygonMesh mesh;
  	// reader.read("bun000_meshlab.ply", mesh);

  	// std::cout << mesh.cloud.height << std::endl;
  	// std::cout << mesh.cloud.width << std::endl;

  	// pcl::visualization::PCLVisualizer visualizer;
  	// visualizer.addPolygonMesh(mesh);

  	// visualizer.spin();

  	// std::cout << mesh.polygons.size() << std::endl;
  	// for (int i = 0; i < 10; ++i)
  	// {
  	// 	std::cout << mesh.polygons[i].vertices.size() << std::endl;

  	// 	for (int j = 0; j < mesh.polygons[i].vertices.size(); ++j)
  	// 	{
  	// 		std::cout << mesh.polygons[i].vertices[j] << " ";
  	// 	}
  	// 	std::cout << std::endl;
  	// }


	// for (int i = 0; i < p_file_indices_conf.size(); ++i)
	// {
	// 	CloudDataPtr cloudData(new CloudData);
	// 	Eigen::Matrix4f transformation;
	// 	BoundariesPtr boundaries(new Boundaries);

	// 	QString fileName = QString(argv[p_file_indices_ply[i]]);
	// 	CloudIO::importCloudData(fileName, cloudData);
	// 	CloudIO::importTransformation(fileName, transformation);
	// 	CloudIO::importBoundaries(fileName, boundaries);
	// 	Cloud* cloud = cloudManager->addCloud(cloudData, Cloud::fromIO, fileName, transformation);
	// 	cloud->setBoundaries(boundaries);
	// }

	// float angleThreshold = 10.0f;
	// pcl::console::parse_argument (argc, argv, "--angle", angleThreshold);
	// angleThreshold = angleThreshold / 180 * M_PI;

	// float distanceThreshold = 0.01f;
	// pcl::console::parse_argument (argc, argv, "--distance", distanceThreshold);

	// QList<Cloud*> cloudList = cloudManager->getAllClouds();
	// for (int i = 0; i < cloudList.size(); ++i)
	// {
	// 	Cloud *cloud = cloudList[i];	
	// 	Eigen::Matrix4f transformation = cloud->getTransformation();
	// 	Eigen::Matrix4f randomRigidTransf = randomRigidTransformation(angleThreshold, distanceThreshold);
	// 	cloud->setTransformation(randomRigidTransf * transformation);
	// 	std::cout << randomRigidTransf << std::endl;
	// }

	// for (int i = 0; i < cloudList.size(); ++i)
	// {
	// 	Cloud *cloud = cloudList[i];		
	// 	QString cloudName = cloud->getCloudName();
	// 	CloudDataPtr cloudData = cloud->getCloudData();
	// 	Eigen::Matrix4f transformation = cloud->getTransformation();
	// 	BoundariesPtr boundaries = cloud->getBoundaries();
	// 	QString fileName =  cloud->getFileName();
	// 	QFileInfo fileInfo(fileName);
	// 	QString newFileName = fileInfo.path() + "/" + fileInfo.baseName() + "_out.ply";
	// 	CloudIO::exportCloudData(newFileName, cloudData);
	// 	CloudIO::exportTransformation(newFileName, transformation);
	// 	CloudIO::exportBoundaries(newFileName, boundaries);

	// 	CloudIO::exportCloudData(newFileName, cloudData);
	// }

	return 0;
}