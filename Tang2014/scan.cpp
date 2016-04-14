#include "scan.h"

#include <iostream>
#include <fstream>

#ifdef WIN32
#define PCL_NO_PRECOMPILE
#endif

#include <boost/filesystem.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/features/boundary.h>
#include <pcl/common/transforms.h>

#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>

namespace Tang2014
{
	void importScanPtrs(const std::string fileName, ScanPtrs &scanPtrs )
	{
		std::fstream file;
		file.open(fileName.c_str());

		char dummy[300];
		int scan_i = 0;
		std::string directory = boost::filesystem::path(fileName).remove_filename().string();
		if (directory == "") directory = ".";
		while( file.getline(dummy, 300) )
		{
			std::cout << "scan " << scan_i << " : " << dummy << std::endl;

			ScanPtr scanPtr(new Scan);
			scanPtrs.push_back(scanPtr);

			//read in file path
			scanPtr->filePath = directory + "/" + std::string(dummy);

			//read in transformation
			QFileInfo fileInfo(dummy);
			//QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";	
			QString tfFileName = QString::fromStdString(directory) + "/" + fileInfo.completeBaseName() + ".tf";	
			std::cout << "read in " << tfFileName.toStdString() << std::endl; 
			QFile tfFile(tfFileName);

			Transformation transformation = Transformation::Identity();
			if (tfFile.open(QIODevice::ReadOnly))
			{
				QTextStream in(&tfFile);
				for (int i = 0; i < 4; ++i)
					for (int j = 0; j < 4; ++j)
						in >> transformation(i, j);
				tfFile.close();
			}
			std::cout << transformation << std::endl;
			scanPtr->transformation = transformation;				

			//read in pointcloud (points and normals)
			std::cout << "read in " << directory <<"/" << dummy << std::endl;
			pcl::PLYReader plyReader;
			scanPtr->pointsPtr.reset(new Points);
			//plyReader.read(dummy, *scanPtr->pointsPtr);
			pcl::PolygonMeshPtr polygonMesh(new pcl::PolygonMesh);
			plyReader.read(directory + "/" + dummy, *polygonMesh);
			pcl::fromPCLPointCloud2(polygonMesh->cloud, *scanPtr->pointsPtr);

			//remove NAN points
			scanPtr->pointsPtr->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
			scanPtr->pointsPtr->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);
			std::vector<int> nanIndicesVector;
			pcl::removeNaNFromPointCloud( *scanPtr->pointsPtr, *scanPtr->pointsPtr, nanIndicesVector );
			std::cout << *scanPtr->pointsPtr << std::endl;

			//transform points and normals
			pcl::transformPointCloudWithNormals(*scanPtr->pointsPtr, *scanPtr->pointsPtr, transformation);

			//read in boundaries
			//QString bdFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".bd";
			QString bdFileName = QString::fromStdString(directory) + "/" + fileInfo.completeBaseName() + ".bd";
			std::cout << "read in " << bdFileName.toStdString() << std::endl;
			pcl::PCDReader pcdReader;
			scanPtr->boundariesPtr.reset(new Boundaries);
			pcdReader.read(bdFileName.toStdString(), *scanPtr->boundariesPtr);
			std::cout << *scanPtr->boundariesPtr << std::endl;

			scan_i++;
		}
	}
}
