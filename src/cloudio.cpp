#include <QtCore/QDebug>
#include <QtCore/QFileInfo>

#define PCL_NO_PRECOMPILE
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/png_io.h>
#include <pcl/filters/filter.h>

#include "../include/cloudio.h"

using namespace registar;

CloudIO::CloudIO(){}

CloudIO::~CloudIO(){}

bool CloudIO::importCloudData(const QString &fileName, CloudDataPtr cloudData)
{
	pcl::PLYReader reader;
	if (reader.read(fileName.toStdString(), *cloudData) == 0)
	{
		//qDebug() << "sensor_origin (x,y,z,w):" << cloudData->sensor_origin_.x() << ", " << cloudData->sensor_origin_.y()
		//		<< ", " << cloudData->sensor_origin_.z() << ", " << cloudData->sensor_origin_.w();
		//qDebug() << "sensor_orientation (w,x,y,z): " << cloudData->sensor_orientation_.w() << ", " << cloudData->sensor_orientation_.x() 
		//		<< ", " << cloudData->sensor_orientation_.y() << ", " << cloudData->sensor_orientation_.z();
		cloudData->sensor_origin_ = Eigen::Vector4f(0, 0, 0, 0);
		cloudData->sensor_orientation_ = Eigen::Quaternionf(1, 0, 0, 0);

		// QFileInfo fileInfo(fileName);
		// QString pngFileName = fileInfo.path() + "/" + fileInfo.baseName() + ".png";
		// pcl::io::savePNGFile( pngFileName.toStdString(), *cloudData);

		std::vector<int> nanIndicesVector;
		pcl::removeNaNFromPointCloud( *cloudData, *cloudData, nanIndicesVector );
	}
	else return false;

	return true;
}

bool CloudIO::importPolygonMesh(const QString &fileName, PolygonMeshPtr polygonMesh)
{
	pcl::PLYReader reader;
	if (reader.read(fileName.toStdString(), *polygonMesh) == 0)
	{
		std::cout << "height: " << polygonMesh->cloud.height << " , " << "width: " <<  polygonMesh->cloud.width << std::endl;
		std::cout << "polygons: " << polygonMesh->polygons.size() << std::endl;
	}
	else return false;

	return true;
}

bool CloudIO::exportPolygonMesh(const QString &fileName, PolygonMeshConstPtr polygonMesh)
{
	return true;
}

bool CloudIO::importTransformation(const QString &fileName, Eigen::Matrix4f &transformation)
{
	QFileInfo fileInfo(fileName);
	QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";

	qDebug() << tfFileName;

	QFile file(tfFileName);
	if (!file.open(QIODevice::ReadOnly))
	{
		qDebug() << "Cannot import transformation!";
		transformation = Eigen::Matrix4f::Identity();
		return false;
	}

	QTextStream in(&file);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			in >> transformation(i, j);
		}
	}
	
	return true;
}

bool CloudIO::importBoundaries(const QString &fileName, BoundariesPtr boundaries)
{
	QFileInfo fileInfo(fileName);
	QString bdFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".bd";

	qDebug() << bdFileName;

	QFile file(bdFileName);
	if (!file.open(QIODevice::ReadOnly))
	{
		qDebug() << "Cannot import boundaries!";
		return false;
	}

	pcl::io::loadPCDFile(bdFileName.toStdString(), *boundaries);

	return true;
}

bool CloudIO::exportCloudData(const QString &fileName, CloudDataConstPtr cloudData)
{
	std::vector<int> nanIndicesVector;
	CloudDataPtr temp(new CloudData);
	pcl::removeNaNFromPointCloud( *cloudData, *temp, nanIndicesVector );

	pcl::PLYWriter writer;
	writer.write(fileName.toStdString(), *temp);

	return true;
}

bool CloudIO::exportTransformation(const QString &fileName, const Eigen::Matrix4f &transformation)
{
	QFileInfo fileInfo(fileName);
	QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";

	qDebug() << tfFileName;

	QFile file(tfFileName);
	if (!file.open(QIODevice::WriteOnly))
	{
		qDebug() << "Cannot export transformation!";
		return false;
	}

	QTextStream out(&file);
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			out << transformation(i, j) << "\t";
		}
		out << endl;
	}

	return true;
}

bool CloudIO::exportBoundaries(const QString &fileName, BoundariesConstPtr boundaries)
{
	QFileInfo fileInfo(fileName);
	QString bdFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".bd";

	qDebug() << bdFileName;

	if(boundaries != NULL && boundaries->size() > 0) pcl::io::savePCDFile(bdFileName.toStdString(), *boundaries);

	return true;
}

