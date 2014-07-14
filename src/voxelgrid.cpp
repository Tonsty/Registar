#include <QtCore/QDebug>
#include <pcl/filters/voxel_grid.h>

#include "../include/voxelgrid.h"

VoxelGrid::VoxelGrid(){}

VoxelGrid::~VoxelGrid(){}

void VoxelGrid::filter(CloudDataPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered)
{
	float leafSizeX = parameters["leafSizeX"].toFloat();
	float leafSizeY = parameters["leafSizeY"].toFloat();
	float leafSizeZ = parameters["leafSizeZ"].toFloat();

	qDebug() << "LeafSizeX : " << QString::number(leafSizeX);
	qDebug() << "LeafSizeY : " << QString::number(leafSizeY);
	qDebug() << "LeafSizeZ : " << QString::number(leafSizeZ);
	
	qDebug() << "Cloud Size Before : " << cloudData->size();

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(cloudData);
	sor.setLeafSize(leafSizeX, leafSizeY, leafSizeZ);
	cloudData_filtered.reset(new CloudData);
	sor.filter(*cloudData_filtered);

	qDebug() << "Cloud Size After : " << cloudData_filtered->size();
}