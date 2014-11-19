#include <QtCore/QDebug>
//#include "../include/qtbase.h"

#define PCL_NO_PRECOMPILE
#include <pcl/common/io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "../pcl_bugfix/gpu_extract_clusters2.hpp"
#include "../include/euclideanclusterextraction.h" 

using namespace registar;

EuclideanClusterExtraction::EuclideanClusterExtraction(){}

EuclideanClusterExtraction::~EuclideanClusterExtraction(){}

// void EuclideanClusterExtraction::filter(CloudDataPtr cloudData, QVariantMap parameters, std::vector<CloudDataPtr> cloudData_filtereds)
void EuclideanClusterExtraction::filter(CloudDataConstPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers)
{
	// cloudData = parameters["cloudData"].value<CloudDataPtr>();
	// cloudData_filtered = parameters["cloudData_filtered"].value<CloudDataPtr>();

	float clusterTolerance = parameters["clusterTolerance"].toFloat(); 
	int minClusterSize = parameters["minClusterSize"].toInt();
	int maxClusterSize = parameters["maxClusterSize"].toInt();
	bool use_cpu = parameters["use_cpu"].toBool();
	bool use_gpu = parameters["use_gpu"].toBool();

	qDebug() << "ClusterTolerance : " << QString::number(clusterTolerance);
	qDebug() << "MinClusterSize : " << QString::number(minClusterSize);
	qDebug() << "MaxClusterSize : " << QString::number(maxClusterSize);

	qDebug() << "use_cpu : " << use_cpu;
	qDebug() << "use_gpu : " << use_gpu;

	std::vector<pcl::PointIndices> cluster_indices;

	if(use_cpu)
	{
		pcl::EuclideanClusterExtraction<PointType> ec;
		ec.setClusterTolerance(clusterTolerance);
		ec.setMinClusterSize(minClusterSize);
		ec.setMaxClusterSize(maxClusterSize);
		pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloudData);

		//std::vector<pcl::PointIndices> cluster_indices;
		ec.extract (cluster_indices);
	}
	else if (use_gpu)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDataXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloudData, *cloudDataXYZ);
		pcl::gpu::EuclideanClusterExtraction2 gpu_ec;
		gpu_ec.setClusterTolerance(clusterTolerance);
		gpu_ec.setMinClusterSize(minClusterSize);
		gpu_ec.setMaxClusterSize(maxClusterSize);
		pcl::gpu::Octree::Ptr tree(new pcl::gpu::Octree);
		pcl::gpu::DeviceArray<pcl::PointXYZ> cloudDataDevice;
		cloudDataDevice.upload(cloudDataXYZ->points);
		tree->setCloud(cloudDataDevice);
		tree->build();
		gpu_ec.setSearchMethod(tree);
		gpu_ec.setHostCloud(cloudDataXYZ);

		//std::vector<pcl::PointIndices> cluster_indices;
		gpu_ec.extract (cluster_indices);
	}

	qDebug() << "ClusterIndicesSize : " << cluster_indices.size();

	pcl::PointIndicesPtr inliers_indices(new pcl::PointIndices);
	for( int i = 0; i < cluster_indices.size(); i++)
	{
		qDebug() << "Cluster " <<QString::number(i) << " : " << cluster_indices[i].indices.size();
		inliers_indices->indices.insert( inliers_indices->indices.end(), cluster_indices[i].indices.begin(), cluster_indices[i].indices.end());
	}

	pcl::ExtractIndices<PointType> indicesExtract;
	indicesExtract.setInputCloud(cloudData);
	indicesExtract.setIndices(inliers_indices);

	CloudDataPtr cloudInliers(new CloudData);
	indicesExtract.setNegative(false);
	indicesExtract.filter(*cloudInliers);

	CloudDataPtr cloudOutliers(new CloudData);
	indicesExtract.setNegative(true);
	indicesExtract.filter(*cloudOutliers);

	cloudData_inliers = cloudInliers;
	cloudData_outliers = cloudOutliers;

	qDebug() << "Cloud Inliers Size : " << cloudData_inliers->size();
	qDebug() << "Cloud Outliers Size : " << cloudData_outliers->size();

	// for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end(); it++)
	// {
	// 	cloudData_filtereds.push_back(CloudDataPtr(new CloudData));
	// 	CloudDataPtr temp = cloudData_filtereds.back();
	// 	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	// 	{
	// 		temp->push_back(cloudData->points[*pit]);
	// 	}
	// }
}

void EuclideanClusterExtraction::filter(CloudDataConstPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered)
{
	CloudDataPtr cloudData_inliers, cloudData_outliers;
	filter(cloudData, parameters, cloudData_inliers, cloudData_outliers);
	cloudData_filtered = cloudData_inliers;
	// cloudData_filtered.reset(new CloudData);
	// std::vector<CloudDataPtr> cloudData_filtereds;
	// filter(cloudData, parameters, cloudData_filtereds);
	// for (int i = 0; i < cloudData_filtereds.size(); ++i)
	// {
	// 	cloudData_filtered->insert(cloudData_filtered->end(), cloudData_filtereds[i]->begin(), cloudData_filtereds[i]->end());
	// }
	// qDebug() << "Cloud Size After : " << cloudData_filtered->size();
}
