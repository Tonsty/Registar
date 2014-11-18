#include <QtCore/QDebug>
//#include "../include/qtbase.h"

#define PCL_NO_PRECOMPILE
#include <pcl/features/boundary.h>
#include <pcl/filters/extract_indices.h>

#include "../include/boundaryestimation.h" 

using namespace registar;

BoundaryEstimation::BoundaryEstimation(){}

BoundaryEstimation::~BoundaryEstimation(){}

void BoundaryEstimation::filter(CloudDataPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers)
{
	float searchRadius = parameters["searchRadius"].toFloat(); 
	float angleThreshold = parameters["angleThreshold"].toFloat();
	float dilationRadius = parameters["dilationRadius"].toFloat();

	qDebug() << "SearchRadius : " << QString::number(searchRadius);
	qDebug() << "AngleThreshold : " << QString::number(angleThreshold);
	qDebug() << "DilationRadius : " << QString::number(dilationRadius);

	qDebug() << "Cloud Size Before : " << cloudData->size();

	pcl::BoundaryEstimation<PointType, PointType, pcl::Boundary> be;
	be.setInputCloud(cloudData);
	be.setInputNormals(cloudData);
	be.setRadiusSearch(searchRadius);
	pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
	be.setSearchMethod(tree);
	be.setAngleThreshold(angleThreshold/180.0f*M_PI);
	boundaries.reset(new Boundaries);
	be.compute(*boundaries);

	for(int i = 0; i < (*boundaries).size(); ++i)
	{
		if((*boundaries).points[i].boundary_point == 1)
		{
			PointType point = (*cloudData)[i];
			std::vector<int> indices;
			std::vector<float> sqr_distances;
			tree->radiusSearch(point, dilationRadius, indices, sqr_distances, cloudData->size());
			for (int j = 0; j < indices.size(); ++j)
			{	
				(*boundaries).points[indices[j]].boundary_point = 2;
			}
			//std::cerr << indices.size() << " " << std::endl;		
		}
	}

	pcl::PointIndicesPtr inliers_indices(new pcl::PointIndices);
	for( int i = 0; i < (*boundaries).size(); i++)
	{
		if ((*boundaries).points[i].boundary_point != 0)
		{
			inliers_indices->indices.push_back(i);
		}
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
}

void BoundaryEstimation::filter(CloudDataPtr cloudData, QVariantMap parameters, BoundariesPtr &boundaries, CloudDataPtr &cloudData_filtered)
{
	CloudDataPtr cloudData_inliers, cloudData_outliers;
	filter(cloudData, parameters, boundaries, cloudData_inliers, cloudData_outliers);
	cloudData_filtered = cloudData_inliers;
}
