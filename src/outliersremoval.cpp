#include <QtCore/QDebug>
//#include "../include/qtbase.h"

#define PCL_NO_PRECOMPILE
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "../include/outliersremoval.h" 

using namespace registar;

OutliersRemoval::OutliersRemoval(){}

OutliersRemoval::~OutliersRemoval(){}

void OutliersRemoval::filter(CloudDataPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_inliers, CloudDataPtr &cloudData_outliers)
{
	int method = parameters["method"].toInt();
	float searchRadius = parameters["searchRadius"].toFloat(); 
	int nearestK = parameters["nearestK"].toInt();
	float deviation = parameters["deviation"].toFloat();

	qDebug() << "Method : " << QString::number(method);
	qDebug() << "SearchRadius : " << QString::number(searchRadius);
	qDebug() << "NearestK : " << QString::number(nearestK);
	qDebug() << "Deviation : " << QString::number(deviation);

	qDebug() << "Cloud Size Before : " << cloudData->size();

	CloudDataPtr cloudInliers(new CloudData), cloudOutliers(new CloudData);
	switch(method)
	{
		case 0:
		{
			pcl::RadiusOutlierRemoval<PointType> ror;
			ror.setInputCloud(cloudData);
			ror.setRadiusSearch(searchRadius);
			ror.setMinNeighborsInRadius(nearestK);
			ror.setNegative(false);
			ror.filter(*cloudInliers);
			ror.setNegative(true);
			ror.filter(*cloudOutliers);
			break;
		}
		case 1:
		{
			pcl::StatisticalOutlierRemoval<PointType> sor;
			sor.setInputCloud(cloudData);
			sor.setMeanK(nearestK);
			sor.setStddevMulThresh(deviation);
			sor.setNegative(false);
			sor.filter(*cloudInliers);
			sor.setNegative(true);
			sor.filter(*cloudOutliers);
			break;
		}
	}

	cloudData_inliers = cloudInliers;
	cloudData_outliers = cloudOutliers;

	qDebug() << "Cloud Inliers Size : " << cloudData_inliers->size();
	qDebug() << "Cloud Outliers Size : " << cloudData_outliers->size();
}

void OutliersRemoval::filter(CloudDataPtr cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered)
{
	CloudDataPtr cloudData_inliers, cloudData_outliers;
	filter(cloudData, parameters, cloudData_inliers, cloudData_outliers);
	cloudData_filtered = cloudData_inliers;
}
