#include <QtCore/QDebug>

#define PCL_NO_PRECOMPILE

#include "../include/normalfield.h"
#include "../include/utilities.h"

using namespace registar;

NormalField::NormalField(){}

NormalField::~NormalField(){}

void NormalField::filter(CloudDataConstPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered)
{
	float X = parameters["X"].toFloat();
	float Y = parameters["Y"].toFloat();
	float Z = parameters["Z"].toFloat();
	int method = parameters["method"].toInt();

	qDebug() << "Viewpoint X : " << X;
	qDebug() << "Viewpoint Y : " << Y;
	qDebug() << "Viewpoint Z : " << Z;

	qDebug() << "Cloud Size Before : " << cloudData->size();

	cloudData_filtered.reset(new CloudData);
	pcl::PointXYZ view_point;
	view_point.x = X;
	view_point.y = Y;
	view_point.z = Z;

	if (method == 0)
	{
		flipPointCloudNormalsTowardsViewpoint(*cloudData, view_point, *cloudData_filtered);
	} else if (method == 3){
		setPointCloudNormalsTowardsViewpoint(*cloudData, view_point, *cloudData_filtered);
	}

	qDebug() << "Cloud Size After : " << cloudData_filtered->size();
}