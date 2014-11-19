#include <QtCore/QtCore>
#include <pcl/io/ply_io.h>

#include "../include/cloudmanager.h"

using namespace registar;

CloudManager::CloudManager(QObject *parent) : QObject(parent) {}

CloudManager::~CloudManager(){}

QString CloudManager::generateCloudName()
{
	static int cloudID = 0;
	//return QString("Cloud_%1").arg(QString::number(cloudID++));
	return QString("%1").arg(QString::number(cloudID++));
}

Cloud* CloudManager::addCloud(CloudDataPtr cloudData, const Polygons &polygons, Cloud::FromWhere fromWhere,
	const QString &fileName, const Eigen::Matrix4f &transformation)
{
	QString cloudName = generateCloudName();
	return new Cloud(cloudData, polygons, fromWhere, fileName, transformation, cloudName, this);
}

void CloudManager::removeCloud(const QString &cloudName)
{
	Cloud *cloud = findChild<Cloud*>(cloudName);
	if(cloud) delete cloud;
}

Cloud* CloudManager::getCloud(const QString &cloudName)
{
	return findChild<Cloud*>(cloudName);
}

QList<Cloud*> CloudManager::getAllClouds()
{
	return findChildren<Cloud*>();
}

QStringList CloudManager::getAllCloudNames()
{
	QStringList allCloudNames;
	QList<Cloud*> allClouds = getAllClouds();
	for (int i = 0; i < allClouds.size(); ++i)
	{
		allCloudNames << QString(allClouds[i]->getCloudName());
	}
	return allCloudNames;
}





