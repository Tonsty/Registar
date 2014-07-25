#include <pcl/common/transforms.h>

#include "../include/registrationdatamanager.h"

RegistrationData::RegistrationData(Cloud *cloud, QString dataName, QObject *parent) : QObject(parent)
{
	this->cloud = cloud;

	cloudData.reset(new CloudData);
	pcl::transformPointCloudWithNormals(*cloud->getCloudData(), *cloudData, cloud->getTransformation());

	kdTree.reset(new KdTree);
	kdTree->setInputCloud(cloudData);

	boundaries = cloud->getBoundaries();

	this->setObjectName(dataName);
}

RegistrationData::~RegistrationData() {}

RegistrationDataManager::RegistrationDataManager(QObject *parent) : QObject(parent) {}

RegistrationDataManager::~RegistrationDataManager(){}

RegistrationData* RegistrationDataManager::addRegistrationData(Cloud *cloud, QString dataName)
{
	return new RegistrationData(cloud, dataName, this);
}

void RegistrationDataManager::removeRegistrationData(QString dataName)
{
	RegistrationData *registrationData = findChild<RegistrationData*>(dataName);
	if(registrationData) delete registrationData;
}

RegistrationData* RegistrationDataManager::getRegistrationData(QString dataName)
{
	return findChild<RegistrationData*>(dataName);
}