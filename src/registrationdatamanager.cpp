#include <QtCore/QStringList>
#include <pcl/common/transforms.h>

#include "../include/registrationdatamanager.h"

using namespace registar;

RegistrationData::RegistrationData(Cloud *cloud, const QString &dataName, QObject *parent) : QObject(parent)
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

RegistrationData* RegistrationDataManager::addRegistrationData(Cloud *cloud, const QString& dataName)
{
	return new RegistrationData(cloud, dataName, this);
}

void RegistrationDataManager::removeRegistrationData(const QString& dataName)
{
	RegistrationData *registrationData = findChild<RegistrationData*>(dataName);
	if(registrationData) delete registrationData;
}

RegistrationData* RegistrationDataManager::getRegistrationData(const QString& dataName)
{
	return findChild<RegistrationData*>(dataName);
}

QList<RegistrationData*> RegistrationDataManager::getAllRegistrationDatas()
{
	return findChildren<RegistrationData*>();
}

QStringList RegistrationDataManager::getAllRegistrationDataNames()
{
	QStringList allRegistrationDataNames;
	QList<RegistrationData*> allRegistrationDatas = getAllRegistrationDatas();
	for (int i = 0; i < allRegistrationDatas.size(); ++i)
	{
		allRegistrationDataNames << allRegistrationDatas[i]->objectName();
	}
	return allRegistrationDataNames;	
}