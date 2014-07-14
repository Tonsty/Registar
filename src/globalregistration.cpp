#include <QtCore/QDebug>

#include "../include/globalregistration.h"

CycleRegistration::CycleRegistration(QList<PairwiseRegistration*> prList, QObject *parent) : QObject(parent)
{
	this->prList = prList;
}

CycleRegistration::~CycleRegistration()
{

}

CycleRegistrationManager::CycleRegistrationManager(QObject *parent) : QObject(parent){}

CycleRegistrationManager::~CycleRegistrationManager(){}


CycleRegistration* CycleRegistrationManager::addCycleRegistration(QList<PairwiseRegistration*> prList)
{
	CycleRegistration* cycleRegistration = new CycleRegistration(prList, this);
	QString cloudNameCycle = "";
	for (int i = 0; i < prList.size()-1; ++i)
	{
		cloudNameCycle.append(prList[i]->cloud_target->getCloudName()).append(",");
	}
	cloudNameCycle.append(prList.back()->cloud_target->getCloudName());
	qDebug() << cloudNameCycle;
	cycleRegistration->setObjectName(cloudNameCycle);
	return cycleRegistration;
}

CycleRegistration* CycleRegistrationManager::getCycleRegistration(QString cloudNameCycle)
{
	return findChild<CycleRegistration*>(cloudNameCycle);
}



