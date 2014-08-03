#ifndef REGISTRATIONDATAMANAGER_H
#define REGISTRATIONDATAMANAGER_H

#include <QtCore/QObject>
#include "cloud.h"
#include "pclbase.h"

class RegistrationData : public QObject
{
	Q_OBJECT

public:
	RegistrationData(Cloud *cloud, QString dataName, QObject *parent = 0);
	virtual ~RegistrationData();

	Cloud * cloud;
	CloudDataPtr cloudData;	
	KdTreePtr kdTree;
	BoundariesPtr boundaries;
};

class RegistrationDataManager : public QObject
{
	Q_OBJECT

public:
	RegistrationDataManager(QObject *parent = 0);
	virtual ~RegistrationDataManager();
	RegistrationData* addRegistrationData(Cloud *cloud, QString dataName);
	void removeRegistrationData(QString dataName);
	RegistrationData* getRegistrationData(QString dataName);

};

#endif