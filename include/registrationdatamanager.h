#ifndef REGISTRATIONDATAMANAGER_H
#define REGISTRATIONDATAMANAGER_H

#include <QtCore/QObject>
#include "cloud.h"
#include "pclbase.h"

namespace registar
{
	class RegistrationData : public QObject
	{
		Q_OBJECT

	public:
		RegistrationData(Cloud *cloud, const QString &dataName, QObject *parent = 0);
		virtual ~RegistrationData();

		Cloud * cloud;
		CloudDataPtr cloudData;	
		KdTreePtr kdTree;
		BoundariesConstPtr boundaries;
	};

	class RegistrationDataManager : public QObject
	{
		Q_OBJECT

	public:
		RegistrationDataManager(QObject *parent = 0);
		virtual ~RegistrationDataManager();
		RegistrationData* addRegistrationData(Cloud *cloud, const QString &dataName);
		void removeRegistrationData(const QString &dataName);
		RegistrationData* getRegistrationData(const QString &dataName);

		QList<RegistrationData*> getAllRegistrationDatas();
		QStringList getAllRegistrationDataNames();
	};
}

#endif