#ifndef GLOBALREGISTRATION_H
#define GLOBALREGISTRATION_H

#include <QtCore/QObject>

#include "pairwiseregistration.h"

class CycleRegistration : public QObject
{
	Q_OBJECT
	
public:
	CycleRegistration(QList<PairwiseRegistration*> prList, QObject *parent = 0);
	~CycleRegistration();
	
	QList<PairwiseRegistration*> prList;

};

class CycleRegistrationManager : public QObject
{
	Q_OBJECT

public:
	CycleRegistrationManager(QObject *parent = 0);
	~CycleRegistrationManager();

	CycleRegistration* addCycleRegistration(QList<PairwiseRegistration*> prList);
	CycleRegistration* getCycleRegistration(QString cloudNameCycle);
};

class GlobalRegistration : public QObject
{
	Q_OBJECT

public:
	GlobalRegistration(QList<PairwiseRegistration*> prList, QList<CycleRegistration*> crList, QObject *parent = 0);
	~GlobalRegistration();

	QList<PairwiseRegistration*> prList;
	QList<CycleRegistration*> crList;
};

#endif 