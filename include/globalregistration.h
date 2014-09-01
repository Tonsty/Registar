#ifndef GLOBALREGISTRATION_H
#define GLOBALREGISTRATION_H

#include <QtGui/QWidget>

#ifndef Q_MOC_RUN
#include "pairwiseregistration.h"
#endif

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

// class GlobalRegistration : public QObject
// {
// 	Q_OBJECT

// public:
// 	GlobalRegistration(QObject *parent = 0);
// 	~GlobalRegistration();
// };



#endif 