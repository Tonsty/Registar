#ifndef GLOBALREGISTRATION_H
#define GLOBALREGISTRATION_H

#include <QtCore/QObject>

#ifndef Q_MOC_RUN
#include "pairwiseregistration.h"
#endif 

enum CycleRegistrationRefineMethod
{
	UNIFORM_REFINE,
	NON_UNIFORM_REFINE,
	MINIMIZE_REFINE
};

class CycleRegistration : public QObject
{
	Q_OBJECT
	
public:
	CycleRegistration(QList<PairwiseRegistration*> prList, QObject *parent = 0);
	~CycleRegistration();
	
	QList<PairwiseRegistration*> prList;
	void refine(CycleRegistrationRefineMethod method = UNIFORM_REFINE);

	void uniform_refine();
	void non_uniform_refine();
	void minimize_refine();
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