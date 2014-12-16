#include <QtCore/QDebug>

#include "../include/utilities.h"
#include "../include/globalregistration.h"

using namespace registar;

CycleRegistration::CycleRegistration(QList<PairwiseRegistration*> prList, QObject *parent) : QObject(parent)
{
	this->prList = prList;
}

CycleRegistration::~CycleRegistration()
{

}

void CycleRegistration::refine(CycleRegistrationRefineMethod method)
{
	switch(method)
	{
		case UNIFORM_REFINE:
		{
			uniform_refine();
			break;
		}
		case NON_UNIFORM_REFINE:
		{
			non_uniform_refine();
			break;
		}
		case MINIMIZE_REFINE:
		{
			minimize_refine();
			break;
		}
	}
}

void CycleRegistration::uniform_refine()
{
	std::vector< Eigen::AngleAxisf > R(prList.size());
	std::vector< Eigen::AngleAxisf > E(prList.size());

	for (int i = 0; i < prList.size(); ++i) R[i] = Eigen::AngleAxisf(toRigidTransformation(prList[i]->getTransformation()).block<3, 3>(0, 0)); // normalize to rigid
	for (int i = 0; i < E.size(); ++i)
	{
		E[i] = Eigen::AngleAxisf(Eigen::Matrix3f::Identity());
		for (int j = 0; j < R.size(); ++j) E[i] = E[i] * R[(j+1) % R.size()];
		E[i] = Eigen::AngleAxisf( -E[i].angle(), E[i].axis());
	}

	std::vector< Eigen::AngleAxisf > newR(prList.size());
	for (int i = 0; i < E.size(); ++i)
	{
		Eigen::AngleAxisf newE = Eigen::AngleAxisf(E[i].angle()/E.size() , E[i].axis());
		newR[i] = R[i] * newE; 
	}

	Eigen::AngleAxisf newR_total = Eigen::AngleAxisf(Eigen::Matrix3f::Identity());
	for (int i = 0; i < newR.size(); ++i)
	{
		newR_total = newR_total * newR[i];
		//std::cout << newR[i].angle() << std::endl;
	}
	std::cout << "newR_total: " << newR_total.angle() << std::endl;

	std::vector< Eigen::AngleAxisf > M(prList.size());
	M[0] = Eigen::AngleAxisf(Eigen::Matrix3f::Identity());
	for (int i = 1; i < newR.size(); ++i)
	{
		M[i] = M[i-1] * newR[i-1];
	}

	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(3*M.size()+3, 3*M.size()+3);
	for (int i = 0; i < M.size(); ++i) A.block<3, 3>(i*3, i*3) = Eigen::Matrix3f::Identity() * 2.0f;
	for (int i = 0; i < M.size(); ++i)
	{
		A.block<3, 3>(3*M.size(), i*3) = M[i].toRotationMatrix();
		A.block<3, 3>(i*3, 3*M.size()) = M[i].toRotationMatrix().transpose();
	}
	//std::cout << A << std::endl;

	std::vector<Eigen::Vector3f> t(prList.size());
	for (int i = 0; i < t.size(); ++i) t[i] = toRigidTransformation(prList[i]->getTransformation()).block<3, 1>(0, 3);

	Eigen::VectorXf b= Eigen::VectorXf::Zero(3*M.size()+3);
	for (int i = 0; i < t.size(); ++i) b.block<3, 1>(3*i, 0) = t[i] * 2.0f;

	Eigen::MatrixXf x = A.fullPivLu().solve(b);

	float error = (A*x-b).norm()/b.norm();
	std::cout << "solution relavtive error: " << error << std::endl;

	std::vector<Eigen::Vector3f> newt(prList.size());
	for (int i = 0; i < t.size(); ++i) newt[i] = x.block<3,1>(3*i, 0);

	for (int i = 0; i < prList.size(); ++i)
	{
		Eigen::Matrix4f transformation_temp = Eigen::Matrix4f::Zero();
		transformation_temp.block<3, 3>(0, 0) = newR[i].toRotationMatrix();
		transformation_temp.block<3, 1>(0, 3) = newt[i];
		transformation_temp(3, 3) = 1.0f;

		prList[i]->initializeTransformation(transformation_temp);
	}
}

void CycleRegistration::non_uniform_refine()
{

}

void CycleRegistration::minimize_refine()
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
		cloudNameCycle.append(prList[i]->getTarget()->objectName()).append(",");
	}
	cloudNameCycle.append(prList.back()->getTarget()->objectName());
	//qDebug() << cloudNameCycle;
	cycleRegistration->setObjectName(cloudNameCycle);
	return cycleRegistration;
}

CycleRegistration* CycleRegistrationManager::getCycleRegistration(QString cloudNameCycle)
{
	return findChild<CycleRegistration*>(cloudNameCycle);
}

GlobalRegistration::GlobalRegistration(QList<PairwiseRegistration*> prList, QList<CycleRegistration*> crList, QObject *parent) : QObject(parent)
{
	this->prList = prList;
	this->crList = crList;
}

GlobalRegistration::~GlobalRegistration()
{

}