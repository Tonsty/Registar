#include "../include/cloud.h"

using namespace registar;

Cloud::Cloud(CloudDataPtr cloudData, const Polygons &polygons, const FromWhere &fromWhere,
	const QString &fileName, const Eigen::Matrix4f &transformation,
	const QString &cloudName, QObject *parent) : QObject(parent)
{
	this->cloudData = cloudData;
	this->polygons = polygons;
	this->fromWhere = fromWhere; 
	this->fileName = fileName;
	this->transformation = transformation;
	this->registrationTransformation = transformation;
	this->setObjectName(cloudName);
}

Cloud::~Cloud(){}

void Cloud::setCloudData(CloudDataPtr cloudData)
{
	this->cloudData = cloudData;
}

CloudDataPtr Cloud::getCloudData()
{
	return this->cloudData;
}

void Cloud::setFromWhere(FromWhere fromWhere)
{
	this->fromWhere = fromWhere;
}

Cloud::FromWhere Cloud::getFromWhere()
{
	return this->fromWhere;
}

void Cloud::setFileName(const QString &_fileName)
{
	this->fileName = fileName;
}

QString Cloud::getFileName()
{
	return this->fileName;
}

void Cloud::setCloudName(const QString &cloudName)
{
	this->setObjectName(cloudName);
}

QString  Cloud::getCloudName()
{
	return this->objectName();
}

void Cloud::setTransformation(const Eigen::Matrix4f &transformation)
{
	this->transformation = transformation;
}

Eigen::Matrix4f Cloud::getTransformation()
{
	return this->transformation;
}

void Cloud::setRegistrationTransformation(const Eigen::Matrix4f &registrationTransformation)
{
	this->registrationTransformation = registrationTransformation;
}

Eigen::Matrix4f Cloud::getRegistrationTransformation()
{
	return this->registrationTransformation;
}

void Cloud::setBoundaries(BoundariesPtr boundaries)
{
	this->boundaries = boundaries;
}

BoundariesPtr Cloud::getBoundaries()
{
	return this->boundaries;
}

void Cloud::setPolygons(const Polygons &polygons)
{
	this->polygons = polygons;
}

Polygons Cloud::getPolygons()
{
	return this->polygons;
}