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

CloudDataConstPtr Cloud::getCloudData()const
{
	return this->cloudData;
}

CloudDataPtr Cloud::getCloudData() 
{
	return this->cloudData;
}

void Cloud::setFromWhere(const FromWhere &fromWhere)
{
	this->fromWhere = fromWhere;
}

const Cloud::FromWhere& Cloud::getFromWhere()const
{
	return this->fromWhere;
}

void Cloud::setFileName(const QString &fileName)
{
	this->fileName = fileName;
}

const QString& Cloud::getFileName()const
{
	return this->fileName;
}

void Cloud::setCloudName(const QString &cloudName)
{
	this->setObjectName(cloudName);
}

const QString&  Cloud::getCloudName()const
{
	return this->objectName();
}

void Cloud::setTransformation(const Eigen::Matrix4f &transformation)
{
	this->transformation = transformation;
}

const Eigen::Matrix4f& Cloud::getTransformation()const
{
	return this->transformation;
}

void Cloud::setRegistrationTransformation(const Eigen::Matrix4f &registrationTransformation)
{
	this->registrationTransformation = registrationTransformation;
}

const Eigen::Matrix4f& Cloud::getRegistrationTransformation()const
{
	return this->registrationTransformation;
}

void Cloud::setBoundaries(BoundariesPtr boundaries)
{
	this->boundaries = boundaries;
}

BoundariesConstPtr Cloud::getBoundaries()const
{
	return this->boundaries;
}

BoundariesPtr Cloud::getBoundaries()
{
	return this->boundaries;
}

void Cloud::setPolygons(const Polygons &polygons)
{
	this->polygons = polygons;
}

const Polygons &Cloud::getPolygons()const
{
	return this->polygons;
}