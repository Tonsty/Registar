#ifndef CLOUD_H
#define CLOUD_H

#include <QtCore/QObject>
// #include <boost/shared_ptr.hpp>

#include "pclbase.h"

class Cloud : public QObject
{
	Q_OBJECT

public:
	// typedef boost::shared_ptr<Cloud> Ptr;
	// typedef boost::shared_ptr<const Cloud> ConstPtr;

	enum FromWhere
	{
		fromNew, fromIO, fromFilter
	};

	Cloud(CloudDataPtr cloudData = CloudDataPtr(), const FromWhere &fromWhere = fromNew, 
		const QString &fileName = "", const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity(),
		const QString &cloudName = "", QObject *parent = 0);
	virtual ~Cloud();

	void setCloudData(CloudDataPtr cloudData);
	CloudDataPtr getCloudData();

	void setFromWhere(FromWhere fromWhere);
	FromWhere getFromWhere();

	void setFileName(const QString &fileName);
	QString getFileName();

	void setCloudName(const QString &cloudName);
	QString  getCloudName();

	void setTransformation(Eigen::Matrix4f transformation);
	Eigen::Matrix4f getTransformation();

	void setRegistrationTransformation(Eigen::Matrix4f registrationTransformation);
	Eigen::Matrix4f getRegistrationTransformation();

	void setBoundaries(BoundariesPtr boundaries);
	BoundariesPtr getBoundaries();

protected:
	CloudDataPtr cloudData;
	QString fileName;
	FromWhere fromWhere;
	Eigen::Matrix4f transformation;

	Eigen::Matrix4f registrationTransformation;
	BoundariesPtr boundaries;
};

#endif
