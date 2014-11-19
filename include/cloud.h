#ifndef CLOUD_H
#define CLOUD_H

#include <QtCore/QObject>

#ifndef Q_MOC_RUN
#include "pclbase.h"
// #include <boost/shared_ptr.hpp>
#endif

namespace registar
{
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

		Cloud(CloudDataPtr cloudData = CloudDataPtr(), const Polygons &polygons = Polygons(0), const FromWhere &fromWhere = fromNew, 
			const QString &fileName = "", const Eigen::Matrix4f &transformation = Eigen::Matrix4f::Identity(),
			const QString &cloudName = "", QObject *parent = 0);
		virtual ~Cloud();

		void setCloudData(CloudDataPtr cloudData);
		CloudDataConstPtr getCloudData()const;

		void setPolygons(const Polygons &polygons);
		const Polygons& getPolygons()const;

		void setFromWhere(const FromWhere &fromWhere);
		const FromWhere &getFromWhere()const;

		void setFileName(const QString &fileName);
		const QString &getFileName()const;

		void setCloudName(const QString &cloudName);
		const QString &getCloudName()const;

		void setTransformation(const Eigen::Matrix4f &transformation);
		const Eigen::Matrix4f &getTransformation()const;

		void setRegistrationTransformation(const Eigen::Matrix4f &registrationTransformation);
		const Eigen::Matrix4f &getRegistrationTransformation()const;

		void setBoundaries(BoundariesPtr boundaries);
		BoundariesConstPtr getBoundaries()const;

	protected:
		CloudDataPtr cloudData;
		Polygons polygons;
		QString fileName;
		FromWhere fromWhere;
		Eigen::Matrix4f transformation;
		Eigen::Matrix4f registrationTransformation;
		BoundariesPtr boundaries;
	};
}

#endif
