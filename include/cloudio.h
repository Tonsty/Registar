#ifndef CLOUDIO_H
#define CLOUDIO_H

#include <QtCore/QObject>

#include "pclbase.h"

namespace registar
{
	class CloudIO
	{
	public:
		CloudIO();
		virtual ~CloudIO();

		static bool importPLYCloudData(const QString &fileName, CloudDataPtr cloudData);
		static bool exportPLYCloudData(const QString &fileName, CloudDataConstPtr cloudData);

		static bool importPLYPolygonMesh(const QString &fileName, PolygonMeshPtr polygonMesh);
		static bool exportPLYPolygonMesh(const QString &fileName, PolygonMeshConstPtr polygonMesh);

		static bool importTransformation(const QString &fileName, Eigen::Matrix4f &transformation);
		static bool exportTransformation(const QString &fileName, const Eigen::Matrix4f &transformation);

		static bool importBoundaries(const QString &fileName, BoundariesPtr boundaries);
		static bool exportBoundaries(const QString &fileName, BoundariesConstPtr boundaries);

		static bool importVTKCloudData(const QString &fileName, CloudDataPtr cloudData);
		static bool exportVTKCloudData(const QString &fileName, CloudDataConstPtr cloudData);

		static bool importVTKPolygonMesh(const QString &fileName, PolygonMeshPtr polygonMesh);
		static bool exportVTKPolygonMesh(const QString &fileName, PolygonMeshConstPtr polygonMesh );
	};
}

#endif
