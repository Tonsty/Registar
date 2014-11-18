#ifndef PCLBASE_H
#define PCLBASE_H

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h> 
#include <pcl/features/boundary.h>
#include <pcl/PolygonMesh.h>

namespace registar
{
	typedef pcl::PointXYZRGBNormal PointType;
	typedef pcl::PointCloud<PointType> CloudData;
	typedef CloudData::Ptr CloudDataPtr;
	typedef CloudData::ConstPtr CloudDataConstPtr;
	typedef pcl::PointCloud<pcl::Boundary> Boundaries;
	typedef Boundaries::Ptr BoundariesPtr;
	typedef Boundaries::ConstPtr BoundariesConstPtr;
	typedef pcl::search::KdTree<PointType> KdTree;
	typedef KdTree::Ptr KdTreePtr;
	typedef KdTree::ConstPtr KdTreeConstPtr;
	typedef pcl::PolygonMesh PolygonMesh;
	typedef PolygonMesh::Ptr PolygonMeshPtr;
	typedef PolygonMesh::ConstPtr PolygonMeshConstPtr;
	typedef pcl::Vertices Polygon;
	typedef std::vector<Polygon> Polygons;
}

#endif



