#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/boundary.h>

namespace tang2014
{
	typedef pcl::PointXYZRGBNormal Point;
	typedef pcl::Boundary Boundary;

	typedef Eigen::Matrix4f Transformation;

	typedef pcl::PointCloud<Point> Points;
	typedef pcl::PointCloud<Boundary> Boundaries;

	typedef Points::Ptr PointsPtr;
	typedef Boundaries::Ptr BoundariesPtr; 

	typedef pcl::search::KdTree<Point> KdTree;
	typedef KdTree::Ptr KdTreePtr;
	typedef std::vector<KdTreePtr> KdTreePtrs;

	typedef std::vector<Transformation> Transformations;
}
