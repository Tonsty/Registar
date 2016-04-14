#ifndef PCL_VISUALIZER2_H
#define PCL_VISUALIZER2_H

#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
	namespace visualization
	{
		class PCLVisualizer2 : public PCLVisualizer
		{
		public:
        	typedef boost::shared_ptr<PCLVisualizer2> Ptr;
        	typedef boost::shared_ptr<const PCLVisualizer2> ConstPtr;
			
			PCLVisualizer2 (const std::string &name = "", const bool create_interactor = true);
			virtual ~PCLVisualizer2();

        	void renderView2 (const int &xres, const int &yres, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::Matrix4f &pose);
        	void renderViewTesselatedSphere2(int xres, int yres, pcl::PointCloud<pcl::PointXYZ>::CloudVectorType & cloud,
            								std::vector<Eigen::Matrix4f,Eigen::aligned_allocator< Eigen::Matrix4f > > & poses, std::vector<float> & enthropies, 
            								int tesselation_level, float view_angle = 45, float radius_sphere = 1, bool use_vertices = true);
			void renderViewTesselatedSphere3(int xres, int yres, pcl::PointCloud<pcl::PointXYZ>::CloudVectorType & cloud,
				std::vector<Eigen::Matrix4f,Eigen::aligned_allocator< Eigen::Matrix4f > > & poses, std::vector<float> & enthropies, 
				int tesselation_level, float view_angle = 45, float radius_sphere = 1, bool use_vertices = true);
		};
	}
}

#endif 
