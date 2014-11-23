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

        	void renderView2 (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
		};
	}
}

#endif 
