#ifndef VIRTUALSCAN_H
#define VIRTUALSCAN_H

#include "pclbase.h"
#include <vtkPolyData.h>

namespace registar
{
	class VirtualScan
	{
	public:
		static void scan(const int nr_scans, const int nr_points_in_scans, const double vert_res, const double hor_res, const double max_dist,
								const Eigen::Matrix4f &pose, vtkSmartPointer<vtkPolyData> vtk_polydata, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
	};
}

#endif