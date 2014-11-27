#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include "../include/virtualscan.h"

using namespace registar;

void VirtualScan::scan(const int nr_scans, const int nr_points_in_scans, const double vert_res, const double hor_res, const double max_dist,
								vtkSmartPointer<vtkPolyData> vtk_polydata, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{

}