#include <pcl/visualization/vtk.h>

#include "../include/virtualscan.h"

using namespace registar;

void VirtualScan::scan(const int nr_scans, const int nr_points_in_scans, const double vert_res, const double hor_res, const double max_dist,
								const Eigen::Matrix4f &pose, vtkSmartPointer<vtkPolyData> vtk_polydata, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	//// Compute start/stop for vertical and horizontal
	//double vert_start = - (static_cast<double> (nr_scans - 1) / 2.0) * vert_res;
	//double vert_end   = + ((nr_scans-1) * vert_res) + vert_start;
	//double hor_start  = - (static_cast<double> (nr_points_in_scans - 1) / 2.0) * hor_res;
	//double hor_end    = + ((nr_points_in_scans-1) * hor_res) + hor_start;

	//// Virtual camera parameters
	//double eye[3]     = {0.0, 0.0, 0.0};
	//double viewray[3] = {0.0, 0.0, 0.0};
	//double up[3]      = {0.0, 0.0, 0.0};
	//double right[3]  = {0.0, 0.0, 0.0};
	//double x_axis[3] = {1.0, 0.0, 0.0};
	//double z_axis[3] = {0.0, 0.0, 1.0};
	//double bounds[6];
	//double temp_beam[3], beam[3], p[3];
	//double p_coords[3], x[3], t;
	//int subId;

	//// Build a spatial locator for our dataset
	//vtkSmartPointer<vtkCellLocator> tree = vtkSmartPointer<vtkCellLocator>::New ();
	//tree->SetDataSet (vtk_polydata);
	//tree->CacheCellBoundsOn ();
	//tree->SetTolerance (0.0);
	//tree->SetNumberOfCellsPerBucket (1);
	//tree->AutomaticOn ();
	//tree->BuildLocator ();
	//tree->Update ();

	//// Get the min-max bounds of data
	//vtk_polydata->GetBounds (bounds);

	//int sid = -1;

}