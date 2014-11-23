#include <vtkRenderWindow.h>
#include <vtkCamera.h>

#include "pcl_visualizer2.h"

pcl::visualization::PCLVisualizer2::PCLVisualizer2 (const std::string &name, const bool create_interactor) : PCLVisualizer(name, create_interactor) {}

pcl::visualization::PCLVisualizer2::~PCLVisualizer2 () { this->~PCLVisualizer(); }

void pcl::visualization::PCLVisualizer2::renderView2 (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
	vtkSmartPointer<vtkRendererCollection> rens_ = this->getRendererCollection();
	vtkSmartPointer<vtkRenderWindow> win_ = this->getRenderWindow();

  	if (rens_->GetNumberOfItems () > 1)
 	{
    	PCL_WARN("[renderView] Method will render only the first viewport\n");
    	return;
  	}

  	int *size = win_->GetSize();
  	int xres = size[0], yres = size[1];

  	win_->Render ();

  	float dwidth = 2.0f / float (xres), dheight = 2.0f / float (yres);

  	cloud->points.resize (xres * yres);
  	cloud->width = xres;
  	cloud->height = yres;

  	float *depth = new float[xres * yres];
  	win_->GetZbufferData (0, 0, xres - 1, yres - 1, &(depth[0]));

  	// Transform cloud to give camera coordinates instead of world coordinates!
  	vtkRenderer *ren = rens_->GetFirstRenderer ();
  	vtkCamera *camera = ren->GetActiveCamera ();
  	vtkSmartPointer<vtkMatrix4x4> projection_transform = camera->GetProjectionTransformMatrix(ren->GetTiledAspectRatio(), 0, 1);
  	vtkSmartPointer<vtkMatrix4x4> view_transform = camera->GetViewTransformMatrix();

  	Eigen::Matrix4f mat2, mat3;
  	for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
    {
      	mat2 (i, j) = static_cast<float> (view_transform->Element[i][j]);
      	mat3 (i, j) = static_cast<float> (projection_transform->Element[i][j]);
    }

  	// std::cout << "ren->GetTiledAspectRatio () : "<< ren->GetTiledAspectRatio() << std::endl;
  	// double *aspect = ren->GetAspect();
  	// std::cout << "ren->GetAspect() : " << aspect[0] << " , " << aspect[1] << std::endl;
  	// double *position = camera->GetPosition();
  	// std::cout << "camera->GetPosition() : " << position[0] << " , " << position[1] << " , " << position[2] << std::endl;
  	// double *focal_point = camera->GetFocalPoint();
  	// std::cout << "camera->GetFocalPoint() : " << focal_point[0] << " , " << focal_point[1] << " , " << focal_point[2] << std::endl;  
  	// double *view_up = camera->GetViewUp();
  	// std::cout << "camera->GetViewUp() : " << view_up[0] << " , " << view_up[1] << " , " << view_up[2] << std::endl;    
  	// double *clipping_range = camera->GetClippingRange();
  	// std::cout << "camera->GetClippingRange() : " << clipping_range[0] << " , " << clipping_range[1] << std::endl;
  	// std::cout << "camera->GetViewAngle() : " << camera->GetViewAngle() << std::endl;

  	// std::cout << "mat2:\n" << mat2 << std::endl;
  	// std::cout << "mat3:\n" << mat3 << std::endl;

  	Eigen::Matrix4f temp2 = mat2.inverse();
  	mat2 = temp2;

  	Eigen::Matrix4f temp3 =  mat3.inverse();
  	mat3 = temp3;  

  	int ptr = 0;
  	for (int y = 0; y < yres; ++y)
  	{
    	for (int x = 0; x < xres; ++x, ++ptr)
    	{
	      	pcl::PointXYZ &pt = (*cloud)[ptr];

      		if (depth[ptr] == 1.0)
      		{
        		pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN ();
        		continue;
      		}

      		Eigen::Vector4f world_coords (dwidth  * float (x) - 1.0f, dheight * float (y) - 1.0f, depth[ptr], 1.0f);
      		world_coords = mat3 * world_coords;

      		float w3 = 1.0f / world_coords[3];
      		world_coords[0] *= w3;
      		world_coords[1] *= w3;
      		world_coords[2] *= w3;
      		world_coords[3] = 1;

      		world_coords = mat2 * world_coords;

      		w3 = 1.0f / world_coords[3];
      		world_coords[0] *= w3;
      		world_coords[1] *= w3;
      		world_coords[2] *= w3;
      		world_coords[3] = 1;

      		pt.x = static_cast<float> (world_coords[0]);
      		pt.y = static_cast<float> (world_coords[1]);
      		pt.z = static_cast<float> (world_coords[2]);
    	}
  	}

  	delete[] depth;	
}
