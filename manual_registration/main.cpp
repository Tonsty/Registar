#include "manual_registration.h"

void print_usage ()
{
  PCL_INFO ("manual_registration cloud1.ply cloud2.ply\n");
  PCL_INFO ("\t cloud1 \t source cloud\n");
  PCL_INFO ("\t cloud2 \t destination cloud\n");
}

int main (int argc, char** argv)
{
  QApplication app(argc, argv);

  pcl::PointCloud<PointT>::Ptr cloud_src (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_dst (new pcl::PointCloud<PointT>);

  if(argc < 3)
  {
    PCL_ERROR ("Incorrect usage\n");
    print_usage();
  }

  // TODO do this with PCL console
  pcl::PLYReader reader1;
  if (reader1.read(argv[1], *cloud_src) != 0) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }
  pcl::PLYReader reader2;
  if (reader2.read(argv[2], *cloud_dst) != 0) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[2]);
    return (-1);
  }

  cloud_src->sensor_orientation_ = Eigen::Quaternionf( 1.0, 0.0, 0.0, 0.0 );
  cloud_src->sensor_origin_ = Eigen::Vector4f( 0.0, 0.0, 0.0, 0.0 );
  cloud_dst->sensor_orientation_ = Eigen::Quaternionf( 1.0, 0.0, 0.0, 0.0 );
  cloud_dst->sensor_origin_ = Eigen::Vector4f( 0.0, 0.0, 0.0, 0.0 );


  ManualRegistration man_reg;

  man_reg.setSrcCloud(cloud_src);
  man_reg.setDstCloud(cloud_dst);

  man_reg.showSrcCloud();
  man_reg.showDstCloud();

  man_reg.show();

  return (app.exec());
}