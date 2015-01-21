#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char **argv)
{
	std::vector<int> ply_file_indices_scans = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	std::string input_filename = argv[ply_file_indices_scans[0]];
	std::string output_filename = argv[ply_file_indices_scans[1]];

	float voxel_size;
	pcl::console::parse_argument(argc, argv, "--voxel_size", voxel_size);

	std::cout << "voxel_size : " << voxel_size << std::endl;

	pcl::PLYReader reader;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud1(new pcl::PointCloud<PointT>);
	pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh);
	reader.read(input_filename, *mesh);
	pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
	pcl::VoxelGrid<PointT> filter;
	filter.setInputCloud(cloud);
	filter.setLeafSize(voxel_size, voxel_size, voxel_size);
	filter.filter(*cloud1);
	pcl::PLYWriter writer;
	writer.write(output_filename, *cloud1);
	return 0;
}
