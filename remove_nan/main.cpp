#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#define PCL_NO_PRECOMPILE
#include <pcl/filters/filter.h>

int main(int argc, char **argv)
{
	std::vector<int> ply_file_indices_scans = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	std::string input_filename = argv[ply_file_indices_scans[0]];
	std::string output_filename = argv[ply_file_indices_scans[1]];

	pcl::PLYReader reader;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh);
	reader.read(input_filename, *mesh);
	pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
	std::vector<int> nanIndicesVector;
	pcl::removeNaNFromPointCloud( *cloud, *cloud, nanIndicesVector );
	pcl::PLYWriter writer;
	writer.write(output_filename, *cloud);
	return 0;
}
