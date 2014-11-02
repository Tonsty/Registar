 #include "set_color.h"

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

#include "../Tang2014/scan.h"

#include "set_color.h"

using namespace Tang2014;

int main(int argc, char**argv)
{
	std::vector<int> p_file_indices_scans = pcl::console::parse_file_extension_argument (argc, argv, ".scans");
  	ScanPtrs scanPtrs;
  	importScanPtrs(argv[p_file_indices_scans[0]], scanPtrs);

  	std::vector<RGB> rgbs = generateUniformColors(scanPtrs.size(), 60, 300);

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
  		for (int j = 0; j < scanPtrs[i]->pointsPtr->size(); ++j)
  		{
  			(*scanPtrs[i]->pointsPtr)[j].r = rgbs[i].r;
  			(*scanPtrs[i]->pointsPtr)[j].g = rgbs[i].g;
  			(*scanPtrs[i]->pointsPtr)[j].b = rgbs[i].b;  			  			
  		}
  	}

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
		pcl::PLYWriter plyWriter;
		plyWriter.write(scanPtrs[i]->filePath, *scanPtrs[i]->pointsPtr);
  	}
  	
	return 0;
}

	// RGB rgb;
	// rgb.r = 0;
	// rgb.g = 0;
	// rgb.b = 127;

	// HSV hsv = rgb2hsv(rgb);

	// std::cout << hsv.h << std::endl;
	// std::cout << hsv.s << std::endl;
	// std::cout << hsv.v << std::endl;

	// hsv.h = 240;
	// hsv.s = 1.0;
	// hsv.v = 0.5;

	// rgb = hsv2rgb(hsv);

	// std::cout << (int)rgb.r << std::endl;
	// std::cout << (int)rgb.g << std::endl;
	// std::cout << (int)rgb.b << std::endl;
