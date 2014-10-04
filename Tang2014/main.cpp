#include "link.h"
#include "loop.h"
#include "scan.h"
#include "globalregistration.h"

#include "SRoMCPS.h"

#include <pcl/console/parse.h>

int main(int argc, char **argv)
{
	std::vector<int> p_file_indices_scans = pcl::console::parse_file_extension_argument (argc, argv, ".scans");
  	ScanPtrs scanPtrs;
  	importScanPtrs(argv[p_file_indices_scans[0]], scanPtrs);

  	std::vector<int> p_file_indices_links = pcl::console::parse_file_extension_argument (argc, argv, ".links");
  	Links links;
  	importLinks(argv[p_file_indices_links[0]], links);

  	std::vector<int> p_file_indices_loops = pcl::console::parse_file_extension_argument (argc, argv, ".loops");
  	Loops loops;
  	importLoops(argv[p_file_indices_loops[0]], loops);

  	GlobalRegistration globalRegistration( scanPtrs, links, loops );
  	globalRegistration.startRegistration();

	return 0;
}
