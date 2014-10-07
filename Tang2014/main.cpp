#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>

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

  // 	for (int i = 0; i < scanPtrs.size(); ++i)
  // 	{
  // 		std::string filePath = scanPtrs[i]->filePath;
		// QFileInfo fileInfo(QString(filePath.c_str()));
		// QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";
		// QFile tfFile(tfFileName);
		// tfFile.open(QIODevice::WriteOnly);
		// QTextStream out(&tfFile);			
		// Transformation transformation = globalRegistration.transformations[i] * scanPtrs[i]->transformation;
		// for (int i = 0; i < 4; ++i)
		// {
		// 	for (int j = 0; j < 4; ++j)
		// 		out << transformation(i, j) << " ";
		// 	out << "\n";
		// }
		// tfFile.close();
  // 	}

	return 0;
}
