#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>

#include "link.h"
#include "loop.h"
#include "scan.h"
#include "globalregistration.h"

#include "../Williams2001/SRoMCPS.h"

#include <pcl/console/parse.h>

using namespace Tang2014;

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

  	GlobalRegistration::Parameters gr_para;
  	//gr_para.doInitialPairRegistration = true;
  	//gr_para.doIncrementalLoopRefine = true;
  	//gr_para.doGlobalRefine = false;

	//gr_para.doInitialPairRegistration = false;
	//gr_para.doIncrementalLoopRefine = false;
	//gr_para.doGlobalRefine = true;

	gr_para.doInitialPairRegistration = true;
	gr_para.doIncrementalLoopRefine = false;
	gr_para.doGlobalRefine = true;

  	gr_para.globalIterationNum_max = 50;
  	gr_para.globalIterationNum_min = 30;

  	PairRegistration::Parameters pr_para;
  	pr_para.mMethod = PairRegistration::POINT_TO_PLANE;
  	pr_para.sMethod = PairRegistration::UMEYAMA;
  	pr_para.distanceTest = true;
  	pr_para.distThreshold = 0.001f;
  	pr_para.angleTest = true;
  	pr_para.angleThreshold = 45.0f;
  	pr_para.boundaryTest = true;
  	pr_para.biDirection = true;
  	pr_para.iterationNum_max = 10;
  	pr_para.iterationNum_min = 5;

  	gr_para.pr_para = pr_para;

  	globalRegistration.setParameters(gr_para);

  	globalRegistration.startRegistration();

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
  		std::string filePath = scanPtrs[i]->filePath;
  		QFileInfo fileInfo(QString(filePath.c_str()));
  		QString tfFileName = fileInfo.path() + "/" + fileInfo.completeBaseName() + ".tf";
  		QFile tfFile(tfFileName);
  		tfFile.open(QIODevice::WriteOnly);
  		QTextStream out(&tfFile);			
  		Transformation transformation = globalRegistration.transformations[i] * scanPtrs[i]->transformation;
  		for (int i = 0; i < 4; ++i)
  		{
  			for (int j = 0; j < 4; ++j)
  				out << transformation(i, j) << " ";
  			out << "\n";
  		}
  		tfFile.close();
  	}

	return 0;
}
