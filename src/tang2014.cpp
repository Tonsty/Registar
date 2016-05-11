#include "../include/tang2014.h"
#include "../include/args_converter.h"

#include <pcl/console/parse.h>

namespace registar {

	tang2014::Transformations Tang2014::startRegistration(tang2014::ScanPtrs scanPtrs, QVariantMap parameters) {

		std::string links_str = parameters["links"].toString().toStdString();
		std::stringstream links_sstr(links_str);

		tang2014::Links links;
		tang2014::ScanIndex a, b;
		int i = 0;
		while( links_sstr >> a >> b ) 
		{
			std::cout << "link " << i << " : " << a << " " << b << std::endl;
			tang2014::Link link;
			link.a = a;
			link.b = b;
			links.push_back(link);
			i++;
		}

		tang2014::Loops loops;
		std::string loops_str = parameters["loops"].toString().toStdString();
		std::stringstream loops_sstr(loops_str);

		char dummy[300];
		int j = 0;
		while( loops_sstr.getline(dummy, 300) )
		{
			std::stringstream sstr(dummy);
			tang2014::ScanIndex scanIndex;
			std::cout << "loop " << j << " : ";
			tang2014::Loop temp;
			while( sstr >> scanIndex ) 
			{
				std::cout << scanIndex << " ";
				temp.scanIndices.push_back(scanIndex);
			}
			std::cout << std::endl;
			loops.push_back(temp);
			j++;
		}

		args_converter ac("tang2014.exe", parameters["arguments"].toString().replace('\n', ' ').toStdString());
		int argc = ac.get_argc();
		char** argv = ac.get_argv();

		std::cout << "argc = " << argc << std::endl;
		for(int i = 0; i < argc; i++) std::cout << argv[i] << std::endl;

		tang2014::GlobalRegistration globalRegistration( scanPtrs, links, loops );
		tang2014::GlobalRegistration::Parameters gr_para;

		gr_para.doInitialPairRegistration = pcl::console::find_switch(argc, argv, "--ipr");
		gr_para.doIncrementalLoopRefine = pcl::console::find_switch(argc, argv, "--il");
		gr_para.doGlobalRefine = pcl::console::find_switch(argc, argv, "--gr");

		int gi_max, gi_min;
		pcl::console::parse_argument(argc, argv, "--gi_max", gi_max);
		pcl::console::parse_argument(argc, argv, "--gi_min", gi_min);
		gr_para.globalIterationNum_max = gi_max;
		gr_para.globalIterationNum_min = gi_min;

		int pi_num;
		pcl::console::parse_argument(argc, argv, "--pi_num", pi_num);
		gr_para.pairIterationNum = pi_num;

		tang2014::PairRegistration::Parameters pr_para;
		pr_para.mMethod = tang2014::PairRegistration::POINT_TO_PLANE;
		pr_para.sMethod = tang2014::PairRegistration::UMEYAMA;
		pr_para.distanceTest = true;
		pr_para.angleTest = true;
		pr_para.boundaryTest = true;
		pr_para.biDirection = true;

		int pi_max, pi_min;
		pcl::console::parse_argument(argc, argv, "--pi_max", pi_max);
		pcl::console::parse_argument(argc, argv, "--pi_min", pi_min);
		pr_para.iterationNum_max = pi_max;
		pr_para.iterationNum_min = pi_min;

		float distThreshold, angleThreshold;
		pcl::console::parse_argument(argc, argv, "--distance", distThreshold);
		pcl::console::parse_argument(argc, argv, "--angle", angleThreshold);
		pr_para.distThreshold = distThreshold;
		pr_para.angleThreshold = angleThreshold;

		gr_para.pr_para = pr_para;

		globalRegistration.setParameters(gr_para);
		globalRegistration.startRegistration();

		return globalRegistration.transformations;
	}

};