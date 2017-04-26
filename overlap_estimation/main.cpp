#include <iostream>
#include <algorithm>

#ifndef _OPENMP
#define _OPENMP
#endif

#include "../Tang2014/scan.h"
#include "../Tang2014/pairregistration.h"
#include "../Tang2014/globalregistration.h"

#include <pcl/console/parse.h>

using namespace tang2014;

struct OverlapInfo
{
	Link link;
	int pointPairNum;
	float fracInA;
	float fracInB;
};

bool overlapInfoComp(const OverlapInfo &a, const OverlapInfo &b)
{
	return a.pointPairNum > b.pointPairNum;
}

int main(int argc, char** argv)
{
	std::vector<int> p_file_indices_scans = pcl::console::parse_file_extension_argument (argc, argv, ".scans");
  	ScanPtrs scanPtrs;
  	importScanPtrs(argv[p_file_indices_scans[0]], scanPtrs);

	Links links;
	Loops loops;
  	GlobalRegistration globalRegistration(scanPtrs, links, loops);
  	globalRegistration.buildKdTreePtrs();

	PairRegistration::Parameters pr_para;
	pr_para.mMethod = PairRegistration::POINT_TO_PLANE;
	pr_para.sMethod = PairRegistration::UMEYAMA;
	pr_para.distanceTest = true;
	pr_para.angleTest = true;
	pr_para.boundaryTest = true;
	pr_para.biDirection = true;
	pr_para.iterationNum_max = 0;
	pr_para.iterationNum_min = 0;

	float distThreshold;
	pcl::console::parse_argument(argc, argv, "--distance", distThreshold);
	float angleThreshold;
	pcl::console::parse_argument(argc, argv, "--angle", angleThreshold);
	pr_para.distThreshold = distThreshold;
	pr_para.angleThreshold = angleThreshold;

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
  		for (int j = i+1; j < scanPtrs.size(); ++j)
  		{
  			Link link;
  			link.a = i;
  			link.b = j;

			// PairRegistrationPtr pairReigstrationPtr(new PairRegistration(scanPtrs[link.a], scanPtrs[link.b]));
			PairRegistrationOMPPtr pairReigstrationPtr(new PairRegistrationOMP(scanPtrs[link.a], scanPtrs[link.b], 8));
			pairReigstrationPtr->setKdTree( globalRegistration.kdTreePtrs[link.a], globalRegistration.kdTreePtrs[link.b] );

			pairReigstrationPtr->setParameter(pr_para);
			pairReigstrationPtr->setTransformation(Transformation::Identity());
			pairReigstrationPtr->initiateCandidateIndices();

			std::cerr << i << " <<-- " << j << std::endl;
			pairReigstrationPtr->generateFinalPointPairs(Transformation::Identity());

			std::pair<Link, PairRegistrationPtr> pairLP(link, pairReigstrationPtr);
			globalRegistration.pairRegistrationPtrMap.insert(pairLP);
  		}
  	}

  	std::vector<OverlapInfo> totalOverlapInfo;

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
  		for (int j = i+1; j < scanPtrs.size(); ++j)
  		{
  			Link link;
  			link.a = i;
  			link.b = j;

  			PairRegistrationPtr pairReigstrationPtr = globalRegistration.pairRegistrationPtrMap[link];

  			OverlapInfo overlapInfo;
  			overlapInfo.link = link;
  			overlapInfo.pointPairNum = pairReigstrationPtr->final_s2t.size();
  			overlapInfo.fracInA = pairReigstrationPtr->final_s2t.size() * 50.0f / scanPtrs[link.a]->pointsPtr->size();
  			overlapInfo.fracInB = pairReigstrationPtr->final_s2t.size() * 50.0f / scanPtrs[link.b]->pointsPtr->size();

			totalOverlapInfo.push_back(overlapInfo);  			

		  	// std::cout << link.a << " : " << scanPtrs[link.a]->filePath << " <<-- " << link.b << " : "<< scanPtrs[link.b]->filePath << " ";
		    // std::cout << std::setw(2) << link.a << " (" << scanPtrs[link.a]->pointsPtr->size() <<") <<-- " << std::setw(2) << link.b << " (" << scanPtrs[link.b]->pointsPtr->size() << ") "
		    //  		 << std::setw(5) << pairReigstrationPtr->final_s2t.size() << " " 
			//      	 << std::setw(3) << (int)( pairReigstrationPtr->final_s2t.size() * 50.0f / scanPtrs[link.a]->pointsPtr->size() ) << "\% "
			//      	 << std::setw(3) << (int)( pairReigstrationPtr->final_s2t.size() * 50.0f / scanPtrs[link.b]->pointsPtr->size() ) << "\% " 
			//      	 << std::endl;
  		}
	}

	sort(totalOverlapInfo.begin(), totalOverlapInfo.end(), overlapInfoComp);

	//std::cout << "Totally Ordered : " << std::endl;

	for (int i = 0; i < totalOverlapInfo.size(); ++i)
	{
		Link link = totalOverlapInfo[i].link;
		int pointPairNum = totalOverlapInfo[i].pointPairNum;
		float fracInA = totalOverlapInfo[i].fracInA;
		float fracInB = totalOverlapInfo[i].fracInB;	

		std::cout << "("<< std::setw(3) << i << ") : "
				<< std::setw(2) << link.a 
				<< " (" << std::setw(7) << scanPtrs[link.a]->pointsPtr->size() <<")" 
				<< " <<-- " 
				<< std::setw(2) << link.b 
				<< " (" << std::setw(7) << scanPtrs[link.b]->pointsPtr->size() << ") "
		    	<< std::setw(7) << pointPairNum << " " 
				<< std::setw(3) << (int)( fracInA ) << "\% "
				<< std::setw(3) << (int)( fracInB ) << "\% " 
				<< std::endl;		
	}

	//std::cout << "\n\n\nPartially Ordered : " << std::endl; 

  	for (int i = 0; i < scanPtrs.size(); ++i)
  	{
  		std::vector<OverlapInfo> partOverlapInfo;

  		for (int j = 0; j < totalOverlapInfo.size(); ++j)
  		{
  			Link link = totalOverlapInfo[j].link;
  			if ( link.a == i || link.b == i)
  			{
  				partOverlapInfo.push_back(totalOverlapInfo[j]);
  			}
  		}

  		std::cerr << std::setw(2) << i 
  					<< "("<< std::setw(7) << scanPtrs[i]->pointsPtr->size() << ")"
  					<< " <<-- ";

  		for (int j = 0; j < partOverlapInfo.size(); ++j)
  		{
  			Link link = partOverlapInfo[j].link;
  			// int pointPairNum = partOverlapInfo[j].pointPairNum;
			float fracInA = partOverlapInfo[j].fracInA;
			float fracInB = partOverlapInfo[j].fracInB;	

  			if ( link.a == i)
  			{
  				std::cerr << std::setw(2) << link.b 
  							<< " (" 
  							// << std::setw(7) << pointPairNum 
  							// << ", " 
  							<< std::setw(3) << (int)( fracInA ) << "\%"
  							<< ")"
							<< ", ";
  			}
  			else
  			{
  				std::cerr << std::setw(2) << link.a 
  							<< " (" 
  							// << std::setw(7) << pointPairNum 
  							// << ", " 
  							<< std::setw(3) << (int)( fracInB ) << "\%"
  							<< ")"
							<< ", ";  				
  			}
  		}

  		std::cerr << std::endl;

  	}

	 for (int i = 0; i < totalOverlapInfo.size(); ++i)
	 {
	 	Link link = totalOverlapInfo[i].link;
	 	int pointPairNum = totalOverlapInfo[i].pointPairNum;
	 	float fracInA = totalOverlapInfo[i].fracInA;
	 	float fracInB = totalOverlapInfo[i].fracInB;	

	 	std::cerr << "("<< i << ") : "
	 			<< std::setw(2) << link.a << " (" << scanPtrs[link.a]->pointsPtr->size() <<") <<-- " << std::setw(2) << link.b << " (" << scanPtrs[link.b]->pointsPtr->size() << ") "
	 	    	<< std::setw(5) << pointPairNum << " " 
	 			<< std::setw(3) << (int)( fracInA ) << "\% "
	 			<< std::setw(3) << (int)( fracInB ) << "\% " 
	 			<< std::endl;		
	 }	

	return 0;
} 
