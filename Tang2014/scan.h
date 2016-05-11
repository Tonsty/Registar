#ifndef SCAN_H
#define SCAN_H

#include "common.h"

#include <vector>
#include <Eigen/Dense>

namespace tang2014
{
	struct Scan
	{
		PointsPtr pointsPtr;
		BoundariesPtr boundariesPtr;
		
		Transformation transformation;
		std::string filePath;

		typedef boost::shared_ptr<Scan> Ptr;
	};
	typedef Scan::Ptr ScanPtr;
	typedef std::vector<ScanPtr> ScanPtrs; 

	void importScanPtrs(const std::string fileName, ScanPtrs &scanPtrs );
}

#endif


