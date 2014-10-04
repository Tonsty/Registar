#ifndef SCAN_H
#define SCAN_H

#include "common.h"

#include <vector>
#include <Eigen/Dense>

struct Scan
{
	PointsPtr pointsPtr;
	BoundariesPtr boundariesPtr;
	Transformation transformation;

	typedef boost::shared_ptr<Scan> Ptr;
};
typedef Scan::Ptr ScanPtr;
typedef std::vector<ScanPtr> ScanPtrs; 

void importScanPtrs(const std::string fileName, ScanPtrs &scanPtrs );

#endif


