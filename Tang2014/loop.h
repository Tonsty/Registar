#ifndef LOOP_H
#define LOOP_H

#include <vector>
#include <string>

#include "link.h"

namespace Tang2014
{
	typedef std::vector<ScanIndex> ScanIndices;
	struct Loop
	{
		ScanIndices  scanIndices;
	};
	typedef std::vector<Loop> Loops;

	void importLoops(const std::string fileName, Loops &loops);
}

#endif 
