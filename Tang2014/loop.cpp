#include "loop.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace Tang2014
{
	void importLoops(const std::string fileName, Loops &loops)
	{
		std::fstream file;
		file.open(fileName.c_str());

		char dummy[300];
		int i = 0;
		while( file.getline(dummy, 300) )
		{
			std::stringstream sstr(dummy);
			ScanIndex scanIndex;
			std::cout << "loop " << i << " : ";
			Loop temp;
			while( sstr >> scanIndex ) 
			{
				std::cout << scanIndex << " ";
				temp.scanIndices.push_back(scanIndex);
			}
			std::cout << std::endl;
			loops.push_back(temp);
			i++;
		}
	}	
}

