#include "link.h"

#include <iostream>
#include <fstream>

namespace tang2014
{
	void importLinks(const std::string fileName, Links &links)
	{
		std::fstream file;
		file.open(fileName.c_str());

		ScanIndex a, b;
		int i = 0;
		while( file >> a >> b ) 
		{
			std::cout << "link " << i << " : " << a << " " << b << std::endl;
			Link link;
			link.a = a;
			link.b = b;
			links.push_back(link);
			i++;
		}
	}	
}
