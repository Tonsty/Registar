#include "loop.h"

#include <iostream>
#include <fstream>
#include <sstream>

void importLoops(const std::string fileName, Loops &loops)
{
	std::fstream file;
	file.open(fileName.c_str());

	char dummy[300];
	int i = 0;
	while( file.getline(dummy, 300) )
	{
		std::stringstream sstr(dummy);
		LinkIndex linkIndex;
		std::cout << "loop " << i << " : ";
		while( sstr >> linkIndex ) 
		{
			std::cout << linkIndex << " ";
		}
		std::cout << std::endl;
		i++;
	}
}
