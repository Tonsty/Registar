#ifndef LOOP_H
#define LOOP_H

#include <vector>
#include <string>

typedef unsigned int LinkIndex;
typedef std::vector<LinkIndex> LinkIndices;
struct Loop
{
	LinkIndices  linkIndices;
};
typedef std::vector<Loop> Loops;

void importLoops(const std::string fileName, Loops &loops);

#endif 
