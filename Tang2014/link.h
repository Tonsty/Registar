#ifndef LINK_H
#define LINK_H

#include <vector>
#include <string>

namespace tang2014
{
	typedef unsigned int ScanIndex;
	struct Link
	{
		ScanIndex a;
		ScanIndex b;
	};
	typedef std::vector<Link> Links;
	struct LinkComp
	{
		inline bool operator()(const Link &linkA, const Link &linkB )
		{
			return ( linkA.a != linkB.a ) ? ( linkA.a < linkB.a ) : ( linkA.b < linkB.b );
		}
	};

	void importLinks(const std::string fileName, Links &links);
}

#endif