#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <iostream>

#include "common.h"
#include "link.h"
#include "loop.h"

typedef ScanIndex VertexBase;
struct GraphVertex
{
	VertexBase vbase;
	GraphVertex() {}
	GraphVertex(VertexBase _vbase) : vbase(_vbase) {}
	inline virtual bool isGraphLoop() const { return false;}
};
std::ostream& operator<<(std::ostream& _out, const GraphVertex &_graphVertex);

struct EdgeBase
{
	Transformation transformation;
	float rmsError;
};
struct GraphEdge
{
	GraphVertex *a;
	GraphVertex *b;
	EdgeBase ebase;
	GraphEdge(GraphVertex *_a, GraphVertex *_b) : a(_a), b(_b) {}
};

typedef float LoopBase;
struct GraphLoop: public GraphVertex
{
	std::vector<GraphVertex*> loop;
	LoopBase lbase;
	inline bool contain(GraphVertex* _v)
	{
		for (int i = 0; i < loop.size(); ++i) if (loop[i] == _v) return true;
		return false;
	}
	inline bool intersected(GraphLoop* _other)
	{
		for (int i = 0; i < _other->loop.size(); ++i) if( contain(_other->loop[i]) ) return true;
		return false;
	}
	inline std::vector<GraphLoop*> blend(GraphLoop* _other)
	{
		// std::cout << "blending..." << std::endl;
		GraphLoop temp;
		for (int i = 0; i < loop.size(); ++i) 
			if (!_other->contain(loop[i])) temp.loop.push_back(loop[i]);
			else temp.loop.push_back(static_cast<GraphVertex*>(_other)); //degenerate vertex in _other into the common loop vertex

		// std::cout << static_cast<const GraphVertex &>(temp) << std::endl;
		return temp.split(_other);
	}
	inline std::vector<GraphLoop*> split(GraphVertex* _separator)
	{
		int first_sep_offset = -1;
		for (int i = 0; i < loop.size(); ++i)
		{
			if (loop[i] == _separator)
			{
				first_sep_offset = i;
				break;
			}
		}
		if(first_sep_offset == 0)  // first is sep
		{
			if ( loop.back() == _separator ) 
			{
				int last_first_sep_offset = -1;
				for (int i = 0; i < loop.size(); i++)
				{
					if (loop[ loop.size() - 1 - i] != _separator)
					{
						last_first_sep_offset = i-1;
						break;
					}
				}
				if (last_first_sep_offset >= 0)
				{
					std::vector<GraphVertex*> temp;
					temp.insert(temp.begin(), loop.end() - last_first_sep_offset - 1, loop.end());
					loop.insert(loop.begin(), temp.begin(), temp.end());
					// std::cout << *this << std::endl;
					loop.erase(loop.end() - last_first_sep_offset - 1, loop.end());
					// std::cout << *this << std::endl;
				}
			}
		}
		else if(first_sep_offset > 0)  // first is not sep
		{
			std::vector<GraphVertex*> temp;
			temp.insert(temp.begin(), loop.begin(), loop.begin() + first_sep_offset );
			loop.insert(loop.end(), temp.begin(), temp.end());
			loop.erase(loop.begin(), loop.begin() + first_sep_offset );
		}

		// std::cout << *this << std::endl;

		std::vector< GraphLoop* > newloops;
		GraphLoop *currentLoop = NULL;
		bool meet = false;
		for (int i = 0; i < loop.size(); ++i)
		{
			if(loop[i] == _separator)
			{
				if( meet == false )
				{
					if ( i != 0 ) newloops.push_back(currentLoop);
					currentLoop = new GraphLoop;
					currentLoop->loop.push_back(loop[i]);
					meet = true;					
				}
			}
			else 
			{
				currentLoop->loop.push_back(loop[i]);
				meet = false;
			}
		}
		newloops.push_back(currentLoop);

		return newloops;
	}
	inline virtual bool isGraphLoop() const { return true;}
};
struct GraphLoopComp
{
	bool operator()(GraphLoop* const &a, GraphLoop* const &b)
	{
		return a->lbase < b->lbase;
	}
};

struct Graph
{
	std::vector<GraphVertex*> vertices;
	std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*> edges;
	std::multiset<GraphLoop*, GraphLoopComp> loops;
};

std::ostream& operator<<(std::ostream& _out, const Graph &_graph);


#endif
