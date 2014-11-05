#include "graph.h"

namespace Tang2014
{
	std::ostream& operator<<(std::ostream& _out, const GraphVertex &_graphVertex)
	{
		if ( !_graphVertex.isGraphLoop()) _out << _graphVertex.vbase << " ";
		else
		{
			const GraphLoop &graphLoop = static_cast<const GraphLoop&>(_graphVertex);
			_out << "( ";
			for (int i = 0; i < graphLoop.loop.size(); ++i) _out << *(graphLoop.loop[i]);
			_out << ") ";
		}
		return _out;
	}

	std::ostream& operator<<(std::ostream& _out, const GraphEdge &_graphEdge)
	{
		_out << "[ " << *_graphEdge.a << *_graphEdge.b << "]" << std::endl;
		return _out;
	}

	std::ostream& operator<<(std::ostream& _out, const Graph &_graph)
	{
		_out << "vertices : " << std::endl;
		for (int i = 0; i < _graph.vertices.size(); ++i)
		{
			_out << _graph.vertices[i]->vbase << " ";
			if (_graph.vertices[i]->isGraphLoop()) _out << "(loop) "; 
			_out << std::endl;
		} 
		_out << std::endl;

		_out << "edges : " << std::endl;
		for (std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::const_iterator it = _graph.edges.begin(); it != _graph.edges.end(); ++it) 
		{
			_out << (*it).second->a->vbase << " ";
			if ((*it).second->a->isGraphLoop()) _out << "(loop) "; 
			_out << (*it).second->b->vbase << " ";
			if ((*it).second->b->isGraphLoop()) _out << "(loop) "; 
			_out << std::endl;
		}
		_out << std::endl;

		_out << "loops : " << std::endl;
		for (std::multiset<GraphLoop*, GraphLoopComp>::const_iterator it = _graph.loops.begin(); it != _graph.loops.end(); it++) 
		{
			for (int j = 0; j < (*it)->loop.size(); ++j)
			{
				_out << (*it)->loop[j]->vbase << " ";
				if( (*it)->loop[j]->isGraphLoop() ) _out << "(loop) ";
			}
			_out << std::endl;
		}

		return _out;
	}
}