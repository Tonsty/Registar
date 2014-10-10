#include "globalregistration.h"

#include "SRoMCPS.h"

#include "../include/mathutilities.h"

void GlobalRegistration::startRegistration()
{
	buildKdTreePtrs();
	initialTransformations();	
	initialPairRegistration();
	incrementalLoopRefine();
	// globalRefine(10);
}

void GlobalRegistration::buildKdTreePtrs()
{
	for (int i = 0; i < scanPtrs.size(); ++i)
	{
		KdTreePtr kdTreePtr(new KdTree);
		kdTreePtr->setInputCloud(scanPtrs[i]->pointsPtr);
		kdTreePtrs.push_back(kdTreePtr);
	}
}

void GlobalRegistration::initialTransformations()
{
	for (int i = 0; i < scanPtrs.size(); ++i)
	{
		transformations.push_back(Transformation::Identity());
	}
}

void GlobalRegistration::initialPairRegistration()
{
	for (int i = 0; i < links.size(); ++i)
	{
		Link link = links[i];
		ScanIndex a = link.a;
		ScanIndex b = link.b;
		PairRegistrationPtr pairReigstrationPtr(new PairRegistration(scanPtrs[a], scanPtrs[b]));
		pairReigstrationPtr->setKdTree(kdTreePtrs[a], kdTreePtrs[b]);
		pairReigstrationPtr->setParameter(PairRegistration::POINT_TO_PLANE, PairRegistration::UMEYAMA, Transformation::Identity(), true, 10.f, true, 45.0f, true, true, 60);
		pairReigstrationPtr->startRegistration();
		std::pair<Link, PairRegistrationPtr> pairLP(link, pairReigstrationPtr);
		pairRegistrationPtrMap.insert(pairLP);
	}
}

void GlobalRegistration::initialGraph()
{
	std::cout << "start intial graph" << std::endl;

	for (int i = 0; i < scanPtrs.size(); ++i) 
	{
		ScanIndex scanIndex = static_cast<ScanIndex>(i);
		graph.vertices.push_back(new GraphVertex(scanIndex));
	}
	for (int i = 0; i < links.size(); ++i)
	{
		Link link = links[i];
		ScanIndex a = link.a;
		ScanIndex b = link.b;
		GraphEdge *newEdge = createGraphEdge(graph.vertices[a], graph.vertices[b]);
		graph.edges[std::pair<GraphVertex*, GraphVertex*>(newEdge->a, newEdge->b)] = newEdge;
	}
	for (int i = 0; i < loops.size(); ++i)
	{
		GraphLoop *temp(new GraphLoop);
		for (int j = 0; j < loops[i].scanIndices.size(); ++j)
		{
			ScanIndex scanIndex = loops[i].scanIndices[j];
			temp->loop.push_back(graph.vertices[scanIndex]);
		}
		temp->lbase = loopEstimateConsistencyError(temp);   //assign consistency priority
		graph.loops.insert(temp);
	}	
}

GraphEdge* GlobalRegistration::createGraphEdge(GraphVertex *_vertex1, GraphVertex *_vertex2)
{
	std::cout << "new edge : [ " << *_vertex1 << " " << *_vertex2 << " ]"<< std::endl;

	GraphEdge* newEdge = new GraphEdge(_vertex1, _vertex2);

	if (!_vertex1->isGraphLoop() && !_vertex2->isGraphLoop())
	{
		Link link;
		link.a = _vertex1->vbase;
		link.b = _vertex2->vbase;
		newEdge->ebase.transformation = pairRegistrationPtrMap[link]->transformation; //assign initial pair registration result

		std::cout << "new base edge transformation : \n" << newEdge->ebase.transformation << std::endl;

		return newEdge;
	}

	std::vector<GraphVertex*> vertices1, vertices2;
	Transformations transformations1, transformations2;

	std::cout << "Decompose Vertex : " << *_vertex1 << std::endl;
	GraphVertexDecompose(_vertex1, Transformation::Identity(), NULL, vertices1, transformations1, false);
	std::cout << "Decompose Vertex : " << *_vertex2 << std::endl;
	GraphVertexDecompose(_vertex2, Transformation::Identity(), NULL, vertices2, transformations2, false);

	PairRegistration::PointPairs all_final_s2t;

	for (int i = 0; i < vertices1.size(); ++i)
	{
		for (int j = 0; j < vertices2.size(); ++j)
		{
			if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
			{
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
					graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( vertices1[i], vertices2[j] ) );
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
					graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( vertices2[j], vertices1[i]) );

				if ( it_12 != graph.edges.end() )
				{
					Link link;
					link.a = (*it_12).second->a->vbase;
					link.b = (*it_12).second->b->vbase;

					std::cout << "find link [ " << link.a << " " << link.b << " ]" << std::endl;
					// std::cout << "link transformation :\n"  << 	pairRegistrationPtrMap[link]->transformation << std::endl;			

					for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
					{
						PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
						temp.targetPoint = transformPointWithNormal( temp.targetPoint, transformations1[i] );   //direct mate not the virtual mate, it should also be enough since we continue update 
						temp.sourcePoint = transformPointWithNormal( temp.sourcePoint, transformations2[j] );   //the transformation using pair registration
						all_final_s2t.push_back(temp);
					}
				}
				else if ( it_21 != graph.edges.end() )  // edge in reverse order
				{
					Link link;
					link.a = (*it_21).second->a->vbase;
					link.b = (*it_21).second->b->vbase;

					std::cout << "find link reverse [ " << link.a << " " << link.b << " ]" << std::endl;
					// std::cout << "link transformation :\n"  << 	pairRegistrationPtrMap[link]->transformation << std::endl;			

					for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
					{
						PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
						PairRegistration::PointPair temp2;
						temp2.targetPoint = temp.sourcePoint;
						temp2.sourcePoint = temp.targetPoint;
						temp2.targetPoint = transformPointWithNormal( temp2.targetPoint, transformations1[i] ); //direct mate not the virtual mate, it should also be enough since we continue update 
						temp2.sourcePoint = transformPointWithNormal( temp2.sourcePoint, transformations2[j] ); //the transformation using pair registration
						all_final_s2t.push_back(temp2);
						// if( link.a == 2 && link.b == 0)
						// {
						// 	all_final_s2t.push_back(temp2);
						// 	if ( k < 10)
						// 	{
						// 		std::cout << "k = " << k << " : x = " << temp.sourcePoint.x << " y = " << temp.sourcePoint.y << " z = " << temp.sourcePoint.z << std::endl;
						// 		std::cout << "k = " << k << " : x = " << temp.targetPoint.x << " y = " << temp.targetPoint.y << " z = " << temp.targetPoint.z << std::endl;
						// 		std::cout << "k = " << k << " : x = " << temp2.sourcePoint.x << " y = " << temp2.sourcePoint.y << " z = " << temp2.sourcePoint.z << std::endl;	
						// 		std::cout << "k = " << k << " : x = " << temp2.targetPoint.x << " y = " << temp2.targetPoint.y << " z = " << temp2.targetPoint.z << std::endl;									
						// 	}
						// }
					}					
				}
			}
		}
	}

	// initial pair registration
	Transformation newTransformation = PairRegistration::solveRegistration(all_final_s2t, PairRegistration::UMEYAMA);

	// std::cout << "initial newTransformation : \n" <<newTransformation << std::endl;

	// keep sub-edges between vertices1 and vertices2 consistent with the root edge between _vertex1 and _vertex2, and also update the pair registration transformation
	for (int i = 0; i < vertices1.size(); ++i)
	{
		for (int j = 0; j < vertices2.size(); ++j)
		{
			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
				graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices1[i], vertices2[j] ) );
			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
				graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices2[j], vertices1[i]) );

			if ( it_12 != graph.edges.end() )
			{
				std::cout << "find edge [ " << *vertices1[i] << " " << *vertices2[j] << " ]" << std::endl;	
				(*it_12).second->ebase.transformation = transformations1[i].inverse() * newTransformation * transformations2[j];

				// std::cout << "transformations1  : \n" << transformations1[i] << std::endl;
				// std::cout << "newTransformation : \n" << newTransformation << std::endl;
				// std::cout << "transformations2 : \n" << transformations2[j] << std::endl;
				//std::cout << "update ebase 12 transformation \n" << (*it_12).second->ebase.transformation << std::endl;


				if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
				{
					Link link;
					link.a = (*it_12).second->a->vbase;
					link.b = (*it_12).second->b->vbase;

					//std::cout << "difference : \n" << pairRegistrationPtrMap[link]->transformation - (*it_12).second->ebase.transformation << std::endl; 
					pairRegistrationPtrMap[link]->transformation = (*it_12).second->ebase.transformation;				
				}	
			}
			else if ( it_21 != graph.edges.end() )  // edge in reverse order
			{
				std::cout << "find reverse edge [ " << *vertices2[j] << " " << *vertices1[i] << " ]" << std::endl;
				(*it_21).second->ebase.transformation = transformations2[j].inverse() * newTransformation.inverse() * transformations1[i];

				// std::cout << "transformations2 : \n" << transformations2[j] << std::endl;
				// std::cout << "newTransformation : \n" << newTransformation << std::endl;
				// std::cout << "transformations1 : \n" << transformations1[j] << std::endl;	
				//std::cout << "update ebase 21 transformation \n" << (*it_21).second->ebase.transformation << std::endl;

				if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
				{
					Link link;
					link.a = (*it_21).second->a->vbase;
					link.b = (*it_21).second->b->vbase;
					// std::cout << "difference : \n" << pairRegistrationPtrMap[link]->transformation - (*it_12).second->ebase.transformation << std::endl;
					pairRegistrationPtrMap[link]->transformation = (*it_21).second->ebase.transformation;				
				}				
			}
		}
	}

	for (int iter = 0; iter < 3; ++iter)
	{
		// then update newTransformation by direct pair registration
		all_final_s2t.clear();

		for (int i = 0; i < vertices1.size(); ++i)
		{
			for (int j = 0; j < vertices2.size(); ++j)
			{
				if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
				{
					std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
						graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices1[i], vertices2[j] ) );
					std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
						graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices2[j], vertices1[i]) );	

					if ( it_12 != graph.edges.end() )
					{
						Link link;
						link.a = (*it_12).second->a->vbase;
						link.b = (*it_12).second->b->vbase;
						// std::cout << "a = " << link.a << " b = " << link.b << std::endl;
						// std::cout << pairRegistrationPtrMap[link]->transformation << std::endl;
						pairRegistrationPtrMap[link]->generateFinalPointPairs( pairRegistrationPtrMap[link]->transformation );
						for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
						{
							PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
							temp.targetPoint = transformPointWithNormal( temp.targetPoint, transformations1[i] );  
							temp.sourcePoint = transformPointWithNormal( temp.sourcePoint, transformations2[j] );  
							all_final_s2t.push_back(temp);
						}
					}
					else if ( it_21 != graph.edges.end() )  // edge in reverse order
					{
						Link link;
						link.a = (*it_21).second->a->vbase;
						link.b = (*it_21).second->b->vbase;
						// std::cout << "a = " << link.a << " b = " << link.b << std::endl;
						// std::cout << pairRegistrationPtrMap[link]->transformation << std::endl;
						pairRegistrationPtrMap[link]->generateFinalPointPairs( pairRegistrationPtrMap[link]->transformation );
						for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
						{
							PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
							PairRegistration::PointPair temp2;
							temp2.targetPoint = temp.sourcePoint;
							temp2.sourcePoint = temp.targetPoint;
							temp2.targetPoint = transformPointWithNormal( temp2.targetPoint, transformations1[i] ); //direct mate not the virtual mate, it should also be enough since we continue update 
							temp2.sourcePoint = transformPointWithNormal( temp2.sourcePoint, transformations2[j] ); //the transformation using pair registration
							all_final_s2t.push_back(temp2);
						}
					}				
				}
			}
		}

		newTransformation = PairRegistration::solveRegistration(all_final_s2t, PairRegistration::UMEYAMA);

		// keep sub-edges between vertices1 and vertices2 consistent with the root edge between _vertex1 and _vertex2, and also update the pair registration transformation
		for (int i = 0; i < vertices1.size(); ++i)
		{
			for (int j = 0; j < vertices2.size(); ++j)
			{
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
					graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices1[i], vertices2[j] ) );
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
					graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertices2[j], vertices1[i]) );

				if ( it_12 != graph.edges.end() )
				{
					// std::cout << "find edge [ " << *vertices1[i] << " " << *vertices2[j] << " ]" << std::endl;	
					(*it_12).second->ebase.transformation = transformations1[i].inverse() * newTransformation * transformations2[j];
					if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
					{
						Link link;
						link.a = (*it_12).second->a->vbase;
						link.b = (*it_12).second->b->vbase;
						pairRegistrationPtrMap[link]->transformation = (*it_12).second->ebase.transformation;				
					}	
				}
				else if ( it_21 != graph.edges.end() )  // edge in reverse order
				{
					// std::cout << "find edge [ " << *vertices2[j] << " " << *vertices1[i] << " ]" << std::endl;
					(*it_21).second->ebase.transformation = transformations2[j].inverse() * newTransformation.inverse() * transformations1[i];
					if ( !vertices1[i]->isGraphLoop() && !vertices2[j]->isGraphLoop() )
					{
						Link link;
						link.a = (*it_21).second->a->vbase;
						link.b = (*it_21).second->b->vbase;
						pairRegistrationPtrMap[link]->transformation = (*it_21).second->ebase.transformation;				
					}				
				}
			}
		}
	}

	// std::cout << "result newTransformation : \n" <<newTransformation << std::endl;

	newEdge->ebase.transformation = newTransformation;
	return newEdge;
}

Transformation GlobalRegistration::GraphVertexDecompose(GraphVertex* currentVertex, Transformation lastTransformation, GraphVertex* lastVertex, 
	std::vector<GraphVertex*> &resultVertices, Transformations &resultTransformations, bool baseVertexOnly)
{

	// std::cout << "Vertex: " << *currentVertex << " " << std::endl;

	Transformation currentTransformation;
	if (lastVertex != NULL)
	{
		std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_cl = 
			graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( currentVertex, lastVertex ) );
		std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_lc = 
			graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( lastVertex, currentVertex ) );

		if (it_cl != graph.edges.end()) 
		{
			currentTransformation = lastTransformation * (*it_cl).second->ebase.transformation.inverse();
			// std::cout << "lastTransformation : \n" << lastTransformation << std::endl;
			// std::cout << "it_cl inverse : \n" << (*it_cl).second->ebase.transformation.inverse() << std::endl;
		} 
		else if (it_lc != graph.edges.end())
		{
			currentTransformation = lastTransformation * (*it_lc).second->ebase.transformation;
			// std::cout << "lastTransformation : \n" << lastTransformation << std::endl;
			// std::cout << "it_lc : \n" << (*it_lc).second->ebase.transformation << std::endl;
		} 
		else 
		{
			std::cerr << "error : edge lost in loop"<< std::endl;
			exit(1);
		}
	}
	else
	{
		currentTransformation = lastTransformation;
	}

	// std::cout << "currentTransformation : \n" << currentTransformation << std::endl;

	if (!currentVertex->isGraphLoop())
	{
		resultVertices.push_back(currentVertex);
		resultTransformations.push_back( currentTransformation );

		return currentTransformation;
	}
	else
	{
		if(!baseVertexOnly) 
		{
			resultVertices.push_back(currentVertex);
			resultTransformations.push_back( currentTransformation );
		}
		lastVertex = NULL;
		lastTransformation = currentTransformation;
		GraphLoop* currentLoop = static_cast<GraphLoop*>(currentVertex);
		for (int i = 0; i < currentLoop->loop.size(); ++i)
		{
			lastTransformation = GraphVertexDecompose( currentLoop->loop[i], lastTransformation, lastVertex, resultVertices, resultTransformations, baseVertexOnly);
			lastVertex = currentLoop->loop[i];
		}

		return currentTransformation;
	}
}

float GlobalRegistration::loopEstimateConsistencyError( GraphLoop* _graphLoop )
{
	return 0.0f;
}

void GlobalRegistration::loopRefine( GraphLoop* _graphLoop )
{

}

void GlobalRegistration::incrementalLoopRefine()
{
	initialGraph();
	std::cout << graph << std::endl;

	int iter = 0;

	GraphLoop* finalLoop;

	while(!graph.loops.empty())
	{
		std::cout << "iter " << iter++ << " : " << std::endl;
		for (std::set<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++) std::cout << *(*it) << " ";
		std::cout << std::endl;

		std::vector< GraphLoop* > wait_insert;
		
		GraphLoop* currentLoop = *(graph.loops.begin());
		finalLoop = currentLoop;

		//do loop refine to currentLoop  ... ...
		loopRefine(currentLoop);

		//generate new loops to wait_insert
		for (std::set<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++) 
		{
			if ( it != graph.loops.begin() && (*it)->intersected(currentLoop) )
			{
				std::vector<GraphLoop*> newloops = (*it)->blend(currentLoop);
				std::cout << "new loop(s) : ";
				for (int i = 0; i < newloops.size(); ++i) std::cout << *newloops[i] << " "; 
				std::cout << std::endl;
				wait_insert.insert(wait_insert.end(), newloops.begin(), newloops.end());
			}
		}

		//keep those unchanged old loops
		std::multiset<GraphLoop*, GraphLoopComp> temp;
		for (std::set<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++)
		{
			if ( it != graph.loops.begin() && !(*it)->intersected(currentLoop) )
			{
				temp.insert(*it);
			}
		}
		graph.loops = temp;

		//delete reduplicate loop and separate edge loops from triangle or polygon loops
		std::vector< GraphLoop* > wait_insert_2, wait_insert_el3;
		for (int i = 0; i < wait_insert.size(); ++i) 
		{
			if ( wait_insert[i]->loop.size() == 2 )
			{
				bool exist = false;
				for (int j = 0; j < wait_insert_2.size(); ++j) if (wait_insert[i]->equal(wait_insert_2[j])) { exist = true; break; } 
				if(!exist) wait_insert_2.push_back(wait_insert[i]);
			}
		}
		for (int i = 0; i < wait_insert.size(); ++i) 
		{
			if ( wait_insert[i]->loop.size() > 2 )
			{
				bool exist = false;
				for (int j = 0; j < wait_insert_el3.size(); ++j) if (wait_insert[i]->equal(wait_insert_el3[j])) { exist = true; break; } 
				if(!exist) wait_insert_el3.push_back(wait_insert[i]);
			}
		}

		//delete edge loop that already belongs to any traingle or polygon loop
		std::vector< GraphLoop* > wait_insert_2_independent;
		for (int i = 0; i < wait_insert_2.size(); ++i)
		{
			bool belong = false;
			for (int j = 0; j < wait_insert_el3.size(); ++j)
			{
				if( wait_insert_2[i]->loop.back() == wait_insert_el3[j]->loop.back() ||
					wait_insert_2[i]->loop.back() == *( wait_insert_el3[j]->loop.begin() + 1 ) ) { belong = true; break; }
			}
			if(!belong) wait_insert_2_independent.push_back(wait_insert_2[i]);
		}

		//add new edge and re-estimate new loop
		//  edge loop case
		for (int i = 0; i < wait_insert_2_independent.size(); ++i)
		{
			GraphVertex* loopVertex = wait_insert_2_independent[i]->loop.front();
			GraphVertex* companionVertex = wait_insert_2_independent[i]->loop.back();

			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it = 
				graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( loopVertex, companionVertex ) );
			if ( it == graph.edges.end() ) 	// add new edge
			{
				GraphEdge* newEdge = createGraphEdge(loopVertex, companionVertex);
				graph.edges[std::pair<GraphVertex*, GraphVertex*>(newEdge->a, newEdge->b)] = newEdge;
			}
			wait_insert_2_independent[i]->lbase = 0.0f; //edge loop are always consistent  
		}
		//  triangle or polygon case
		for (int i = 0; i < wait_insert_el3.size(); ++i)
		{
			GraphVertex* loopVertex = wait_insert_el3[i]->loop.front();
			GraphVertex* companionVertex1 = *( wait_insert_el3[i]->loop.begin() + 1 );			
			GraphVertex* companionVertex2 = wait_insert_el3[i]->loop.back();

			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it1 = 
				graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( loopVertex, companionVertex1 ) );
			if ( it1 == graph.edges.end() ) // add new edge
			{
				// assign new pair registration result to new edge if one to many 
				GraphEdge* newEdge = createGraphEdge(loopVertex, companionVertex1);
				graph.edges[std::pair<GraphVertex*, GraphVertex*>(newEdge->a, newEdge->b)] = newEdge;				
			}

			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it2 = 
				graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( loopVertex, companionVertex2 ) );
			if ( it2 == graph.edges.end() ) // add new edge
			{
				// assign new pair registration result to new edge if one to many
				GraphEdge* newEdge = createGraphEdge(loopVertex, companionVertex2);
				graph.edges[std::pair<GraphVertex*, GraphVertex*>(newEdge->a, newEdge->b)] = newEdge;				
			}

			wait_insert_el3[i]->lbase = loopEstimateConsistencyError(wait_insert_el3[i]); //re-estimate	loop consistency error
		}		

		//insert the left new loops
		for (int i = 0; i < wait_insert_2_independent.size(); ++i) graph.loops.insert(wait_insert_2_independent[i]);	
		for (int i = 0; i < wait_insert_el3.size(); ++i) graph.loops.insert(wait_insert_el3[i]);
	}

	std::cout << "Final loop : " << *finalLoop << std::endl;

	std::cout << "Decompose final loop : " << *finalLoop << std::endl;
	std::vector<GraphVertex*> final_vertices;
	Transformations final_transformations;
	GraphVertexDecompose(finalLoop, Transformation::Identity(), NULL, final_vertices, final_transformations, true);
	for (int i = 0; i < final_vertices.size(); ++i)
	{
		std::cout << "set transformation to " << final_vertices[i]->vbase << std::endl;
		std::cout << final_transformations[i] << std::endl;
		transformations[ final_vertices[i]->vbase ] = final_transformations[i];
	}

}

void GlobalRegistration::globalPairRefine()
{

}

void GlobalRegistration::globalRefine(unsigned int _interationNum)
{
	PointsPtr buffer(new Points);
	PairRegistration::PointPairs s2t, t2s;

	sromcps::ScanIndexPairs sipairs;
	std::vector<sromcps::PointPairWithWeights> ppairwwss;
	int M = scanPtrs.size();

	sromcps::PointPairWithWeights buffer1;

	float last_rms_error = std::numeric_limits<float>::max();

	for (int iter = 0; iter < _interationNum; ++iter)
	{
		sipairs.clear();
		ppairwwss.clear();
		for (int i = 0; i < links.size(); ++i)
		{
			Link link = links[i];
			ScanIndex a = link.a;
			ScanIndex b = link.b;

			sipairs.push_back(sromcps::ScanIndexPair(a,b));

			buffer->clear();
			s2t.clear();
			t2s.clear();
			Transformation transformation = transformations[a].inverse() * transformations[b];
			PairRegistrationPtr pairRegistrationPtr = pairRegistrationPtrMap[link];
			pairRegistrationPtr->generatePointPairs(buffer, transformation, s2t, t2s);

			for (int j = 0; j < s2t.size(); ++j) s2t[j].sourcePoint = transformPointWithNormal(s2t[j].sourcePoint, transformation.inverse());
			for (int j = 0; j < t2s.size(); ++j) t2s[j].sourcePoint = transformPointWithNormal(t2s[j].sourcePoint, transformation);

			buffer1.clear();
			for (int j = 0; j < s2t.size(); ++j)
			{
				sromcps::PointPairWithWeight temp;
				temp.w = 1.0;
				temp.ppair.first = s2t[j].targetPoint.getVector3fMap().cast<sromcps::Scalar>(); 
				temp.ppair.second = s2t[j].sourcePoint.getVector3fMap().cast<sromcps::Scalar>();
				buffer1.push_back(temp);
			}
			for (int j = 0; j < t2s.size(); ++j)
			{
				sromcps::PointPairWithWeight temp;
				temp.w = 1.0;
				temp.ppair.first = t2s[j].sourcePoint.getVector3fMap().cast<sromcps::Scalar>(); 
				temp.ppair.second = t2s[j].targetPoint.getVector3fMap().cast<sromcps::Scalar>();
				buffer1.push_back(temp);
			}		
			ppairwwss.push_back(buffer1);
		}

		sromcps::SRoMCPS sromcps_globalrefine(sipairs, ppairwwss, M);

		float total_error = ( sromcps_globalrefine.R *  sromcps_globalrefine.Q * sromcps_globalrefine.R.transpose() ).trace();
		float total_weight = 0.0f;
		for (int i = 0; i < ppairwwss.size(); ++i)
		{
			for (int j = 0; j < ppairwwss[i].size(); ++j)
			{
				total_weight += ppairwwss[i][j].w;
			}
		}
		float rms_error = sqrtf( total_error / total_weight );
		std::cout << "globalrefine rms_error = " <<  rms_error << " total_weight = " << total_weight << std::endl;

		if ( last_rms_error < rms_error )
		{
			std::cout << "globalrefine converged after " << iter << " iteration(s)" << std::endl;
			break;
		}
		last_rms_error = rms_error;

		for (int i = 0; i < M; ++i)
		{
			transformations[i].block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*i).cast<float>();
			transformations[i].block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*i, 0).cast<float>();			 
		}
	}
}

	// Transformation resultTransformation, resultTransformation0, resultTransformation1;
	// resultTransformation0 = Transformation::Identity();
	// resultTransformation0.block<3,3>(0,0) = sromcps_globalrefine.R.block<3,3>(0,0).cast<float>();
	// resultTransformation0.block<3,1>(0,3) = sromcps_globalrefine.T.block<3,1>(0,0).cast<float>();
	// resultTransformation1 = Transformation::Identity();
	// resultTransformation1.block<3,3>(0,0) = sromcps_globalrefine.R.block<3,3>(0,3).cast<float>();
	// resultTransformation1.block<3,1>(0,3) = sromcps_globalrefine.T.block<3,1>(3,0).cast<float>();

	// resultTransformation = resultTransformation1 * resultTransformation0.inverse();
	// std::cout << resultTransformation << std::endl;

		// for (int i = 0; i < wait_insert.size(); ++i)
		// {
		// 	if ( wait_insert[i]->loop.size() > 2 ) //polygon or triangle loop case, add two connected new edge and the loop
		// 	{
		// 		std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it1 = 
		// 			graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop.back() ) );
		// 		if ( it1 == graph.edges.end() )
		// 		{
		// 			// assign new pair registration result to new edge if one to many 
		// 			// add new edge
		// 		}

		// 		std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it2 = 
		// 			graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop[1] ) );
		// 		if ( it2 != graph.edges.end() )
		// 		{
		// 			// assign new pair registration result to new edge if one to many
		// 			// add new edge
		// 		}

		// 		wait_insert[i]->lbase = loopEstimateConsistencyError(wait_insert[i]);  //assign consistency priority

		// 		graph.loops.insert(wait_insert[i]);
		// 	}
		// 	else if( wait_insert[i]->loop.size() == 2 ) //single edge loop case, add single new edge that doesn't belong to any other polygon or triangle loop 
		// 	{
		// 		std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it = 
		// 			graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop.back() ) );
		// 		if ( it == graph.edges.end() )
		// 		{
		// 			// assign new pair registration result to new edge if one to many
		// 			// add new edge
		// 		}
		// 		graph.loops.insert(wait_insert[i]);		//add if doesn't belong to any other polygon or triangle loop				
		// 	}
		// 	//else internal loop

		// }

