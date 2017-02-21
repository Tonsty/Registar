#ifndef _OPENMP
#define _OPENMP
#endif

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>

#include "pairregistration.h"
#include "globalregistration.h"

#include "../Williams2001/SRoMCPS.h"
#include "../include/utilities.h"

namespace tang2014
{
	void GlobalRegistration::startRegistration()
	{
		pcl::ScopeTime time("calculation");
		buildKdTreePtrs();
		std::cout << "time after building kdtrees : "<< time.getTimeSeconds() << std::endl;
		initialTransformations();
		initialPairRegistration();
		std::cout << "time after initial pair registration : " << time.getTimeSeconds() << std::endl;
		if(para.doIncrementalLoopRefine) incrementalLoopRefine();
		std::cout << "time after incremental loop refinement : " << time.getTimeSeconds() << std::endl;
		if(para.doGlobalRefine && para.doInitialPairRegistration) globalPairRefine();
		else if(para.doGlobalRefine) globalRefine(para.globalIterationNum_max, para.globalIterationNum_min);
		std::cout << "time after global refinement : " << time.getTimeSeconds() << std::endl;
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
		int threads = omp_get_num_procs();
		std::cout << threads << "threads" << std::endl;
		for (int i = 0; i < links.size(); ++i)
		{
			Link link = links[i];
			ScanIndex a = link.a;
			ScanIndex b = link.b;
			// PairRegistrationPtr pairReigstrationPtr(new PairRegistration(scanPtrs[a], scanPtrs[b]));
			PairRegistrationOMPPtr pairReigstrationPtr(new PairRegistrationOMP(scanPtrs[a], scanPtrs[b], threads));
			pairReigstrationPtr->setKdTree(kdTreePtrs[a], kdTreePtrs[b]);
			pairReigstrationPtr->setParameter(para.pr_para);
			pairReigstrationPtr->setTransformation(Transformation::Identity());
			pairReigstrationPtr->initiateCandidateIndices();
			if(para.doInitialPairRegistration) 
			{
				std::cout << "pair registration : " << link.a << " <<-- " << link.b << std::endl;
				// pairReigstrationPtr->startRegistration();
				pairReigstrationPtr->startRegistrationOMP();
			}
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

		generateFinalPointPairs(vertices1, transformations1, vertices2, transformations2, false, true, all_final_s2t);

		// initial pair registration
		Transformation newTransformation = PairRegistration::solveRegistration(all_final_s2t, PairRegistration::UMEYAMA);

		std::cout << "initial newTransformation : \n" <<newTransformation << std::endl;

		// keep sub-edges between vertices1 and vertices2 consistent with the root edge between _vertex1 and _vertex2, and also update the pair registration transformation
		makeEdgesConsistent(vertices1, transformations1, vertices2, transformations2, newTransformation);

		for (int iter = 0; iter < para.pairIterationNum; ++iter)
		{
			// then update newTransformation by direct pair registration
			all_final_s2t.clear();
			generateFinalPointPairs(vertices1, transformations1, vertices2, transformations2, true, false, all_final_s2t);

			newTransformation = PairRegistration::solveRegistration(all_final_s2t, PairRegistration::UMEYAMA);

			// keep sub-edges between vertices1 and vertices2 consistent with the root edge between _vertex1 and _vertex2, and also update the pair registration transformation
			makeEdgesConsistent(vertices1, transformations1, vertices2, transformations2, newTransformation);

		}

		// std::cout << "result newTransformation : \n" <<newTransformation << std::endl;

		newEdge->ebase.transformation = newTransformation;
		return newEdge;
	}

	Transformation GlobalRegistration::GraphVertexDecompose(GraphVertex* currentVertex, const Transformation &lastTransformation, GraphVertex* lastVertex, 
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
			Transformation lastTransformation = currentTransformation;
			GraphLoop* currentLoop = static_cast<GraphLoop*>(currentVertex);
			for (int i = 0; i < currentLoop->loop.size(); ++i)
			{
				lastTransformation = GraphVertexDecompose( currentLoop->loop[i], lastTransformation, lastVertex, resultVertices, resultTransformations, baseVertexOnly);
				lastVertex = currentLoop->loop[i];
			}

			return currentTransformation;
		}
	}

	void GlobalRegistration::generateFinalPointPairs(std::vector<GraphVertex*> &_vertices1, Transformations &_transformations1, 
		std::vector<GraphVertex*> &_vertices2, Transformations &_transformations2, 
		bool _rePairGenerate, bool useVirtualMate, PairRegistration::PointPairs &_all_final_s2t)
	{
		for (int i = 0; i < _vertices1.size(); ++i)
		{
			for (int j = 0; j < _vertices2.size(); ++j)
			{
				if ( !_vertices1[i]->isGraphLoop() && !_vertices2[j]->isGraphLoop() )
				{
					std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
						graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( _vertices1[i], _vertices2[j] ) );
					std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
						graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( _vertices2[j], _vertices1[i]) );

					if ( it_12 != graph.edges.end() )
					{
						Link link;
						link.a = (*it_12).second->a->vbase;
						link.b = (*it_12).second->b->vbase;

						// std::cout << "find link [ " << link.a << " " << link.b << " ]" << std::endl;
						// std::cout << "link transformation :\n"  << 	pairRegistrationPtrMap[link]->transformation << std::endl;			

						if(_rePairGenerate) pairRegistrationPtrMap[link]->generateFinalPointPairs( pairRegistrationPtrMap[link]->transformation );

						for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
						{
							if( useVirtualMate )
							{
								PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
								Transformation transformation = pairRegistrationPtrMap[link]->transformation;
								PairRegistration::PointPair temp1, temp2;
								temp1.targetPoint = registar::transformPointWithNormal( temp.sourcePoint, _transformations1[i] * transformation );
								temp1.sourcePoint = registar::transformPointWithNormal( temp.sourcePoint, _transformations2[j] );
								temp2.targetPoint = registar::transformPointWithNormal( temp.targetPoint, _transformations1[i] );
								temp2.sourcePoint = registar::transformPointWithNormal( temp.targetPoint, _transformations2[j] * transformation.inverse() );
								_all_final_s2t.push_back(temp1);
								_all_final_s2t.push_back(temp2);							
							}
							else
							{
								PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
								temp.targetPoint = registar::transformPointWithNormal( temp.targetPoint, _transformations1[i] );   //direct mate not the virtual mate, it should also be enough since we continue update 
								temp.sourcePoint = registar::transformPointWithNormal( temp.sourcePoint, _transformations2[j] );   //the transformation using pair registration
								_all_final_s2t.push_back(temp);
							}
						}
					}
					else if ( it_21 != graph.edges.end() )  // edge in reverse order
					{
						Link link;
						link.a = (*it_21).second->a->vbase;
						link.b = (*it_21).second->b->vbase;

						// std::cout << "find link reverse [ " << link.a << " " << link.b << " ]" << std::endl;
						// std::cout << "link transformation :\n"  << 	pairRegistrationPtrMap[link]->transformation << std::endl;	

						if(_rePairGenerate) pairRegistrationPtrMap[link]->generateFinalPointPairs( pairRegistrationPtrMap[link]->transformation );

						for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
						{
							if ( useVirtualMate )
							{
								PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];					
								PairRegistration::PointPair temp2;
								temp2.targetPoint = temp.sourcePoint;
								temp2.sourcePoint = temp.targetPoint;
								Transformation transformation = pairRegistrationPtrMap[link]->transformation;
								Transformation transformation2 = transformation.inverse();	
								PairRegistration::PointPair temp2_1, temp2_2;
								temp2_1.targetPoint = registar::transformPointWithNormal( temp2.sourcePoint, _transformations1[i] * transformation2 );
								temp2_1.sourcePoint = registar::transformPointWithNormal( temp2.sourcePoint, _transformations2[j] );
								temp2_2.targetPoint = registar::transformPointWithNormal( temp2.targetPoint, _transformations1[i] );
								temp2_2.sourcePoint = registar::transformPointWithNormal( temp2.targetPoint, _transformations2[j] * transformation2.inverse() );
								_all_final_s2t.push_back(temp2_1);
								_all_final_s2t.push_back(temp2_2);
							}
							else
							{
								PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
								PairRegistration::PointPair temp2;
								temp2.targetPoint = temp.sourcePoint;
								temp2.sourcePoint = temp.targetPoint;
								temp2.targetPoint = registar::transformPointWithNormal( temp2.targetPoint, _transformations1[i] ); //direct mate not the virtual mate, it should also be enough since we continue update 
								temp2.sourcePoint = registar::transformPointWithNormal( temp2.sourcePoint, _transformations2[j] ); //the transformation using pair registration
								_all_final_s2t.push_back(temp2);
							}
						}					
					}
				}
			}
		}
	}

	void GlobalRegistration::makeEdgesConsistent(std::vector<GraphVertex*> &_vertices1, Transformations &_transformations1, 
		std::vector<GraphVertex*> &_vertices2, Transformations &_transformations2,
		Transformation &_newTransformation)
	{
		for (int i = 0; i < _vertices1.size(); ++i)
		{
			for (int j = 0; j < _vertices2.size(); ++j)
			{
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = 
					graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( _vertices1[i], _vertices2[j] ) );
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = 
					graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( _vertices2[j], _vertices1[i]) );

				if ( it_12 != graph.edges.end() )
				{
					// std::cout << "find edge [ " << *_vertices1[i] << " " << *_vertices2[j] << " ]" << std::endl;	
					(*it_12).second->ebase.transformation = _transformations1[i].inverse() * _newTransformation * _transformations2[j];

					// std::cout << "transformations1  : \n" << transformations1[i] << std::endl;
					// std::cout << "newTransformation : \n" << newTransformation << std::endl;
					// std::cout << "transformations2 : \n" << transformations2[j] << std::endl;
					//std::cout << "update ebase 12 transformation \n" << (*it_12).second->ebase.transformation << std::endl;

					if ( !_vertices1[i]->isGraphLoop() && !_vertices2[j]->isGraphLoop() )
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
					// std::cout << "find reverse edge [ " << *_vertices2[j] << " " << *_vertices1[i] << " ]" << std::endl;
					(*it_21).second->ebase.transformation = _transformations2[j].inverse() * _newTransformation.inverse() * _transformations1[i];

					// std::cout << "transformations2 : \n" << transformations2[j] << std::endl;
					// std::cout << "newTransformation : \n" << newTransformation << std::endl;
					// std::cout << "transformations1 : \n" << transformations1[j] << std::endl;	
					//std::cout << "update ebase 21 transformation \n" << (*it_21).second->ebase.transformation << std::endl;

					if ( !_vertices1[i]->isGraphLoop() && !_vertices2[j]->isGraphLoop() )
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
	}

	float GlobalRegistration::loopEstimateConsistencyError( GraphLoop* _graphLoop )
	{
		return loopRefine(_graphLoop, false);
	}

	float GlobalRegistration::loopRefine( GraphLoop* _graphLoop, bool _closing )
	{
		if (_graphLoop->loop.size() <= 2) return std::numeric_limits<float>::max();

		williams2001::ScanIndexPairs sipairs;
		std::vector<williams2001::PointPairWithWeights> ppairwwss;
		int M = _graphLoop->loop.size();

		williams2001::PointPairWithWeights buffer1;

		std::cout << "refine loop : " << *_graphLoop << std::endl;
		for (int i = 0; i < M; ++i)
		{
			GraphVertex* vertex1 = _graphLoop->loop[i];
			GraphVertex* vertex2 = _graphLoop->loop[(i+1)%M];
			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertex1, vertex2 ) );
			std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertex2, vertex1 ) );

			if (it_12 != graph.edges.end() ) 
			{
				// std::cout << "refine edge : " << *(*it_12).second << std::endl;
				std::vector<GraphVertex*> vertices1, vertices2;
				Transformations transformations1, transformations2;

				GraphVertexDecompose(vertex1, Transformation::Identity(), NULL, vertices1, transformations1, false);
				GraphVertexDecompose(vertex2, Transformation::Identity(), NULL, vertices2, transformations2, false);

				PairRegistration::PointPairs all_final_s2t;

				generateFinalPointPairs(vertices1, transformations1, vertices2, transformations2, false, true, all_final_s2t);

				// std::cout << "all_final_s2t.size() : " << all_final_s2t.size() << std::endl;

				ScanIndex a = i;
				ScanIndex b = (i + 1)%M;

				sipairs.push_back(williams2001::ScanIndexPair(a,b));

				buffer1.clear();
				for (int k = 0; k < all_final_s2t.size(); ++k)
				{
					PairRegistration::PointPair temp = all_final_s2t[k];
					williams2001::PointPairWithWeight temp_1;
					temp_1.w = 1.0;
					temp_1.ppair.first = temp.targetPoint.getVector3fMap().cast<williams2001::Scalar>(); 
					temp_1.ppair.second = temp.sourcePoint.getVector3fMap().cast<williams2001::Scalar>();
					buffer1.push_back(temp_1);
				}
				ppairwwss.push_back(buffer1);
			}
			else if (it_21 != graph.edges.end())
			{
				// std::cout << "refine inverse edge : " << *(*it_21).second << std::endl;
				std::vector<GraphVertex*> vertices1, vertices2;
				Transformations transformations1, transformations2;

				GraphVertexDecompose(vertex1, Transformation::Identity(), NULL, vertices1, transformations1, false);
				GraphVertexDecompose(vertex2, Transformation::Identity(), NULL, vertices2, transformations2, false);

				PairRegistration::PointPairs all_final_s2t;

				generateFinalPointPairs(vertices1, transformations1, vertices2, transformations2, false, true, all_final_s2t);

				// std::cout << "inverse all_final_s2t.size() : " << all_final_s2t.size() << std::endl;

				ScanIndex a = i;
				ScanIndex b = (i + 1)%M;

				sipairs.push_back(williams2001::ScanIndexPair(a,b));

				buffer1.clear();
				for (int k = 0; k < all_final_s2t.size(); ++k)
				{
					PairRegistration::PointPair temp = all_final_s2t[k];
					williams2001::PointPairWithWeight temp_1;
					temp_1.w = 1.0;
					temp_1.ppair.first = temp.targetPoint.getVector3fMap().cast<williams2001::Scalar>(); 
					temp_1.ppair.second = temp.sourcePoint.getVector3fMap().cast<williams2001::Scalar>();
					buffer1.push_back(temp_1);
				}
				ppairwwss.push_back(buffer1);
			}
			else
			{
				std::cout << "loop lose edge" << std::endl;
				exit(1);
			}
		}

		// std::cout << "debug here 3!" << std::endl;

		// std::cout << "M :" << M << std::endl;
		// std::cout << "sipairs.size() : " << sipairs.size() << std::endl;
		// std::cout << "ppairwwss.size() : " << ppairwwss.size() << std::endl;

		williams2001::SRoMCPS sromcps_globalrefine(sipairs, ppairwwss, M);

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
		std::cout << "looprefine rms_error = " <<  rms_error << " total_weight = " << total_weight << std::endl;

		// std::cout << "debug here 4!" << std::endl;

		if (_closing)
		{
			for (int i = 0; i < M; ++i)
			{
				GraphVertex* vertex1 = _graphLoop->loop[i];
				GraphVertex* vertex2 = _graphLoop->loop[(i+1)%M];
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_12 = graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertex1, vertex2 ) );
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it_21 = graph.edges.find( std::pair<GraphVertex*, GraphVertex*>( vertex2, vertex1 ) );

				if (it_12 != graph.edges.end() ) 
				{
					// std::cout << "debug here 1!" << std::endl;

					std::vector<GraphVertex*> vertices1, vertices2;
					Transformations transformations1, transformations2;

					GraphVertexDecompose(vertex1, Transformation::Identity(), NULL, vertices1, transformations1, false);
					GraphVertexDecompose(vertex2, Transformation::Identity(), NULL, vertices2, transformations2, false);

					Transformation transformation1 = Transformation::Identity();
					transformation1.block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*i).cast<float>();
					transformation1.block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*i, 0).cast<float>();			 

					Transformation transformation2 = Transformation::Identity();
					transformation2.block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*((i + 1) %M)).cast<float>();
					transformation2.block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*((i + 1) %M), 0).cast<float>();		

					Transformation newTransformation = transformation1.inverse() * transformation2;
					makeEdgesConsistent(vertices1, transformations1, vertices2, transformations2, newTransformation);

				}
				else if (it_21 != graph.edges.end())
				{
					// std::cout << "debug here 2!" << std::endl;

					std::vector<GraphVertex*> vertices1, vertices2;
					Transformations transformations1, transformations2;

					GraphVertexDecompose(vertex1, Transformation::Identity(), NULL, vertices1, transformations1, false);
					GraphVertexDecompose(vertex2, Transformation::Identity(), NULL, vertices2, transformations2, false);

					Transformation transformation1 = Transformation::Identity();
					transformation1.block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*i).cast<float>();
					transformation1.block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*i, 0).cast<float>();			 

					Transformation transformation2 = Transformation::Identity();
					transformation2.block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*((i + 1) %M)).cast<float>();
					transformation2.block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*((i + 1) %M), 0).cast<float>();		

					Transformation newTransformation = transformation1.inverse() * transformation2;
					makeEdgesConsistent(vertices1, transformations1, vertices2, transformations2, newTransformation);
				}
				else
				{
					std::cout << "loop lose edge" << std::endl;
					exit(1);
				}
			}
		}

		return rms_error;
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
			for (std::multiset<GraphLoop*, GraphLoopComp>::const_iterator it = graph.loops.begin(); it != graph.loops.end(); it++) std::cout << *(*it) << " ";
			std::cout << std::endl;

			std::vector< GraphLoop* > wait_insert;
			
			GraphLoop* currentLoop = *(graph.loops.begin());
			finalLoop = currentLoop;

			//do loop refine to currentLoop  ... ...
			loopRefine(currentLoop, true);

			//generate new loops to wait_insert
			for (std::multiset<GraphLoop*, GraphLoopComp>::const_iterator it = graph.loops.begin(); it != graph.loops.end(); it++) 
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
			for (std::multiset<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++)
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
		williams2001::ScanIndexPairs sipairs;
		std::vector<williams2001::PointPairWithWeights> ppairwwss;
		int M = scanPtrs.size();

		williams2001::PointPairWithWeights buffer1;

		for (int i = 0; i < links.size(); ++i)
		{
			Link link = links[i];
			ScanIndex a = link.a;
			ScanIndex b = link.b;

			sipairs.push_back(williams2001::ScanIndexPair(a,b));

			buffer1.clear();
			for (int k = 0; k < pairRegistrationPtrMap[link]->final_s2t.size(); ++k)
			{
				PairRegistration::PointPair temp = pairRegistrationPtrMap[link]->final_s2t[k];
				Transformation transformation = pairRegistrationPtrMap[link]->transformation;
				PairRegistration::PointPair temp1, temp2;
				temp1.targetPoint = registar::transformPointWithNormal( temp.sourcePoint, transformation );
				temp1.sourcePoint = temp.sourcePoint;
				temp2.targetPoint = temp.targetPoint;
				temp2.sourcePoint = registar::transformPointWithNormal( temp.targetPoint, transformation.inverse() );

				williams2001::PointPairWithWeight temp_1;
				temp_1.w = 1.0;
				temp_1.ppair.first = temp1.targetPoint.getVector3fMap().cast<williams2001::Scalar>(); 
				temp_1.ppair.second = temp1.sourcePoint.getVector3fMap().cast<williams2001::Scalar>();
				buffer1.push_back(temp_1);

				williams2001::PointPairWithWeight temp_2;
				temp_2.w = 1.0;
				temp_2.ppair.first = temp2.targetPoint.getVector3fMap().cast<williams2001::Scalar>(); 
				temp_2.ppair.second = temp2.sourcePoint.getVector3fMap().cast<williams2001::Scalar>();
				buffer1.push_back(temp_2);
			}
			ppairwwss.push_back(buffer1);
		}

		williams2001::SRoMCPS sromcps_globalrefine(sipairs, ppairwwss, M);	

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

		for (int i = 0; i < M; ++i)
		{
			transformations[i].block<3, 3>(0, 0) = sromcps_globalrefine.R.block<3, 3>(0, 3*i).cast<float>();
			transformations[i].block<3, 1>(0, 3) = sromcps_globalrefine.T.block<3, 1>(3*i, 0).cast<float>();			 
		}
	}

	void GlobalRegistration::globalRefine(unsigned int _iterationNum_max, unsigned int _iterationNum_min)
	{
		PointsPtr buffer(new Points);
		PairRegistration::PointPairs s2t, t2s;

		int threads = omp_get_num_procs();
		std::cout << threads << "threads" << std::endl;

		williams2001::ScanIndexPairs sipairs;
		std::vector<williams2001::PointPairWithWeights> ppairwwss;
		int M = scanPtrs.size();

		williams2001::PointPairWithWeights buffer1;

		float last_rms_error = std::numeric_limits<float>::max();

		for (int iter = 0; iter < _iterationNum_max; ++iter)
		{
			sipairs.clear();
			ppairwwss.clear();
			for (int i = 0; i < links.size(); ++i)
			{
				Link link = links[i];
				ScanIndex a = link.a;
				ScanIndex b = link.b;

				sipairs.push_back(williams2001::ScanIndexPair(a,b));

				buffer->clear();
				s2t.clear();
				t2s.clear();
				Transformation transformation = transformations[a].inverse() * transformations[b];
				PairRegistrationPtr pairRegistrationPtr = pairRegistrationPtrMap[link];
				ScanPtr target = pairRegistrationPtr->target;
				ScanPtr source = pairRegistrationPtr->source;
				KdTreePtr targetKdTree = pairRegistrationPtr->targetKdTree;
				KdTreePtr sourceKdTree = pairRegistrationPtr->sourceKdTree;
				std::vector<int> &targetCandidateIndices = pairRegistrationPtr->targetCandidateIndices;	
				std::vector<int> &targetCandidateIndices_temp = pairRegistrationPtr->targetCandidateIndices_temp;			
				std::vector<int> &sourceCandidateIndices = pairRegistrationPtr->sourceCandidateIndices;
				std::vector<int> &sourceCandidateIndices_temp = pairRegistrationPtr->sourceCandidateIndices_temp;
				PairRegistration::Parameters para = pairRegistrationPtr->para;
				// PairRegistration::generatePointPairs(target, source, targetKdTree, sourceKdTree, 
				// 	targetCandidateIndices, targetCandidateIndices_temp,
				// 	sourceCandidateIndices, sourceCandidateIndices_temp,
				// 	buffer, transformation, para, s2t, t2s);

				PairRegistrationOMP::generatePointPairsOMP(target, source, targetKdTree, sourceKdTree, 
					targetCandidateIndices, targetCandidateIndices_temp,
					sourceCandidateIndices, sourceCandidateIndices_temp,
					buffer, transformation, para, s2t, t2s, threads);

				for (int j = 0; j < s2t.size(); ++j) s2t[j].sourcePoint = registar::transformPointWithNormal(s2t[j].sourcePoint, transformation.inverse());
				for (int j = 0; j < t2s.size(); ++j) t2s[j].sourcePoint = registar::transformPointWithNormal(t2s[j].sourcePoint, transformation);

				buffer1.clear();
				for (int j = 0; j < s2t.size(); ++j)
				{
					williams2001::PointPairWithWeight temp;
					temp.w = 1.0;
					temp.ppair.first = s2t[j].targetPoint.getVector3fMap().cast<williams2001::Scalar>(); 
					temp.ppair.second = s2t[j].sourcePoint.getVector3fMap().cast<williams2001::Scalar>();
					buffer1.push_back(temp);
				}
				for (int j = 0; j < t2s.size(); ++j)
				{
					williams2001::PointPairWithWeight temp;
					temp.w = 1.0;
					temp.ppair.first = t2s[j].sourcePoint.getVector3fMap().cast<williams2001::Scalar>(); 
					temp.ppair.second = t2s[j].targetPoint.getVector3fMap().cast<williams2001::Scalar>();
					buffer1.push_back(temp);
				}
				// std::cout << transformation << std::endl;
				// std::cout << link.a << " <<-- " << link.b << " : " << buffer1.size() << std::endl;	
				ppairwwss.push_back(buffer1);
			}

			williams2001::SRoMCPS sromcps_globalrefine(sipairs, ppairwwss, M);

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

			if ( last_rms_error < rms_error && iter > _iterationNum_min)
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

