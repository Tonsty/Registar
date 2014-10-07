#include "globalregistration.h"

#include "SRoMCPS.h"

#include "../include/mathutilities.h"

void GlobalRegistration::startRegistration()
{
	// buildKdTreePtrs();
	// initialTransformations();	
	// initialPairRegistration();
	incrementalLoopRefine();
	//globalRefine(60);
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
		pairReigstrationPtr->setParameter(PairRegistration::POINT_TO_PLANE, PairRegistration::UMEYAMA, Transformation::Identity(), true, 0.12f, true, 45.0f, true, true, 60);
		//pairReigstrationPtr->startRegistration();
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
		GraphEdge *temp = new GraphEdge(graph.vertices[a], graph.vertices[b]);
		//temp->ebase.transformation = pairRegistrationPtrMap[link]->transformation; //assign initial pair registration result
		graph.edges[std::pair<GraphVertex*, GraphVertex*>(graph.vertices[a], graph.vertices[b])] = temp;
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

		for (std::set<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++) 
		{
			if ( it != graph.loops.begin() && (*it)->intersected(currentLoop) )
			{
				std::vector<GraphLoop*> newloops = (*it)->blend(currentLoop);
				std::cout << "new loops : ";
				for (int i = 0; i < newloops.size(); ++i) std::cout << *newloops[i] << " "; 
				std::cout << std::endl;
				wait_insert.insert(wait_insert.end(), newloops.begin(), newloops.end());
			}
		}

		std::multiset<GraphLoop*, GraphLoopComp> temp;
		for (std::set<GraphLoop*, GraphLoopComp>::iterator it = graph.loops.begin(); it != graph.loops.end(); it++)
		{
			if ( it != graph.loops.begin() && !(*it)->intersected(currentLoop) )
			{
				temp.insert(*it);
			}
		}
		graph.loops = temp;

		for (int i = 0; i < wait_insert.size(); ++i)
		{
			if ( wait_insert[i]->loop.size() > 2 ) //polygon or triangle loop case, add two connected new edge and the loop
			{
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it1 = 
					graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop.back() ) );
				if ( it1 == graph.edges.end() )
				{
					// add new edge
				}

				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it2 = 
					graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop[1] ) );
				if ( it2 != graph.edges.end() )
				{
					// add new edge
				}

				wait_insert[i]->lbase = loopEstimateConsistencyError(wait_insert[i]);  //assign consistency priority

				graph.loops.insert(wait_insert[i]);
			}
			else if( wait_insert[i]->loop.size() == 2 ) //single edge loop case, add single new edge that doesn't belong to any other polygon or triangle loop 
			{
				std::map< std::pair<GraphVertex*, GraphVertex*>, GraphEdge*>::iterator  it = 
					graph.edges.find(std::pair<GraphVertex*, GraphVertex*>( wait_insert[i]->loop.front(), wait_insert[i]->loop.back() ) );
				if ( it == graph.edges.end() )
				{
					// add new edge
				}
				graph.loops.insert(wait_insert[i]);		//add if doesn't belong to any other polygon or triangle loop				
			}
			//else internal loop

		}
	}

	std::cout << "Final loop : " << *finalLoop << std::endl;
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

