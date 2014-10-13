#include "pairregistration.h"
#include "../include/mathutilities.h"

#include <pcl/common/transforms.h>

void PairRegistration::startRegistration()
{
	PointsPtr buffer(new Points);
	PointPairs s2t, t2s;
	Eigen::Matrix3Xf src, tgt;

	float last_rms_error = std::numeric_limits<float>::max();

	Transformation lastTransformation = Transformation::Identity();
	Transformation initialTransformation = transformation;
	for (int iter = 0; iter < para.iterationNum_max; ++iter)
	{
		s2t.clear();
		t2s.clear();
		buffer->clear();
		Transformation tempTransformation;
		if(para.biDirection) 
		{
			generatePointPairs(buffer, initialTransformation, s2t, t2s);
			// std::cout << s2t.size() << std::endl;
			// std::cout << t2s.size() << std::endl;
			for (int i = 0; i < t2s.size(); ++i)
			{
				PointPair temp;
				// temp.sourcePoint = pcl::transformPoint(t2s[i].targetPoint, Eigen::Affine3f( initialTransformation ) );
				// temp.targetPoint = pcl::transformPoint(t2s[i].sourcePoint, Eigen::Affine3f( initialTransformation ) );
				temp.sourcePoint = transformPointWithNormal(t2s[i].targetPoint, initialTransformation);
				temp.targetPoint = transformPointWithNormal(t2s[i].sourcePoint, initialTransformation);
				s2t.push_back(temp);
			}
		}
		else 
		{
			generatePointPairs(buffer, initialTransformation, s2t);
		}
		tempTransformation = solveRegistration(s2t, src, tgt);	

		float total_error = 0.0f;
		float total_weight = 0.0f;
		for (int i = 0; i < s2t.size(); ++i)
		{
			total_error += ( transformPointWithNormal(s2t[i].sourcePoint, tempTransformation).getVector3fMap() - s2t[i].targetPoint.getVector3fMap() ).squaredNorm();
			total_weight += 1.0f;
		}
		float rms_error = sqrtf( total_error / total_weight );
		std::cout << "pairregistration rms_error = " <<  rms_error << " total_weight = " << total_weight << std::endl;
		if ( last_rms_error < rms_error && iter > para.iterationNum_min )
		{
			std::cout << "pairregistration converged after " << iter << " iteration(s)" << std::endl;
			break;
		}
		last_rms_error = rms_error;

		lastTransformation = initialTransformation;
		initialTransformation = tempTransformation * initialTransformation;	
	}
	transformation = initialTransformation;
	// std::cout << transformation << std::endl;

	final_s2t.swap(s2t);
	for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = transformPointWithNormal(final_s2t[j].sourcePoint, lastTransformation.inverse());
}

void PairRegistration::generatePointPairs(PointsPtr _sbuffer, Transformation _transformation, PointPairs &_s2t)
{
	pcl::transformPointCloudWithNormals(*source->pointsPtr, *_sbuffer, _transformation);

	MatchMethod mMethod = para.mMethod;
	bool distanceTest = para.distanceTest;
	float distThreshold = para.distThreshold;
	bool angleTest = para.angleTest;
	float angleThreshold = para.angleThreshold;
	bool boundaryTest = para.boundaryTest;

	float distanceThreshold2 = distThreshold * distThreshold;
	float cosAngleThreshold = cosf(angleThreshold / 180.f * M_PI);

	std::vector<float> distances;

	for (int index_query = 0; index_query < _sbuffer->size(); ++index_query)
	{
		Point point_query = (*_sbuffer)[index_query];
		int K = 1;
		std::vector<int> indices(K);
		std::vector<float> distance2s(K);
		if ( targetKdTree->nearestKSearch(point_query, K, indices, distance2s) > 0 )
		{
			int index_match = indices[0];
			if ( (!boundaryTest) || ((*target->boundariesPtr)[index_match].boundary_point == 0) )
			{
				if ( (!distanceTest) || (distance2s[0] < distanceThreshold2) )
				{
					Point point_match = (*target->pointsPtr)[index_match];
					Eigen::Vector3f normal_query = point_query.getNormalVector3fMap();
					Eigen::Vector3f normal_match = point_match.getNormalVector3fMap();
					normal_query.normalize();
					normal_match.normalize();

					if ( (!angleTest) || (normal_query.dot(normal_match) > cosAngleThreshold) )
					{
						PointPair pointPair;
						pointPair.sourcePoint = point_query;					
						switch(mMethod)
						{
							case POINT_TO_POINT:
							{
								pointPair.targetPoint = point_match;
								break;
							}
							case POINT_TO_PLANE:
							{
								pointPair.targetPoint = point_match;
								pointPair.targetPoint.getVector3fMap() = point_query.getVector3fMap() - (point_query.getVector3fMap() - point_match.getVector3fMap()).dot(normal_match) * normal_match;
								break;
							}
						}
						distances.push_back( ( point_query.getVector3fMap() - point_match.getVector3fMap() ).norm() );
						_s2t.push_back(pointPair);
					}
				}
			}
		}
	}

	// float mean_distance = 0.0f;
	// for (int i = 0; i < distances.size(); ++i) mean_distance += distances[i];
	// mean_distance /= distances.size();
	// float delta_distance = 0.0f;
	// for (int i = 0; i < distances.size(); ++i) delta_distance += ( distances[i] - mean_distance ) * ( distances[i] - mean_distance );
	// delta_distance = sqrtf( delta_distance / distances.size() );

	// PointPairs s2t_temp;
	// for (int i = 0; i < _s2t.size(); i++)
	// {
	// 	if (distances[i] - mean_distance < 1.0f * delta_distance)
	// 	{
	// 		s2t_temp.push_back(_s2t[i]);
	// 	}
	// }
	// std::cout << "cut pair by delta from " << _s2t.size() << " to " << s2t_temp.size() << std::endl;
	// _s2t.swap(s2t_temp);

}

void PairRegistration::generatePointPairs(PointsPtr _buffer, Transformation _transformation, PointPairs &_s2t, PointPairs &_t2s)
{
	generatePointPairs(_buffer, _transformation, _s2t);

	source.swap(target);
	sourceKdTree.swap(targetKdTree);	
	generatePointPairs(_buffer, _transformation.inverse(), _t2s);
	source.swap(target);
	sourceKdTree.swap(targetKdTree);	
}

void PairRegistration::generateFinalPointPairs(Transformation _transformation)
{
	PointsPtr buffer(new Points);
	PointPairs s2t, t2s;

	generatePointPairs(buffer, _transformation, s2t, t2s);

	for (int i = 0; i < t2s.size(); ++i)
	{
		PointPair temp;
		// temp.sourcePoint = pcl::transformPoint(t2s[i].targetPoint, Eigen::Affine3f( initialTransformation ) );
		// temp.targetPoint = pcl::transformPoint(t2s[i].sourcePoint, Eigen::Affine3f( initialTransformation ) );
		temp.sourcePoint = transformPointWithNormal(t2s[i].targetPoint, _transformation);
		temp.targetPoint = transformPointWithNormal(t2s[i].sourcePoint, _transformation);
		s2t.push_back(temp);
	}

	// float total_error = 0.0f;
	// float total_weight = 0.0f;
	// for (int i = 0; i < s2t.size(); ++i)
	// {
	// 	total_error += ( s2t[i].sourcePoint.getVector3fMap() - s2t[i].targetPoint.getVector3fMap() ).squaredNorm();
	// 	total_weight += 1.0f;
	// }
	// float rms_error = sqrtf( total_error / total_weight );
	// std::cout << "Final Point Pairs rms_error = " <<  rms_error << " total_weight = " << total_weight << std::endl;	

	final_s2t.swap(s2t);
	for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = transformPointWithNormal(final_s2t[j].sourcePoint, _transformation.inverse());

	// std::cout << "Final Point Pairs : " << final_s2t.size() << std::endl;
}

Transformation PairRegistration::solveRegistration(PointPairs &_s2t, Eigen::Matrix3Xf &src, Eigen::Matrix3Xf &tgt)
{
	src.resize(Eigen::NoChange, _s2t.size());
	tgt.resize(Eigen::NoChange, _s2t.size());

	for (int i = 0; i < _s2t.size(); ++i)
	{
		src(0, i) = _s2t[i].sourcePoint.x;
		src(1, i) = _s2t[i].sourcePoint.y;
		src(2, i) = _s2t[i].sourcePoint.z;

		tgt(0, i) = _s2t[i].targetPoint.x;
		tgt(1, i) = _s2t[i].targetPoint.y;
		tgt(2, i) = _s2t[i].targetPoint.z;
	}

	switch(para.sMethod)
	{
		case UMEYAMA:
		{
			return pcl::umeyama (src, tgt, false);
		}
		case SVD:
		{
			return Transformation::Identity();
		}
	}
	return Transformation::Identity();
}

Transformation PairRegistration::solveRegistration(PointPairs &_s2t, PairRegistration::SolveMethod _sMethod)
{
	Eigen::Matrix3Xf src, tgt;
	src.resize(Eigen::NoChange, _s2t.size());
	tgt.resize(Eigen::NoChange, _s2t.size());

	for (int i = 0; i < _s2t.size(); ++i)
	{
		src(0, i) = _s2t[i].sourcePoint.x;
		src(1, i) = _s2t[i].sourcePoint.y;
		src(2, i) = _s2t[i].sourcePoint.z;

		tgt(0, i) = _s2t[i].targetPoint.x;
		tgt(1, i) = _s2t[i].targetPoint.y;
		tgt(2, i) = _s2t[i].targetPoint.z;
	}	

	switch(_sMethod)
	{
		case UMEYAMA:
		{
			return pcl::umeyama (src, tgt, false);
		}
		case SVD:
		{
			return Transformation::Identity();
		}
	}
	return Transformation::Identity();
}