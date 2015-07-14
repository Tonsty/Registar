#ifdef _OPENMP
#include <omp.h>
#endif

#include "pairregistration.h"
#include "../include/utilities.h"

#include <pcl/common/transforms.h>

namespace Tang2014
{
	void PairRegistration::startRegistration()
	{
		// initiateCandidateIndices();

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
				generatePointPairs(target, source, targetKdTree, sourceKdTree, 
					targetCandidateIndices, targetCandidateIndices_temp, 
					sourceCandidateIndices, sourceCandidateIndices_temp,
					buffer, initialTransformation, para, s2t, t2s);
				// std::cout << "s2t.size() = " << s2t.size() << std::endl;
				// std::cout << "t2s.size() = " << t2s.size() << std::endl;
				for (int i = 0; i < t2s.size(); ++i)
				{
					PointPair temp;
					temp.sourcePoint = registar::transformPointWithNormal(t2s[i].targetPoint, initialTransformation );
					temp.targetPoint = registar::transformPointWithNormal(t2s[i].sourcePoint, initialTransformation );
					s2t.push_back(temp);
				}
			}
			else 
			{
				generatePointPairs(target, source, targetKdTree, sourceCandidateIndices, sourceCandidateIndices_temp, buffer, initialTransformation, para, s2t);
				// std::cout << "s2t.size() = " << s2t.size() << std::endl;
			}
			tempTransformation = solveRegistration(s2t, src, tgt);	

			float total_error = 0.0f;
			float total_weight = 0.0f;
			for (int i = 0; i < s2t.size(); ++i)
			{
				total_error += ( registar::transformPointWithNormal(s2t[i].sourcePoint, tempTransformation).getVector3fMap() - s2t[i].targetPoint.getVector3fMap() ).squaredNorm();
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
		std::cout << transformation << std::endl;

		final_s2t.swap(s2t);
		for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = registar::transformPointWithNormal(final_s2t[j].sourcePoint, lastTransformation.inverse());
	}

	void PairRegistration::initiateCandidateIndices()
	{
		targetCandidateIndices.clear();
		sourceCandidateIndices.clear();
		for (int i = 0; i < target->pointsPtr->size(); ++i) targetCandidateIndices.push_back(i);
		for (int i = 0; i < source->pointsPtr->size(); ++i) sourceCandidateIndices.push_back(i);
	}

	void PairRegistration::generatePointPairs(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, 
											std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
											PointsPtr _sbuffer, Transformation _transformation, PairRegistration::Parameters _para, PointPairs &_s2t)
	{
		pcl::transformPointCloudWithNormals(*_source->pointsPtr, *_sbuffer, _transformation);

		MatchMethod mMethod = _para.mMethod;
		bool distanceTest = _para.distanceTest;
		float distThreshold = _para.distThreshold;
		bool angleTest = _para.angleTest;
		float angleThreshold = _para.angleThreshold;
		bool boundaryTest = _para.boundaryTest;

		float distanceThreshold2 = distThreshold * distThreshold;
		float cosAngleThreshold = cosf(angleThreshold / 180.f * M_PI);

		// std::vector<float> distances;

		_sourceCandidateIndices_temp.clear();

		for (std::vector<int>::iterator it = _sourceCandidateIndices.begin(); it != _sourceCandidateIndices.end(); it++)
		{
			int index_query = *it;

		// for (int index_query = 0; index_query < _sbuffer->size(); ++index_query)
		// {	

			Point point_query = (*_sbuffer)[index_query];
			int K = 1;
			std::vector<int> indices(K);
			std::vector<float> distance2s(K);
			if ( _targetKdTree->nearestKSearch(point_query, K, indices, distance2s) > 0 )
			{
				int index_match = indices[0];
				if ( (!boundaryTest) || ((*_target->boundariesPtr)[index_match].boundary_point == 0) )
				{
					if ( (!distanceTest) || (distance2s[0] < distanceThreshold2) )
					{
						Point point_match = (*_target->pointsPtr)[index_match];
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
							// distances.push_back( ( point_query.getVector3fMap() - point_match.getVector3fMap() ).norm() );
							_s2t.push_back(pointPair);
						}
					}
				}
				
				if ( ((*_target->boundariesPtr)[index_match].boundary_point == 0) ||  distance2s[0] < ( distanceThreshold2 * 2.0f * 2.0f ) )
				{
					_sourceCandidateIndices_temp.push_back(index_query);
				}
			}
		}

		// _sourceCandidateIndices.swap(_sourceCandidateIndices_temp);

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

	void PairRegistration::generatePointPairs(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree, 
		std::vector<int> &_targetCandidateIndices, std::vector<int> &_targetCandidateIndices_temp,
		std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
		PointsPtr _buffer, Transformation _transformation, PairRegistration::Parameters _para, PointPairs &_s2t, PointPairs &_t2s)
	{
		generatePointPairs(_target, _source, _targetKdTree, _sourceCandidateIndices, _sourceCandidateIndices_temp, _buffer, _transformation, _para, _s2t);
		generatePointPairs(_source, _target, _sourceKdTree, _targetCandidateIndices, _targetCandidateIndices_temp, _buffer, _transformation.inverse(), _para,_t2s);
	}

	void PairRegistration::generateFinalPointPairs(Transformation _transformation)
	{
		PointsPtr buffer(new Points);
		PointPairs s2t, t2s;

		generatePointPairs(target, source, targetKdTree, sourceKdTree, 
			targetCandidateIndices, targetCandidateIndices_temp,
			sourceCandidateIndices, sourceCandidateIndices_temp,
			buffer, _transformation, para, s2t, t2s);

		for (int i = 0; i < t2s.size(); ++i)
		{
			PointPair temp;
			temp.sourcePoint = registar::transformPointWithNormal(t2s[i].targetPoint, _transformation);
			temp.targetPoint = registar::transformPointWithNormal(t2s[i].sourcePoint, _transformation);
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
		for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = registar::transformPointWithNormal(final_s2t[j].sourcePoint, _transformation.inverse());
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

	#ifdef _OPENMP

	void PairRegistrationOMP::startRegistrationOMP()
	{
		if (threads == 0 || threads == 1) 
		{
			startRegistration();
			return;
		}

		// initiateCandidateIndices();

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
				generatePointPairsOMP(target, source, targetKdTree, sourceKdTree, 
					targetCandidateIndices, targetCandidateIndices_temp, 
					sourceCandidateIndices, sourceCandidateIndices_temp,
					buffer, initialTransformation, para, s2t, t2s, threads);
				// std::cout << s2t.size() << std::endl;
				// std::cout << t2s.size() << std::endl;
				for (int i = 0; i < t2s.size(); ++i)
				{
					PointPair temp;
					temp.sourcePoint = registar::transformPointWithNormal(t2s[i].targetPoint, initialTransformation);
					temp.targetPoint = registar::transformPointWithNormal(t2s[i].sourcePoint, initialTransformation);
					s2t.push_back(temp);
				}
			}
			else 
			{
				generatePointPairsOMP(target, source, targetKdTree, sourceCandidateIndices, sourceCandidateIndices_temp, buffer, initialTransformation, para, s2t, threads);
			}
			tempTransformation = solveRegistration(s2t, src, tgt);	

			float total_error = 0.0f;
			float total_weight = 0.0f;
			for (int i = 0; i < s2t.size(); ++i)
			{
				total_error += ( registar::transformPointWithNormal(s2t[i].sourcePoint, tempTransformation).getVector3fMap() - s2t[i].targetPoint.getVector3fMap() ).squaredNorm();
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
		for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = registar::transformPointWithNormal(final_s2t[j].sourcePoint, lastTransformation.inverse());


	}

	void PairRegistrationOMP::generatePointPairsOMP(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, 
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _sbuffer, const Transformation &_transformation, PairRegistration::Parameters _para, PointPairs &_s2t, unsigned int _threads)
	{
		pcl::transformPointCloudWithNormals(*_source->pointsPtr, *_sbuffer, _transformation);

		MatchMethod mMethod = _para.mMethod;
		bool distanceTest = _para.distanceTest;
		float distThreshold = _para.distThreshold;
		bool angleTest = _para.angleTest;
		float angleThreshold = _para.angleThreshold;
		bool boundaryTest = _para.boundaryTest;

		float distanceThreshold2 = distThreshold * distThreshold;
		float cosAngleThreshold = cosf(angleThreshold / 180.f * M_PI);

		// std::vector<float> distances;

		_sourceCandidateIndices_temp.clear();

		std::vector< std::vector<int> > sourceCandidateIndices_in_threads( _threads );
		std::vector<PointPairs> s2t_in_threads( _threads );

		// for (std::vector<int>::iterator it = _sourceCandidateIndices.begin(); it != _sourceCandidateIndices.end(); it++)
		// {
		// 	int index_query = *it;
		// for (int index_query = 0; index_query < _sbuffer->size(); ++index_query)
		// {

	    #pragma omp parallel for schedule (dynamic,1000) num_threads (_threads)
		for (int it = 0; it < _sourceCandidateIndices.size(); it++)
		{
			int tn = omp_get_thread_num (); 
			
			int index_query = _sourceCandidateIndices[it];

			Point point_query = (*_sbuffer)[index_query];
			int K = 1;
			std::vector<int> indices(K);
			std::vector<float> distance2s(K);
			if ( _targetKdTree->nearestKSearch(point_query, K, indices, distance2s) > 0 )
			{
				int index_match = indices[0];
				if ( (!boundaryTest) || ((*_target->boundariesPtr)[index_match].boundary_point == 0) )
				{
					if ( (!distanceTest) || (distance2s[0] < distanceThreshold2) )
					{
						Point point_match = (*_target->pointsPtr)[index_match];
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
							// distances.push_back( ( point_query.getVector3fMap() - point_match.getVector3fMap() ).norm() );
							// _s2t.push_back(pointPair);
							s2t_in_threads[tn].push_back(pointPair);
						}
					}
				}
				
				if ( ((*_target->boundariesPtr)[index_match].boundary_point == 0) ||  distance2s[0] < ( distanceThreshold2 * 2.0f * 2.0f ) )
				{
					// _sourceCandidateIndices_temp.push_back(index_query);
					sourceCandidateIndices_in_threads[tn].push_back(index_query);
				}
			}
		}

		for(int tn = 0; tn < _threads; tn++)
		{
			_s2t.insert(_s2t.end(), s2t_in_threads[tn].begin(), s2t_in_threads[tn].end());
			_sourceCandidateIndices_temp.insert(_sourceCandidateIndices_temp.end(), sourceCandidateIndices_in_threads[tn].begin(), sourceCandidateIndices_in_threads[tn].end());
		}

		// _sourceCandidateIndices.swap(_sourceCandidateIndices_temp);

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

	void PairRegistrationOMP::generatePointPairsOMP(ScanPtr _target, ScanPtr _source, KdTreePtr _targetKdTree, KdTreePtr _sourceKdTree, 
								std::vector<int> &_targetCandidateIndices, std::vector<int> &_targetCandidateIndices_temp,
								std::vector<int> &_sourceCandidateIndices, std::vector<int> &_sourceCandidateIndices_temp,
								PointsPtr _buffer, const Transformation &_transformation, PairRegistration::Parameters _para, PointPairs &_s2t, PointPairs &_t2s, unsigned int _thread)
	{
		generatePointPairsOMP(_target, _source, _targetKdTree, _sourceCandidateIndices, _sourceCandidateIndices_temp, _buffer, _transformation, _para, _s2t, _thread);
		generatePointPairsOMP(_source, _target, _sourceKdTree, _targetCandidateIndices, _targetCandidateIndices_temp, _buffer, _transformation.inverse(), _para,_t2s, _thread);
	}

	void PairRegistrationOMP::generateFinalPointPairs(Transformation _transformation)
	{
		PointsPtr buffer(new Points);
		PointPairs s2t, t2s;

		generatePointPairsOMP(target, source, targetKdTree, sourceKdTree, 
			targetCandidateIndices, targetCandidateIndices_temp,
			sourceCandidateIndices, sourceCandidateIndices_temp,
			buffer, _transformation, para, s2t, t2s, threads);

		for (int i = 0; i < t2s.size(); ++i)
		{
			PointPair temp;
			temp.sourcePoint = registar::transformPointWithNormal(t2s[i].targetPoint, _transformation);
			temp.targetPoint = registar::transformPointWithNormal(t2s[i].sourcePoint, _transformation);
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
		for (int j = 0; j < final_s2t.size(); ++j) final_s2t[j].sourcePoint = registar::transformPointWithNormal(final_s2t[j].sourcePoint, _transformation.inverse());

		// std::cout << "Final Point Pairs : " << final_s2t.size() << std::endl;
	}

	#endif
}
