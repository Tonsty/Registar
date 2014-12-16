#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>
#include <time.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/features/normal_3d.h>

#include "pclbase.h"

namespace registar
{
	inline float getScaleFromTransformation(const Eigen::Matrix4f &transformation)
	{
		Eigen::Matrix4f T = transformation;
		Eigen::Matrix3f R = T.block(0, 0, 3, 3);
		float c = (R * R.transpose()).diagonal().mean();
		c = sqrtf(c);
		return c;
	}

	inline Eigen::Matrix4f toRigidTransformation(const Eigen::Matrix4f &nonRigidTransformation)
	{
		Eigen::Matrix4f T = nonRigidTransformation;
		Eigen::Matrix3f R = T.block(0, 0, 3, 3);
		float c = (R * R.transpose()).diagonal().mean();
		c = sqrtf(c);
		R /= c;
		Eigen::Vector3f t = T.block(0, 3, 3, 1);

		Eigen::Matrix4f rigidTransformation = Eigen::Matrix4f::Identity();
		rigidTransformation.block(0, 0, 3, 3) = R;
		rigidTransformation.block(0, 3, 3, 1) = t;
		return rigidTransformation;
	}

	inline PointType transformPointWithNormal(const PointType &point, const Eigen::Matrix4f &transformation)
	{
		PointType ret = point;
		ret.x = transformation(0, 0) * point.x + transformation(0, 1) * point.y + transformation(0, 2) * point.z + transformation(0, 3);
		ret.y = transformation(1, 0) * point.x + transformation(1, 1) * point.y + transformation(1, 2) * point.z + transformation(1, 3);
		ret.z = transformation(2, 0) * point.x + transformation(2, 1) * point.y + transformation(2, 2) * point.z + transformation(2, 3);

		ret.normal_x = transformation(0, 0) * point.normal_x + transformation(0, 1) * point.normal_y + transformation(0, 2) * point.normal_z;
		ret.normal_y = transformation(1, 0) * point.normal_x + transformation(1, 1) * point.normal_y + transformation(1, 2) * point.normal_z;
		ret.normal_z = transformation(2, 0) * point.normal_x + transformation(2, 1) * point.normal_y + transformation(2, 2) * point.normal_z;

		return ret;
	}

	inline float randomFloat(float min = 0.0f, float max = 1.0f)
	{
		//srand((unsigned int)time(0));

		int a = rand() % 65536;
		float b = ((float)a)/65535;

		return min + ( max - min ) * b;
	}

	inline Eigen::Vector3f randomUnitVector()
	{
		float u0;
		float u1;
		do{
			u0 = randomFloat(-1.0f, 1.0f);
			u1 = randomFloat(-1.0f, 1.0f);
		}while( (u0 * u0 + u1 * u1 ) > 1.0f );

		float x0 = 2 * u0 * sqrt(1 - u0 * u0 - u1 * u1);
		float x1 = 2 * u1 * sqrt(1 - u0 * u0 - u1 * u1);
		float x2 = 1 - 2 * (u0 * u0 + u1 * u1);

		return Eigen::Vector3f( x0, x1, x2 );	
	}

	inline Eigen::AngleAxisf randomRotation( float angleScale = 1.0f * M_PI)
	{
		return Eigen::AngleAxisf( randomFloat() * angleScale, randomUnitVector() );
	}

	inline Eigen::Vector3f randomTranslation( float translationScale = 1.0f )
	{
		float u0;
		float u1;
		float u2;
		do{
			u0 = randomFloat(-1.0f, 1.0f);
			u1 = randomFloat(-1.0f, 1.0f);
			u2 = randomFloat(-1.0f, 1.0f);
		}while( (u0 * u0 + u1 * u1 + u2 * u2) > 1.0f );
		return Eigen::Vector3f(u0, u1, u2) * translationScale;
	}

	inline Eigen::Matrix4f randomRigidTransformation(float angleScale = 1.0f * M_PI, float translationScale = 1.0f )
	{
		Eigen::Matrix4f randomRigidTransf = Eigen::Matrix4f::Identity();
		randomRigidTransf.block<3,3>(0,0) = randomRotation( angleScale ).toRotationMatrix();
		randomRigidTransf.block<3,1>(0,3) = randomTranslation( translationScale );
		return randomRigidTransf;
	}

	inline void flipPointCloudNormalsTowardsViewpoint(const CloudData &cloud_in, const pcl::PointXYZ &view_point, CloudData &cloud_out)
	{
		pcl::copyPointCloud(cloud_in, cloud_out);
	
		for (int i = 0; i < cloud_in.size(); ++i)
		{
			pcl::flipNormalTowardsViewpoint(cloud_in[i], view_point.x, view_point.y, view_point.z, 
				cloud_out[i].normal_x, cloud_out[i].normal_y, cloud_out[i].normal_z); 
		}
	}

	inline void setPointCloudColor(CloudData &cloud, const unsigned char &r, const unsigned char &g, const unsigned char &b)
	{
		for (int i = 0; i < cloud.size(); i++)
		{
			cloud[i].r = r;
			cloud[i].g = b;
			cloud[i].b = b;
		}
	}

	inline void addNoiseToPointCloud(const CloudData &cloud_in, const float &noise_std, CloudData &cloud_out)
	{
		pcl::copyPointCloud(cloud_in, cloud_out);
		
		boost::mt19937 rng (static_cast<unsigned int> (std::time (0)));
		boost::normal_distribution<float> normal_distrib (0.0f, noise_std * noise_std);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > gaussian_rng (rng, normal_distrib);
		
		for (int i = 0; i < cloud_in.size(); i++)
		{
			cloud_out[i].x = cloud_in[i].x + gaussian_rng();
			cloud_out[i].y = cloud_in[i].y + gaussian_rng();
			cloud_out[i].z = cloud_in[i].z + gaussian_rng();
		}

	}

	inline void addNoiseAlongViewPointToPointCloud(const CloudData &cloud_in, const float &noise_std, const pcl::PointXYZ &view_point, CloudData &cloud_out)
	{
		pcl::copyPointCloud(cloud_in, cloud_out);

		boost::mt19937 rng (static_cast<unsigned int> (std::time (0)));
		boost::normal_distribution<float> normal_distrib (0.0f, noise_std * noise_std);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > gaussian_rng (rng, normal_distrib);

		for (int i = 0; i < cloud_in.size(); i++)
		{
			Eigen::Vector3f view_direction = (cloud_in[i].getVector3fMap() - view_point.getVector3fMap()).normalized();
			cloud_out[i].getVector3fMap() = cloud_in[i].getVector3fMap() + view_direction * gaussian_rng();
		}
	}
}

#endif