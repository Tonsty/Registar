#ifndef MATHUTILITIES_H
#define MATHUTILITIES_H

#include <Eigen/Dense>


inline float getScaleFromTransformation(const Eigen::Matrix4f &transformation)
{
	Eigen::Matrix4f T = transformation;
	Eigen::Matrix3f R = T.block(0,0,3,3);
	float c = (R * R.transpose()).diagonal().mean();
	c = sqrtf(c);
	return c;
}

inline Eigen::Matrix4f toRigidTransformation(const Eigen::Matrix4f &nonRigidTransformation)
{
	Eigen::Matrix4f T = nonRigidTransformation;
	Eigen::Matrix3f R = T.block(0,0,3,3);
	float c = (R * R.transpose()).diagonal().mean();
	c = sqrtf(c);
	R /= c;
	Eigen::Vector3f t = T.block(0, 3, 3, 1);

	Eigen::Matrix4f rigidTransformation = Eigen::Matrix4f::Identity();
	rigidTransformation.block(0, 0, 3, 3) = R;
	rigidTransformation.block(0, 3, 3, 1) = t;
	return rigidTransformation;
}



#endif