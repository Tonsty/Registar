#ifndef MATHUTILITIES_H
#define MATHUTILITIES_H

#include <Eigen/Dense>

#include "pclbase.h"

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



#endif