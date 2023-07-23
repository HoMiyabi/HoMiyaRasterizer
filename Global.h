#pragma once
#include <numbers>
#include <Eigen/Core>

inline float ToRad(float degree)
{
	return degree * std::numbers::pi / 180.f;
}

inline Eigen::Vector4f ToVec4(Eigen::Vector3f vec, float num)
{
	return {vec[0], vec[1], vec[2], num};
}