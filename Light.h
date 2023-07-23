#pragma once
#include <Eigen/Core>

class Light
{
public:
	Eigen::Vector3f position_;
	Eigen::Vector3f intensity_;
	Light(Eigen::Vector3f position, Eigen::Vector3f intensity): position_(position), intensity_(intensity) {}
};