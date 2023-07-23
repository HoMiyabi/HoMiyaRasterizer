#pragma once
#include <Eigen/Core>
#include "Light.h"
#include "Texture.h"

class Payload
{
public:
	Eigen::Vector3f viewPos_;
	Eigen::Vector3f normal_;
	Eigen::Vector2f texCoords_;
	const Texture* texture_;
	const std::vector<Light>& viewSpaceLights_;
	Payload(Eigen::Vector3f viewPos, Eigen::Vector3f normal, Eigen::Vector2f texCoords, const Texture* texture, const std::vector<Light>& viewSpaceLights):
		viewPos_(viewPos), normal_(normal), texCoords_(texCoords), texture_(texture), viewSpaceLights_(viewSpaceLights) { }
};