#pragma once
#include <vector>
#include "Light.h"
#include "MeshTriangle.h"
#include <Eigen/Core>

class Scene
{
public:
	std::vector<Light> lights;
	std::vector<MeshTriangle> meshTriangles;
};