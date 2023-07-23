#pragma once
#include <Eigen/Core>
#include <array>

class Triangle
{
public:
	std::array<Eigen::Vector4f, 3> vertices;
	std::array<Eigen::Vector3f, 3> normal;
	std::array<Eigen::Vector2f, 3> texCoords;
	Eigen::Vector3f barycentriCoords(int x, int y) const
	{
		auto& A = vertices[0];
		auto& B = vertices[1];
		auto& C = vertices[2];
		float gamma = ((A.y() - B.y()) * x + (B.x() - A.x()) * y + A.x() * B.y() - B.x() * A.y()) /
			((A.y() - B.y()) * C.x() + (B.x() - A.x()) * C.y() + A.x() * B.y() - B.x() * A.y());
		float beta = ((A.y() - C.y()) * x + (C.x() - A.x()) * y + A.x() * C.y() - C.x() * A.y()) /
			((A.y() - C.y()) * B.x() + (C.x() - A.x()) * B.y() + A.x() * C.y() - C.x() * A.y());
		float alpha = 1 - beta - gamma;
		return {alpha, beta, gamma};
	}
};