#pragma once
#include <string>
#include "OBJ_Loader.h"
#include "Triangle.h"
#include <vector>
#include <Eigen/Core>
#include "Texture.h"

class MeshTriangle
{
public:
	std::vector<Triangle> triangles;
	Texture texture;
	MeshTriangle(const std::string& filepath)
	{
		objl::Loader loader;
		loader.LoadFile(filepath);
		for (const auto& mesh : loader.LoadedMeshes) {
			for (size_t i = 0; i < mesh.Vertices.size(); i += 3) {
				Triangle triangle;
				for (int j = 0; j < 3; j++) {
					size_t idx = i + j;
					triangle.vertices[j] = {
						mesh.Vertices[idx].Position.X,
						mesh.Vertices[idx].Position.Y,
						mesh.Vertices[idx].Position.Z, 1
					};
					triangle.normal[j] = {
						mesh.Vertices[idx].Normal.X,
						mesh.Vertices[idx].Normal.Y,
						mesh.Vertices[idx].Normal.Z
					};
					triangle.texCoords[j] = {
						mesh.Vertices[idx].TextureCoordinate.X,
						mesh.Vertices[idx].TextureCoordinate.Y,
					};
				}
				triangles.emplace_back(triangle);
			}
		}
	}
};