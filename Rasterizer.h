#pragma once
#include <Eigen/Dense>
#include <numbers>
#include "Global.h"
#include "Triangle.h"
#include <vector>
#include "Payload.h"

inline Eigen::Vector3f shader(const Payload& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.texture_->getColor(payload.texCoords_.x(), payload.texCoords_.y());
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
	Eigen::Vector3f ambientLightIntensity {10, 10, 10};
	float p = 150;
	Eigen::Vector3f color {0, 0, 0};
	for (auto& light : payload.viewSpaceLights_) {
		auto l = (light.position_ - payload.viewPos_).normalized();
		auto v = (-payload.viewPos_).normalized();
		auto h = (l + v).normalized();
		float r2 = (light.position_ - payload.viewPos_).dot(light.position_ - payload.viewPos_);
		auto ambient = ka.cwiseProduct(ambientLightIntensity);
		auto diffuse = kd.cwiseProduct(light.intensity_ / r2 * std::max(0.f, payload.normal_.dot(l)));
		auto specular = ks.cwiseProduct(light.intensity_ / r2 * std::pow(std::max(0.f, payload.normal_.dot(h)), p));
		color += ambient + diffuse + specular;
	}
	color[0] = std::min(color[0], 1.0f);
	color[1] = std::min(color[1], 1.0f);
	color[2] = std::min(color[2], 1.0f);
	return color;
}

class Rasterizer
{
public:
	int width_;
	int height_;
	Eigen::Matrix4f model;
	Eigen::Matrix4f view;
	Eigen::Matrix4f projection;
	std::vector<float> depthBuffer;
	SDL_Surface* frameBuffer = nullptr;
	Rasterizer(int width, int height): width_(width), height_(height)
	{
		depthBuffer.resize(width * height);
		frameBuffer = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_RGBA8888);
	}
	void clear()
	{
		std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::lowest());
		SDL_FillRect(frameBuffer, NULL, 0);
	}
	int getIndex(int x, int y)
	{
		return width_ * (height_ - y - 1) + x;
	}
	void setModel(float angle)
	{
		float rad = std::numbers::pi * angle / 180.f;
		Eigen::Matrix4f R {{cos(rad), 0, sin(rad), 0},
						   {0, 1, 0, 0},
						   {-sin(rad), 0, cos(rad), 0},
						   {0, 0, 0, 1}};
		Eigen::Matrix4f S {{2.5, 0, 0, 0},
						   {0, 2.5, 0, 0},
						   {0, 0, 2.5, 0},
						   {0, 0, 0, 1}};
		Eigen::Matrix4f T {{1, 0, 0, 0},
						   {0, 1, 0, 0},
						   {0, 0, 1, 0},
						   {0, 0, 0, 1}};
		model =  T * R * S;
	}
	void setView(Eigen::Vector3f e, Eigen::Vector3f g, Eigen::Vector3f t)
	{
		Eigen::Matrix4f T {{1, 0, 0, -e.x()},
						   {0, 1, 0, -e.y()},
						   {0, 0, 1, -e.z()},
						   {0, 0, 0, 1}};
		auto right = g.cross(t);
		Eigen::Matrix4f R {{right.x(), right.y(), right.z(), 0},
						   {t.x(), t.y(), t.z(), 0},
						   {-g.x(), -g.y(), -g.z(), 0},
						   {0, 0, 0, 1}};
		view = R * T;
	}
	/*
	* zNear and zFar are positive.
	*/
	void setProjection(float yFov, float zNear, float zFar)
	{
		float aspectRatio = static_cast<float>(width_) / height_;
		float n = -zNear;
		float f = -zFar;
		float yRad = ToRad(yFov);
		float t = zNear * tan(yRad * 0.5f);
		float b = -t;
		float r = t * aspectRatio;
		float l = -r;
		Eigen::Matrix4f ortho {{2.f / (r - l), 0, 0, -0.5f * (r + l)},
							   {0, 2.f / (t - b), 0, -0.5f * (t + b)},
							   {0, 0, 2.f / (n - f), -0.5f * (n + f)},
							   {0, 0, 0, 1}};
		Eigen::Matrix4f perspToOrtho {{n, 0, 0, 0},
									  {0, n, 0, 0},
									  {0, 0, n + f, -n * f},
									  {0, 0, 1, 0}};
		projection = ortho * perspToOrtho;
	}
	void draw(const Scene& scene)
	{
		clear();
		SDL_LockSurface(frameBuffer);
		Eigen::Matrix4f mv = view * model;
		Eigen::Matrix4f mvp = projection * mv;
		int i = 0;
		for (auto& mesh : scene.meshTriangles) {
			for (auto& triangle : mesh.triangles) {
				Triangle newTri;
				newTri.vertices = {mvp * triangle.vertices[0], mvp * triangle.vertices[1], mvp * triangle.vertices[2]};
				
				//Homogeneous division
				for (auto& vertex : newTri.vertices) {
					vertex.x() /= vertex.w();
					vertex.y() /= vertex.w();
					vertex.z() /= vertex.w();
				}
				
				for (auto& vertex : newTri.vertices) {
					vertex.x() = (vertex.x() + 1.f) * width_ * 0.5f;
					vertex.y() = (vertex.y() + 1.f) * height_ * 0.5f;
				}

				auto mvInverseTranspose = mv.inverse().transpose();
				newTri.normal = {(mvInverseTranspose * ToVec4(triangle.normal[0], 0)).head(3),
								 (mvInverseTranspose * ToVec4(triangle.normal[1], 0)).head(3),
								 (mvInverseTranspose * ToVec4(triangle.normal[2], 0)).head(3)};
				newTri.texCoords = triangle.texCoords;

				std::array<Eigen::Vector3f, 3> viewSpacePos = {
					(mv * triangle.vertices[0]).head(3),
					(mv * triangle.vertices[1]).head(3),
					(mv * triangle.vertices[2]).head(3)
				};

				std::vector<Light> viewSpaceLights;
				for (auto light : scene.lights) {
					light.position_ = (view * ToVec4(light.position_, 1)).head(3);
					viewSpaceLights.emplace_back(light);
				}
				rasterizerTriangle(newTri, viewSpacePos, &mesh.texture, viewSpaceLights);
			}
		}
		SDL_UnlockSurface(frameBuffer);
	}
	void rasterizerTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& viewSpacePos, const Texture* texture, const std::vector<Light>& viewSpaceLights)
	{
		auto [xMin, xMax] = std::minmax({t.vertices[0].x(), t.vertices[1].x(), t.vertices[2].x()});
		auto [yMin, yMax] = std::minmax({t.vertices[0].y(), t.vertices[1].y(), t.vertices[2].y()});
		for (int y = yMax; y >= static_cast<int>(yMin); y--) {
			for (int x = xMin; x <= xMax; x++) {
				if (x < 0 || x >= width_ || y < 0 || y >= height_) {
					continue;
				}
				float xCenter = x + 0.5;
				float yCenter = y + 0.5;
				auto baryCoordsPrime = t.barycentriCoords(xCenter, yCenter);
				float alphaPrime = baryCoordsPrime.x();
				float betaPrime = baryCoordsPrime.y();
				float gammaPrime = baryCoordsPrime.z();
				if (alphaPrime >= 0 && betaPrime >= 0 && gammaPrime >= 0) {
					float w = 1.f / (alphaPrime / t.vertices[0].w() + betaPrime / t.vertices[1].w() + gammaPrime / t.vertices[2].w());
					if (w > depthBuffer[getIndex(x, y)]) {
						depthBuffer[getIndex(x, y)] = w;
						float alpha = alphaPrime / t.vertices[0].w() * w;
						float beta = betaPrime / t.vertices[1].w() * w;
						float gamma = gammaPrime / t.vertices[2].w() * w;
						auto normal = (alpha * t.normal[0] + beta * t.normal[1] + gamma * t.normal[2]).normalized();
						auto texCoords = alpha * t.texCoords[0] + beta * t.texCoords[1] + gamma * t.texCoords[2];
						auto viewSpaceShadingPos = alpha * viewSpacePos[0] + beta * viewSpacePos[1] + gamma * viewSpacePos[2];
						Payload payload {viewSpaceShadingPos, normal, texCoords, texture, viewSpaceLights};
						auto vecColor = shader(payload) * 255;
						auto numColor = SDL_MapRGB(frameBuffer->format, vecColor[0], vecColor[1], vecColor[2]);
						static_cast<Uint32*>(frameBuffer->pixels)[getIndex(x, y)] = numColor;
					}
				}
			}
		}
	}
};