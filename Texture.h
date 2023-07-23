#pragma once
#include <SDL2/SDL_image.h>
#include <string>
#include <Eigen/Core>
#include <cmath>
#include "Global.h"

class Texture
{
public:
	SDL_Surface* image = nullptr;
	Texture() {}
	Texture(const std::string& filepath)
	{
		
		image = IMG_Load(filepath.c_str());
		if (image->format->format != SDL_PIXELFORMAT_RGBA8888) {
			auto imageP = SDL_ConvertSurfaceFormat(image, SDL_PIXELFORMAT_RGBA8888, 0);
			SDL_FreeSurface(image);
			image = imageP;
		}
		SDL_LockSurface(image);
	}
	Eigen::Vector3f getColor(float u, float v) const
	{
		v = 1 - v;
		auto x = static_cast<int>(image->w * u);
		auto y = static_cast<int>(image->h * v);
		Uint32 pixel = static_cast<Uint32*>(image->pixels)[y * image->w + x];
		Uint8 r, g, b;
		SDL_GetRGB(pixel, image->format, &r, &g, &b);
		return {r / 255.f, g / 255.f, b / 255.f};
	}
};