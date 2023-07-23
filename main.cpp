#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include "Scene.h"
#include "Rasterizer.h"
#include <Eigen/Core>

int main(int argc, char** argv)
{
	constexpr int WIDTH = 1200;
	constexpr int HEIGHT = 800;
	SDL_Init(SDL_INIT_EVERYTHING);
	IMG_Init(IMG_INIT_PNG);
	auto window = SDL_CreateWindow("HoMiyaRasterizer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
	auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

	Scene scene;
	Light l1 {{20, 20, 20}, {500, 500, 500}};
	Light l2 {{-20, 20, 0}, {500, 500, 500}};
	scene.lights.emplace_back(l1);
	//scene.lights.emplace_back(l2);
	MeshTriangle mesh {"./spot_triangulated_good.obj"};
	Texture texture {"./spot_texture.png"};
	mesh.texture = texture;
	scene.meshTriangles.emplace_back(mesh);

	Rasterizer rasterizer {WIDTH, HEIGHT};
	rasterizer.setProjection(45, 0.1, 50);

	bool isRunning = true;
	int i = 0;
	float theta = 0;
	float phi = 0;
	const float xMoveToDegreeRatio = 0.05;
	const float yMoveToDegreeRatio = 0.05;
	const float velocity = 0.2;
	float x = 0;
	float y = 0;
	float z = 10;
	SDL_SetRelativeMouseMode(SDL_TRUE);
	while (isRunning) {
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
				case SDL_QUIT: {
					isRunning = false;
					break;
				}
				case SDL_MOUSEMOTION: {
					theta += event.motion.xrel * xMoveToDegreeRatio;
					phi -= event.motion.yrel * yMoveToDegreeRatio;
					break;
				}
				case SDL_KEYDOWN: {
					if (event.key.keysym.scancode == SDL_SCANCODE_ESCAPE) {
						isRunning = false;
					}
					break;
				}
				case SDL_MOUSEBUTTONDOWN: {
					if (event.button.button == SDL_BUTTON_LEFT) {
						Light l {{x, y, z}, {20, 20, 20}};
						scene.lights.emplace_back(l);
					} else if (event.button.button == SDL_BUTTON_RIGHT) {
						Eigen::Vector3f pos {x, y, z};
						auto it = std::min_element(scene.lights.begin(), scene.lights.end(), [pos](auto& l1, auto& l2) {
							return (l1.position_ - pos).squaredNorm() < (l2.position_ - pos).squaredNorm();
						});
						if (it != scene.lights.end()) {
							scene.lights.erase(it);
						}
					}
					break;
				}
			}
		}
		auto keyState = SDL_GetKeyboardState(NULL);
		if (keyState[SDL_SCANCODE_A]) {
			z += -velocity * sin(ToRad(theta));
			x += -velocity * cos(ToRad(theta));
		}
		if (keyState[SDL_SCANCODE_D]) {
			z += velocity * sin(ToRad(theta));
			x += velocity * cos(ToRad(theta));
		}
		if (keyState[SDL_SCANCODE_W]) {
			z += -velocity * cos(ToRad(theta));
			x += velocity * sin(ToRad(theta));
		}
		if (keyState[SDL_SCANCODE_S]) {
			z += velocity * cos(ToRad(theta));
			x += -velocity * sin(ToRad(theta));
		}
		if (keyState[SDL_SCANCODE_LCTRL]) {
			y -= velocity;
		}
		if (keyState[SDL_SCANCODE_SPACE]) {
			y += velocity;
		}
		SDL_RenderClear(renderer);

		rasterizer.setModel(140);
		rasterizer.setView({x, y, z},
						   {cos(ToRad(phi)) * sin(ToRad(theta)), sin(ToRad(phi)), -cos(ToRad(phi)) * cos(ToRad(theta))},
						   {-sin(ToRad(phi)) * sin(ToRad(theta)), cos(ToRad(phi)), sin(ToRad(phi)) * cos(ToRad(theta))});
		rasterizer.draw(scene);
		auto frameTexture = SDL_CreateTextureFromSurface(renderer, rasterizer.frameBuffer);
		SDL_RenderCopy(renderer, frameTexture, NULL, NULL);
		SDL_DestroyTexture(frameTexture);

		SDL_RenderPresent(renderer);
		//SDL_Log("frame %d", i++);
	}
	SDL_DestroyWindow(window);
	SDL_Quit();
	IMG_Quit();
	return 0;
}