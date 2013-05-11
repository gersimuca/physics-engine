#include <SDL.h>
#include "Ball.h"
#include <vector>
#include <cstdlib>
#include <ctime>

// Screen dimensions
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

// Function to generate a random float between min and max
float randFloat(float min, float max) {
    return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
}

int main(int argc, char* args[]) {
    // Initialize SDL
    SDL_Init(SDL_INIT_VIDEO);

    // Create an SDL window
    SDL_Window* window = SDL_CreateWindow("Bouncing Balls", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    // Create an SDL renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Initialize random seed
    srand(static_cast<unsigned>(time(0)));

    // Create a vector to store balls
    std::vector<Ball> balls;

    // Add 10 balls to the vector with random properties
    for (int i = 0; i < 10; ++i) {
        float x = randFloat(50.0f, SCREEN_WIDTH - 50.0f);
        float y = randFloat(50.0f, SCREEN_HEIGHT - 50.0f);
        float radius = randFloat(10.0f, 30.0f);
        float vx = randFloat(-200.0f, 200.0f);
        float vy = randFloat(-200.0f, 200.0f);
        SDL_Color color = { static_cast<Uint8>(rand() % 256), static_cast<Uint8>(rand() % 256), static_cast<Uint8>(rand() % 256), 255 };
        balls.emplace_back(x, y, radius, vx, vy, color);
    }

    bool quit = false;
    SDL_Event e;
    Uint32 startTime = SDL_GetTicks();

    // Main game loop
    while (!quit) {
        // Handle events
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        // Calculate delta time
        Uint32 currentTime = SDL_GetTicks();
        float deltaTime = (currentTime - startTime) / 1000.0f;
        startTime = currentTime;

        // Update balls
        for (auto& ball : balls) {
            ball.update(deltaTime);
        }

        // Check for and resolve collisions between balls
        for (size_t i = 0; i < balls.size(); ++i) {
            for (size_t j = i + 1; j < balls.size(); ++j) {
                if (balls[i].checkCollision(balls[j])) {
                    balls[i].resolveCollision(balls[j]);
                }
            }
        }

        // Clear the screen
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Render balls
        for (auto& ball : balls) {
            ball.render(renderer);
        }

        // Update the screen
        SDL_RenderPresent(renderer);
    }

    // Clean up
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
