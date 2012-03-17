#include <SDL.h>
#include <iostream>
#include "Ball.h"

// Constants for screen dimensions
const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    // Create a window
    SDL_Window* window = SDL_CreateWindow(
        "SDL2 Bouncing Ball",             // Window title
        SDL_WINDOWPOS_UNDEFINED,          // Initial x position
        SDL_WINDOWPOS_UNDEFINED,          // Initial y position
        SCREEN_WIDTH,                     // Width
        SCREEN_HEIGHT,                    // Height
        SDL_WINDOW_SHOWN                  // Flags
    );

    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    // Create a renderer for the window
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Initialize ball with position in the center of the screen and a constant velocity
    Ball ball = { SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f, 300.0f, 300.0f };

    bool quit = false;
    SDL_Event e;
    Uint32 lastTime = SDL_GetTicks(); // Store the current time in milliseconds

    // Main loop
    while (!quit) {
        Uint32 currentTime = SDL_GetTicks(); // Get the current time in milliseconds
        float deltaTime = (currentTime - lastTime) / 1000.0f; // Calculate the time since last frame in seconds
        lastTime = currentTime; // Update the last time

        // Handle events
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true; // Set quit flag if the window close event is detected
            }
        }

        // Update ball position based on elapsed time
        updateBall(ball, deltaTime);

        // Clear the screen with a black color
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Draw the ball on the screen
        drawBall(renderer, ball);

        // Present the rendered frame to the screen
        SDL_RenderPresent(renderer);
    }

    // Clean up
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}