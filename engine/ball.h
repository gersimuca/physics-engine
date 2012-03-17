#ifndef BALL_H
#define BALL_H

#include <SDL.h>

// Structure to represent a ball in the simulation
struct Ball {
    float x, y;          // Position of the ball
    float vx, vy;        // Velocity of the ball in x and y directions
};

// Function to update the ball's position and handle collisions with window edges
void updateBall(Ball& ball, float deltaTime);

// Function to draw the ball on the screen
void drawBall(SDL_Renderer* renderer, const Ball& ball);

#endif // BALL_H
