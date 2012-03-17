#include "Ball.h"

// Constant for the radius of the ball
const int BALL_RADIUS = 20;

// Function to update the ball's position based on its velocity and check for collisions
void updateBall(Ball& ball, float deltaTime) {
    // // Update position based on velocity and elapsed time
    ball.x += ball.vx * deltaTime;
    ball.y += ball.vy * deltaTime;

    // Check for collision with the left or right window edges
    if (ball.x - BALL_RADIUS < 0 || ball.x + BALL_RADIUS > 800) {
        ball.vx = -ball.vx; // Reverse horizontal velocity
    }

    // Check for collision with the top or bottom window edges
    if (ball.y - BALL_RADIUS < 0 || ball.y + BALL_RADIUS > 600) {
        ball.vy = -ball.vy; // Reverse vertical velocity
    }
}


// Function to draw the ball on the renderer
void drawBall(SDL_Renderer* renderer, const Ball& ball) {
    // Set the draw color to red
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    // Create a rectangle that represents the ball's bounding box
    SDL_Rect ballRect = { static_cast<int>(ball.x - BALL_RADIUS), 
                          static_cast<int>(ball.y - BALL_RADIUS), 
                          BALL_RADIUS * 2, 
                          BALL_RADIUS * 2 };

    // Draw the rectangle (ball) on the renderer
    SDL_RenderFillRect(renderer, &ballRect);
}