#ifndef BALL_H
#define BALL_H

#include <SDL.h>

// Ball class represents a circular ball in the simulation
class Ball {
public:
    // Constructor to initialize a ball
    Ball(float x, float y, float radius, float vx, float vy, SDL_Color color);

    // Method to update the ball's position
    void update(float dt);

    // Method to render the ball on the screen
    void render(SDL_Renderer* renderer);

    // Method to check collision with another ball
    bool checkCollision(const Ball& other);

    // Method to resolve collision with another ball
    void resolveCollision(Ball& other);

private:
    float x, y; // Position of the ball
    float radius; // Radius of the ball
    float vx, vy; // Velocity of the ball
    SDL_Color color; // Color of the ball
};

#endif // BALL_H
