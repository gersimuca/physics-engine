#include "Ball.h"
#include <cmath>

// Constructor to initialize a ball
Ball::Ball(float x, float y, float radius, float vx, float vy, SDL_Color color)
    : x(x), y(y), radius(radius), vx(vx), vy(vy), color(color) {}

// Method to update the ball's position
void Ball::update(float dt) {
    x += vx * dt;
    y += vy * dt;

    // Simple boundary collision (walls)
    if (x - radius < 0 || x + radius > 800) {
        vx = -vx; // Reverse velocity on x-axis
    }
    if (y - radius < 0 || y + radius > 600) {
        vy = -vy; // Reverse velocity on y-axis
    }
}

// Method to render the ball on the screen
void Ball::render(SDL_Renderer* renderer) {
    // Drawing a filled circle
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
                SDL_RenderDrawPoint(renderer, static_cast<int>(x + dx), static_cast<int>(y + dy));
            }
        }
    }
}

// Method to check collision with another ball
bool Ball::checkCollision(const Ball& other) {
    float dx = other.x - x;
    float dy = other.y - y;
    float distance = sqrt(dx * dx + dy * dy);
    return distance < (radius + other.radius); // True if balls are colliding
}

// Method to resolve collision with another ball
void Ball::resolveCollision(Ball& other) {
    float dx = other.x - x;
    float dy = other.y - y;
    float distance = sqrt(dx * dx + dy * dy);

    if (distance == 0) {
        return; // Prevent divide by zero
    }

    float overlap = (radius + other.radius) - distance;

    // Normalize the distance vector
    float nx = dx / distance;
    float ny = dy / distance;

    // Separate the balls
    x -= nx * overlap / 2;
    y -= ny * overlap / 2;
    other.x += nx * overlap / 2;
    other.y += ny * overlap / 2;

    // Calculate relative velocity
    float rvx = other.vx - vx;
    float rvy = other.vy - vy;

    // Calculate relative velocity in terms of the normal direction
    float velocityAlongNormal = rvx * nx + rvy * ny;

    // Do not resolve if balls are separating
    if (velocityAlongNormal > 0) {
        return;
    }

    // Calculate restitution (elastic collision)
    float e = 1.0f; // Perfectly elastic collision

    // Calculate impulse scalar
    float j = -(1 + e) * velocityAlongNormal;
    j /= 1 / radius + 1 / other.radius;

    // Apply impulse
    float impulseX = j * nx;
    float impulseY = j * ny;

    vx -= impulseX / radius;
    vy -= impulseY / radius;
    other.vx += impulseX / other.radius;
    other.vy += impulseY / other.radius;
}
