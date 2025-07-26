#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

constexpr float GRAVITY = 9.8f;   // Gravity constant
constexpr float FPS = 60.0f;      // Frames per second
constexpr float TIME_STEP = 1.0f / FPS;  // Time step per frame

// Utility functions
inline float sqr(float x) { return x * x; }

// Vector2 class for positions, velocities, and forces
struct Vector2 {
    float x, y;

    Vector2() : x(0), y(0) {}
    Vector2(float x, float y) : x(x), y(y) {}

    Vector2 operator+(const Vector2& other) const { return {x + other.x, y + other.y}; }
    Vector2 operator-(const Vector2& other) const { return {x - other.x, y - other.y}; }
    Vector2 operator*(float scalar) const { return {x * scalar, y * scalar}; }
    Vector2 operator/(float scalar) const { return {x / scalar, y / scalar}; }

    float dot(const Vector2& other) const { return x * other.x + y * other.y; }

    float length() const { return std::sqrt(x * x + y * y); }
    void normalize() { float len = length(); if (len > 0) { x /= len; y /= len; } }
};

// Base class for physics objects
struct RigidBody {
    Vector2 position;
    Vector2 velocity;
    Vector2 force;
    float mass;
    bool isStatic;

    RigidBody(float x, float y, float mass = 1.0f, bool isStatic = false)
        : position(x, y), mass(mass), isStatic(isStatic), velocity(0, 0), force(0, 0) {}

    void applyForce(const Vector2& f) { force = force + f; }
    void applyGravity() { if (!isStatic) applyForce({0, GRAVITY * mass}); }
    void integrate() {
        if (!isStatic) {
            velocity = velocity + (force / mass) * TIME_STEP;
            position = position + velocity * TIME_STEP;
        }
        force = {0, 0}; // Reset force after integration
    }
};

// Simple AABB collision detection
bool checkAABBCollision(const RigidBody& a, const RigidBody& b, float widthA, float heightA, float widthB, float heightB) {
    return a.position.x < b.position.x + widthB &&
           a.position.x + widthA > b.position.x &&
           a.position.y < b.position.y + heightB &&
           a.position.y + heightA > b.position.y;
}

// Circle-Circle Collision Detection
bool checkCircleCollision(const RigidBody& a, const RigidBody& b, float radiusA, float radiusB) {
    float dx = a.position.x - b.position.x;
    float dy = a.position.y - b.position.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    return distance < (radiusA + radiusB);
}

// Physics World to simulate interactions
class PhysicsWorld {
public:
    std::vector<std::shared_ptr<RigidBody>> bodies;
    bool debugVisualizationEnabled = false;

    void addBody(std::shared_ptr<RigidBody> body) {
        bodies.push_back(body);
    }

    void update() {
        for (auto& body : bodies) {
            if (!body->isStatic) {
                body->applyGravity();
                body->integrate();
            }
        }

        // Handle collisions (simple brute-force pairwise)
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                auto& bodyA = bodies[i];
                auto& bodyB = bodies[j];

                // AABB vs AABB (simplified, assuming they are rectangular)
                if (checkAABBCollision(*bodyA, *bodyB, 50, 50, 50, 50)) {
                    resolveCollision(*bodyA, *bodyB);
                }

                // Circle vs Circle
                if (checkCircleCollision(*bodyA, *bodyB, 25, 25)) {
                    resolveCollision(*bodyA, *bodyB);
                }
            }
        }
    }

    void resolveCollision(RigidBody& a, RigidBody& b) {
        Vector2 normal = b.position - a.position;
        normal.normalize();

        Vector2 relativeVelocity = b.velocity - a.velocity;
        float velocityAlongNormal = relativeVelocity.dot(normal);

        if (velocityAlongNormal > 0) return; // Ignore if they are separating

        // Coefficient of restitution (elasticity)
        float e = 1.0f;  // Perfectly elastic for now

        float j = -(1 + e) * velocityAlongNormal;
        j /= (1 / a.mass + 1 / b.mass);

        Vector2 impulse = normal * j;

        a.velocity = a.velocity - impulse / a.mass;
        b.velocity = b.velocity + impulse / b.mass;
    }

    void toggleDebugVisualization() {
        debugVisualizationEnabled = !debugVisualizationEnabled;
    }

    void debugDraw() {
        if (debugVisualizationEnabled) {
            for (auto& body : bodies) {
                // Visualize the bodies (could be done using a game framework like SDL, SFML, or similar)
                std::cout << "Body at position: (" << body->position.x << ", " << body->position.y << ")\n";
            }
        }
    }
};

int main() {
    PhysicsWorld world;

    // Create a few rigid bodies
    auto staticWall = std::make_shared<RigidBody>(200, 200, 0.0f, true);
    auto movingBall = std::make_shared<RigidBody>(100, 100, 1.0f);
    movingBall->velocity = {50, 0};

    world.addBody(staticWall);
    world.addBody(movingBall);

    // Simulation loop
    for (int frame = 0; frame < 600; ++frame) {  // Simulate for 10 seconds (60 FPS)
        world.update();

        if (frame % 60 == 0) {  // Toggle debug visualization every 1 second
            world.toggleDebugVisualization();
        }

        world.debugDraw();  // Output visualization data
    }

    return 0;
}
