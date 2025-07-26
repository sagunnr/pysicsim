#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

/**
 * @brief Physics engine simulation.
 * @intuition This system simulates basic rigid body dynamics, including gravity, friction, and elastic collisions.
 * @approach The code implements a simple 2D physics system with a focus on efficient and modular collision detection, 
 * gravitational force application, and velocity updates. It utilizes AABB for rectangular collisions, 
 * Circle-Circle for circular collisions, and resolves collisions using an impulse-based approach.
 * @complexity
 *  - Time complexity: O(N^2) for pairwise collision checks
 *  - Space complexity: O(N) for storing bodies in the physics world
 */

constexpr float GRAVITY = 9.8f;        // Gravity constant
constexpr float FPS = 60.0f;           // Frames per second
constexpr float TIME_STEP = 1.0f / FPS; // Time step per frame
constexpr float FRICTION_COEFFICIENT = 0.5f; // Friction coefficient

// Utility functions
inline float sqr(float x) { return x * x; }

/**
 * @brief A 2D vector class for positions, velocities, and forces.
 */
struct Vector2 {
    float x{0};
    float y{0};

    Vector2() = default;
    Vector2(float xVal, float yVal) : x{xVal}, y{yVal} {}

    // Friend operators for symmetry
    friend Vector2 operator+(const Vector2& lhs, const Vector2& rhs) {
        return Vector2(lhs.x + rhs.x, lhs.y + rhs.y);
    }
    friend Vector2 operator-(const Vector2& lhs, const Vector2& rhs) {
        return Vector2(lhs.x - rhs.x, lhs.y - rhs.y);
    }
    friend Vector2 operator*(const Vector2& lhs, float scalar) {
        return Vector2(lhs.x * scalar, lhs.y * scalar);
    }
    friend Vector2 operator/(const Vector2& lhs, float scalar) {
        return Vector2(lhs.x / scalar, lhs.y / scalar);
    }

    float dot(const Vector2& other) const {
        return x * other.x + y * other.y;
    }

    float length() const {
        return std::sqrt(x * x + y * y);
    }
    
    void normalize() {
        float len = length();
        if (len > 0) {
            x /= len;
            y /= len;
        }
    }
};

/**
 * @brief A class representing a rigid body in the physics simulation.
 */
class RigidBody {
public:
    Vector2 position;     // Position of the body
    float mass;           // Mass of the body
    bool isStatic;        // Whether the body is static (immovable)
    Vector2 velocity;     // Velocity of the body
    Vector2 force;        // Forces acting on the body

    /**
     * @brief Constructs a RigidBody object with the given parameters.
     * @param x The x-coordinate of the body's initial position.
     * @param y The y-coordinate of the body's initial position.
     * @param m The mass of the body.
     * @param isStaticIn Whether the body is static (immovable).
     */
    RigidBody(float x, float y, float m = 1.0f, bool isStaticIn = false)
        : position{x, y}, mass{m}, isStatic{isStaticIn} {}

    /**
     * @brief Applies a force to the body.
     * @param f The force to be applied.
     */
    void applyForce(const Vector2& f) {
        force = force + f;
    }

    /**
     * @brief Applies gravity to the body.
     */
    void applyGravity() {
        if (!isStatic)
            applyForce({0, GRAVITY * mass});
    }

    /**
     * @brief Applies friction to the body.
     * @param normal The normal vector of the collision surface.
     */
    void applyFriction(const Vector2& normal) {
        if (!isStatic) {
            Vector2 velocityAlongNormal = velocity * normal;
            float frictionMagnitude = FRICTION_COEFFICIENT * mass * GRAVITY;

            if (velocityAlongNormal.length() > 0) {
                Vector2 frictionForce = normal * -frictionMagnitude * (velocityAlongNormal.length() / velocityAlongNormal.length());
                applyForce(frictionForce);
            }
        }
    }

    /**
     * @brief Integrates the body's velocity and position based on forces.
     */
    void integrate() {
        if (!isStatic) {
            velocity = velocity + (force / mass) * TIME_STEP;
            position = position + velocity * TIME_STEP;
        }
        force = {0, 0}; // Reset force after integration
    }
};

/**
 * @brief A class representing the physics world containing rigid bodies.
 */
class PhysicsWorld {
public:
    std::vector<std::shared_ptr<RigidBody>> bodies;   // List of all bodies in the world
    bool debugVisualizationEnabled{false};           // Flag to enable/disable debug visualization

    /**
     * @brief Adds a body to the physics world.
     * @param body The body to be added.
     */
    void addBody(std::shared_ptr<RigidBody> body) {
        bodies.push_back(std::move(body));
    }

    /**
     * @brief Updates the physics world (applies gravity, integrates, handles collisions).
     */
    void update() {
        for (const auto& body : bodies) {
            if (!body->isStatic) {
                body->applyGravity();
                body->integrate();
            }
        }

        // Handle collisions (brute-force pairwise)
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                const auto& bodyA = bodies[i];
                const auto& bodyB = bodies[j];

                // AABB vs AABB
                if (checkAABBCollision(*bodyA, *bodyB, 50, 50, 50, 50)) {
                    resolveCollision(*bodyA, *bodyB);
                    bodyA->applyFriction({1, 0}); // Apply friction in X direction
                    bodyB->applyFriction({1, 0});
                }

                // Circle vs Circle
                if (checkCircleCollision(*bodyA, *bodyB, 25, 25)) {
                    resolveCollision(*bodyA, *bodyB);
                    bodyA->applyFriction({1, 0});
                    bodyB->applyFriction({1, 0});
                }
            }
        }
    }

    /**
     * @brief Resolves the collision between two bodies.
     * @param a The first body.
     * @param b The second body.
     */
    void resolveCollision(RigidBody& a, RigidBody& b) const {
        Vector2 normal = b.position - a.position;
        normal.normalize();

        Vector2 relativeVelocity = b.velocity - a.velocity;
        float velocityAlongNormal = relativeVelocity.dot(normal);

        if (velocityAlongNormal > 0) return; // Ignore if they are separating

        constexpr float e = 1.0f;  // Perfectly elastic collision

        float j = -(1 + e) * velocityAlongNormal;
        j /= (1 / a.mass + 1 / b.mass);

        Vector2 impulse = normal * j;

        a.velocity = a.velocity - impulse / a.mass;
        b.velocity = b.velocity + impulse / b.mass;
    }

    /**
     * @brief Toggles the debug visualization.
     */
    void toggleDebugVisualization() {
        debugVisualizationEnabled = !debugVisualizationEnabled;
    }

    /**
     * @brief Draws debug information to the console.
     */
    void debugDraw() const {
        if (debugVisualizationEnabled) {
            for (const auto& body : bodies) {
                std::cout << "Body at position: (" << body->position.x << ", " << body->position.y << ")\n";
                std::cout << "Velocity: (" << body->velocity.x << ", " << body->velocity.y << ")\n";
                std::cout << "Force: (" << body->force.x << ", " << body->force.y << ")\n";
            }
        }
    }

private:
    /**
     * @brief Checks for a collision between two AABBs.
     * @param a The first body.
     * @param b The second body.
     * @param widthA The width of the first body.
     * @param heightA The height of the first body.
     * @param widthB The width of the second body.
     * @param heightB The height of the second body.
     * @return True if the bodies are colliding, false otherwise.
     */
    bool checkAABBCollision(const RigidBody& a, const RigidBody& b, float widthA, float heightA, float widthB, float heightB) const {
        return a.position.x < b.position.x + widthB &&
               a.position.x + widthA > b.position.x &&
               a.position.y < b.position.y + heightB &&
               a.position.y + heightA > b.position.y;
    }

    /**
     * @brief Checks for a collision between two circles.
     * @param a The first body.
     * @param b The second body.
     * @param radiusA The radius of the first body.
     * @param radiusB The radius of the second body.
     * @return True if the bodies are colliding, false otherwise.
     */
    bool checkCircleCollision(const RigidBody& a, const RigidBody& b, float radiusA, float radiusB) const {
        float dx = a.position.x - b.position.x;
        float dy = a.position.y - b.position.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        return distance < (radiusA + radiusB);
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
