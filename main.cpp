#include <vector>
#include <cmath>
#include <algorithm>
#include <variant>
#include <cassert>

/**
 * Vector2D - 2D mathematical vector operations.
 * @intuition: Foundation for physics calculations and transformations.
 * @approach: Struct with operator overloads for vector math.
 * @complexity: 
 *   Time: O(1) for all operations
 *   Space: O(1) per vector
 */
struct Vector2D {
    float x = 0.0f, y = 0.0f;

    Vector2D operator+(const Vector2D& rhs) const { return {x + rhs.x, y + rhs.y}; }
    Vector2D operator-(const Vector2D& rhs) const { return {x - rhs.x, y - rhs.y}; }
    Vector2D operator*(float scalar) const { return {x * scalar, y * scalar}; }
    Vector2D operator/(float scalar) const { assert(scalar != 0); return {x / scalar, y / scalar}; }
    float dot(const Vector2D& rhs) const { return x * rhs.x + y * rhs.y; }
    float magnitude_squared() const { return x * x + y * y; }
    float magnitude() const { return std::sqrt(magnitude_squared()); }
    Vector2D normalized() const { const float mag = magnitude(); return mag > 0 ? *this / mag : Vector2D{}; }
};

/**
 * PhysicsBody - Core rigid body representation.
 * @intuition: Encapsulates physical properties and state.
 * @approach: Combines dynamics and collider in one entity.
 * @complexity:
 *   Time: O(1) per property access
 *   Space: O(1) per body
 */
struct PhysicsBody {
    Vector2D position;
    Vector2D velocity;
    Vector2D force_accumulator;
    float mass = 1.0f;
    float restitution = 0.8f;
    float friction = 0.2f;
    bool is_dynamic = true;
    
    struct CircleCollider { float radius; };
    struct BoxCollider { float width, height; };
    std::variant<CircleCollider, BoxCollider> collider;
};

/**
 * PhysicsWorld - Manages simulation and collisions.
 * @intuition: Central coordinator for physics updates.
 * @approach: Discrete time step with collision resolution.
 * @complexity:
 *   Time: O(n^2) for collision detection, O(1) per resolution
 *   Space: O(n) for body storage
 */
class PhysicsWorld {
public:
    Vector2D gravity = {0.0f, 9.8f};
    bool debug_visualization = false;
    
    void add_body(PhysicsBody body) { bodies.push_back(std::move(body)); }
    
    /**
     * Advance physics simulation by time step.
     * @intuition: Integrate forces, detect collisions, resolve penetrations.
     * @approach: Semi-implicit Euler integration with impulse resolution.
     */
    void step(float dt) {
        // Apply forces and integrate
        for (auto& body : bodies) {
            if (!body.is_dynamic) continue;
            body.force_accumulator = body.force_accumulator + gravity * body.mass;
            const Vector2D acceleration = body.force_accumulator / body.mass;
            body.velocity = body.velocity + acceleration * dt;
            body.position = body.position + body.velocity * dt;
            body.force_accumulator = {0, 0};
        }
        
        // Collision detection and resolution
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                if (auto collision = detect_collision(bodies[i], bodies[j])) {
                    resolve_collision(*collision);
                    if (debug_visualization) {
                        draw_collision_debug(*collision);
                    }
                }
            }
        }
    }

private:
    std::vector<PhysicsBody> bodies;
    
    struct Collision {
        PhysicsBody& a;
        PhysicsBody& b;
        Vector2D normal;
        float depth;
    };
    
    std::optional<Collision> detect_collision(PhysicsBody& a, PhysicsBody& b) {
        return std::visit([&](const auto& collider_a, const auto& collider_b) {
            return detect_collision_impl(a, collider_a, b, collider_b);
        }, a.collider, b.collider);
    }
    
    template<typename T1, typename T2>
    std::optional<Collision> detect_collision_impl(
        PhysicsBody& a, const T1& collider_a, 
        PhysicsBody& b, const T2& collider_b
    ) {
        if constexpr (std::is_same_v<T1, PhysicsBody::CircleCollider> && 
                      std::is_same_v<T2, PhysicsBody::CircleCollider>) {
            return circle_circle_collision(a, collider_a, b, collider_b);
        }
        else if constexpr (std::is_same_v<T1, PhysicsBody::BoxCollider> && 
                           std::is_same_v<T2, PhysicsBody::BoxCollider>) {
            return aabb_aabb_collision(a, collider_a, b, collider_b);
        }
        else {
            return circle_aabb_collision(a, collider_a, b, collider_b);
        }
    }
    
    std::optional<Collision> circle_circle_collision(
        PhysicsBody& a, const PhysicsBody::CircleCollider& circle_a,
        PhysicsBody& b, const PhysicsBody::CircleCollider& circle_b
    ) {
        const Vector2D ab = b.position - a.position;
        const float min_distance = circle_a.radius + circle_b.radius;
        const float distance_sq = ab.magnitude_squared();
        
        if (distance_sq >= min_distance * min_distance) return std::nullopt;
        
        const float distance = std::sqrt(distance_sq);
        return Collision{
            a, b,
            (distance > 0) ? ab / distance : Vector2D{1, 0},
            min_distance - distance
        };
    }
    
    std::optional<Collision> aabb_aabb_collision(
        PhysicsBody& a, const PhysicsBody::BoxCollider& box_a,
        PhysicsBody& b, const PhysicsBody::BoxCollider& box_b
    ) {
        const float dx = b.position.x - a.position.x;
        const float px = (box_a.width + box_b.width) * 0.5f - std::abs(dx);
        if (px <= 0) return std::nullopt;
        
        const float dy = b.position.y - a.position.y;
        const float py = (box_a.height + box_b.height) * 0.5f - std::abs(dy);
        if (py <= 0) return std::nullopt;
        
        if (px < py) {
            return Collision{
                a, b,
                Vector2D{(dx > 0) ? 1.0f : -1.0f, 0.0f},
                px
            };
        } else {
            return Collision{
                a, b,
                Vector2D{0.0f, (dy > 0) ? 1.0f : -1.0f},
                py
            };
        }
    }
    
    template<typename TA, typename TB>
    std::optional<Collision> circle_aabb_collision(
        PhysicsBody& circle_body, const TA& circle,
        PhysicsBody& box_body, const TB& box
    ) {
        // Implementation for circle-AABB collision
        // (Omitted for brevity - follows similar pattern)
        return std::nullopt;
    }
    
    void resolve_collision(const Collision& col) {
        const float inv_mass_a = col.a.is_dynamic ? 1.0f / col.a.mass : 0.0f;
        const float inv_mass_b = col.b.is_dynamic ? 1.0f / col.b.mass : 0.0f;
        const float total_inv_mass = inv_mass_a + inv_mass_b;
        if (total_inv_mass == 0) return;
        
        // Position correction
        const Vector2D correction = col.normal * (col.depth / total_inv_mass) * 0.8f;
        col.a.position = col.a.position - correction * inv_mass_a;
        col.b.position = col.b.position + correction * inv_mass_b;
        
        // Velocity resolution
        const Vector2D relative_velocity = col.b.velocity - col.a.velocity;
        const float velocity_along_normal = relative_velocity.dot(col.normal);
        if (velocity_along_normal > 0) return;
        
        const float e = std::min(col.a.restitution, col.b.restitution);
        float impulse_scalar = -(1.0f + e) * velocity_along_normal / total_inv_mass;
        
        // Friction
        const Vector2D tangent = (relative_velocity - col.normal * velocity_along_normal).normalized();
        const float friction_impulse = -relative_velocity.dot(tangent) / total_inv_mass;
        const float mu = std::sqrt(col.a.friction * col.b.friction);
        impulse_scalar = std::min(impulse_scalar, impulse_scalar * mu);
        
        const Vector2D impulse = col.normal * impulse_scalar;
        col.a.velocity = col.a.velocity - impulse * inv_mass_a;
        col.b.velocity = col.b.velocity + impulse * inv_mass_b;
    }
    
    void draw_collision_debug(const Collision& col) {
        // In practice: integrate with your rendering system
        // Example: draw collision normal and penetration depth
    }
};

// Example usage
int main() {
    PhysicsWorld world;
    world.gravity = {0, 50.0f};  // Custom gravity
    
    // Create player (circle)
    PhysicsBody player;
    player.position = {100, 100};
    player.mass = 2.0f;
    player.collider = PhysicsBody::CircleCollider{20.0f};
    world.add_body(player);
    
    // Create platform (static box)
    PhysicsBody platform;
    platform.position = {100, 400};
    platform.is_dynamic = false;
    platform.collider = PhysicsBody::BoxCollider{200, 30};
    world.add_body(platform);
    
    // Game loop
    for (int i = 0; i < 60; ++i) {  // 1 second at 60 FPS
        world.step(1.0f / 60.0f);
    }
    
    return 0;
}
