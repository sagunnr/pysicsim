#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>

namespace physx {
    struct Config {
        static constexpr float gravity = 9.8f;
        static constexpr float dt = 1.0f / 60.0f;
        static constexpr float friction = 0.5f;
    };

    struct Vec2D {
        float x{}, y{};
        Vec2D() = default;
        Vec2D(float x_, float y_) : x(x_), y(y_) {}

        Vec2D operator+(const Vec2D& rhs) const { return {x + rhs.x, y + rhs.y}; }
        Vec2D operator-(const Vec2D& rhs) const { return {x - rhs.x, y - rhs.y}; }
        Vec2D operator*(float s) const { return {x * s, y * s}; }
        Vec2D operator/(float s) const { return {x / s, y / s}; }

        float dot(const Vec2D& o) const { return x * o.x + y * o.y; }
        float mag() const { return std::sqrt(x * x + y * y); }
        Vec2D norm() const {
            float len = mag();
            return len > 0 ? (*this) / len : Vec2D{0, 0};
        }
    };

    struct Particle {
        Vec2D pos, vel, accum;
        float mass;
        bool immobile;

        Particle(float px, float py, float m = 1.0f, bool staticFlag = false)
            : pos(px, py), mass(m), immobile(staticFlag) {}

        void accumulate(const Vec2D& f) {
            accum = accum + f;
        }

        void integrate() {
            if (immobile) return;
            Vec2D accel = accum / mass;
            vel = vel + accel * Config::dt;
            pos = pos + vel * Config::dt;
            accum = {0, 0};
        }
    };

    class SimulationZone {
        std::vector<std::shared_ptr<Particle>> units;
        bool visualDebug = false;

        bool aabbOverlap(const Particle& a, const Particle& b, float wa, float ha, float wb, float hb) const {
            return (a.pos.x < b.pos.x + wb && a.pos.x + wa > b.pos.x &&
                    a.pos.y < b.pos.y + hb && a.pos.y + ha > b.pos.y);
        }

        bool circleOverlap(const Particle& a, const Particle& b, float ra, float rb) const {
            Vec2D delta = a.pos - b.pos;
            return delta.dot(delta) < (ra + rb) * (ra + rb);
        }

        void impulseExchange(Particle& a, Particle& b) {
            Vec2D n = b.pos - a.pos;
            n = n.norm();
            Vec2D relVel = b.vel - a.vel;
            float sepVel = relVel.dot(n);
            if (sepVel > 0) return;

            constexpr float restitution = 1.0f;
            float j = -(1 + restitution) * sepVel / (1 / a.mass + 1 / b.mass);
            Vec2D impulse = n * j;

            a.vel = a.vel - impulse / a.mass;
            b.vel = b.vel + impulse / b.mass;
        }

        void frictionForce(Particle& p, const Vec2D& normal) {
            if (p.immobile) return;
            float fMag = Config::friction * p.mass * Config::gravity;
            Vec2D f = normal * (-fMag);
            p.accumulate(f);
        }

    public:
        void spawn(const std::shared_ptr<Particle>& p) {
            units.emplace_back(p);
        }

        void applyPhysics() {
            for (auto& p : units) {
                if (!p->immobile) {
                    p->accumulate({0, Config::gravity * p->mass});
                }
                p->integrate();
            }
        }

        void handleContacts() {
            for (size_t i = 0; i < units.size(); ++i) {
                for (size_t j = i + 1; j < units.size(); ++j) {
                    auto& A = *units[i];
                    auto& B = *units[j];

                    if (aabbOverlap(A, B, 50, 50, 50, 50)) {
                        impulseExchange(A, B);
                        frictionForce(A, {1, 0});
                        frictionForce(B, {1, 0});
                    }

                    if (circleOverlap(A, B, 25, 25)) {
                        impulseExchange(A, B);
                        frictionForce(A, {1, 0});
                        frictionForce(B, {1, 0});
                    }
                }
            }
        }

        void toggleViz() { visualDebug = !visualDebug; }

        void inspect() const {
            if (!visualDebug) return;
            for (const auto& p : units) {
                std::cout << "[Pos] (" << p->pos.x << ", " << p->pos.y << ") ";
                std::cout << "[Vel] (" << p->vel.x << ", " << p->vel.y << ") ";
                std::cout << "[Frc] (" << p->accum.x << ", " << p->accum.y << ")\n";
            }
        }

        void tick() {
            applyPhysics();
            handleContacts();
        }
    };
}

int main() {
    using namespace physx;

    SimulationZone sim;
    auto wall = std::make_shared<Particle>(200, 200, 0.0f, true);
    auto ball = std::make_shared<Particle>(100, 100, 1.0f);
    ball->vel = {50, 0};

    sim.spawn(wall);
    sim.spawn(ball);

    for (int f = 0; f < 600; ++f) {
        sim.tick();
        if (f % 60 == 0) sim.toggleViz();
        sim.inspect();
    }

    return 0;
}
