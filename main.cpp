#include <array>
#include <vector>
#include <memory>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <functional>
#include <map>
#include <string>
#include <random>
#include <type_traits> // for std::to_underlying

namespace PhysicsSim {

    // --- Coordinate System ---
    template<int Dim, typename T = double>
    class Vec {
        std::array<T, Dim> values_;

    public:
        template<typename... Args>
        explicit Vec(Args... args) : values_{ static_cast<T>(args)... } {}

        Vec() : values_{} {}

        T& operator[](size_t i) { return values_[i]; }
        const T& operator[](size_t i) const { return values_[i]; }

        template<typename Op>
        Vec apply(const Vec& rhs, Op op) const {
            Vec result;
            for (size_t i = 0; i < Dim; ++i) {
                result[i] = op(values_[i], rhs[i]);
            }
            return result;
        }

        T norm() const {
            T sum = 0;
            for (auto v : values_)
                sum += v * v;
            return std::sqrt(sum);
        }

        Vec normalized() const {
            if (auto n = norm(); n > T(1e-10)) {
                Vec result;
                for (size_t i = 0; i < Dim; ++i)
                    result[i] = values_[i] / n;
                return result;
            }
            return Vec{};
        }
    };

    using Vec2f = Vec<2, float>;

    // --- Geometry ---
    enum class GeometryType : uint8_t {
        Quadrilateral = 0xAA,
        Spheroid = 0xBB,
        Undefined = 0xFF
    };

    template<GeometryType T>
    struct GeometryTraits;

    template<>
    struct GeometryTraits<GeometryType::Quadrilateral> {
        using ParamType = std::pair<float, float>;
        static constexpr const char* Name = "Quadrilateral";
    };

    template<>
    struct GeometryTraits<GeometryType::Spheroid> {
        using ParamType = float;
        static constexpr const char* Name = "Spheroid";
    };

    class GeometryBase {
    public:
        GeometryType type;

        virtual ~GeometryBase() = default;
        virtual std::pair<Vec2f, Vec2f> boundingBox(const Vec2f& center) const = 0;
        virtual std::string name() const = 0;
    };

    template<GeometryType T>
    class Geometry : public GeometryBase {
        typename GeometryTraits<T>::ParamType params_;

    public:
        explicit Geometry(const typename GeometryTraits<T>::ParamType& params)
            : params_(params) {
            type = T;
        }

        const auto& params() const { return params_; }

        std::pair<Vec2f, Vec2f> boundingBox(const Vec2f& center) const override {
            if constexpr (T == GeometryType::Quadrilateral) {
                auto [w, h] = params_;
                Vec2f halfExtents{ w / 2, h / 2 };
                return {
                    center.apply(halfExtents, std::minus<float>()),
                    center.apply(halfExtents, std::plus<float>())
                };
            }
            else if constexpr (T == GeometryType::Spheroid) {
                float r = params_;
                Vec2f extents{ r, r };
                return {
                    center.apply(extents, std::minus<float>()),
                    center.apply(extents, std::plus<float>())
                };
            }
            return { Vec2f{0, 0}, Vec2f{0, 0} };
        }

        std::string name() const override {
            return GeometryTraits<T>::Name;
        }
    };

    // --- Entity States ---
    enum class EntityState { Dormant, Kinetic, Static, Transitioning };

    // --- Physical Entity ---
    class Entity {
        std::map<std::string, float, std::less<>> properties_;
        std::unique_ptr<GeometryBase> geometry_;

        // Use in-class initializer; remove redundant constructor init for state_
        EntityState state_ = EntityState::Dormant;

        Vec2f position_;

        Vec2f momentum_{0, 0};
        Vec2f acceleration_{0, 0};
        Vec2f externalForce_{0, 0};

    public:
        static std::mt19937& rng() {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            return gen;
        }

        Entity(const Vec2f& position,
               std::unique_ptr<GeometryBase> geometry,
               std::map<std::string, float, std::less<>> props = {})
            : position_(position), geometry_(std::move(geometry)), properties_(std::move(props)) 
        {
            std::uniform_int_distribution<int> dist(0, 3);
            state_ = static_cast<EntityState>(dist(rng()));

            if (properties_.empty()) {
                properties_ = {
                    {"mass", 1.0f},
                    {"bounce", 0.8f},
                    {"roughness", 0.3f},
                    {"kinetic", state_ != EntityState::Static ? 1.0f : 0.0f}
                };
            }
        }

        float getProperty(const std::string& key) const {
            auto it = properties_.find(key);
            return it != properties_.end() ? it->second : 0.0f;
        }

        void setProperty(const std::string& key, float value) {
            properties_[key] = value;
        }

        const Vec2f& getPosition() const { return position_; }
        const Vec2f& getMomentum() const { return momentum_; }
        GeometryBase* getGeometry() const { return geometry_.get(); }
        EntityState getState() const { return state_; }

        void setPosition(const Vec2f& pos) { position_ = pos; }
        void setMomentum(const Vec2f& mom) { momentum_ = mom; }

        void addForce(const Vec2f& force) {
            if (state_ != EntityState::Static)
                externalForce_ = externalForce_.apply(force, std::plus<float>());
        }

        void integrate(float dt) {
            if (state_ == EntityState::Static) return;

            float mass = getProperty("mass");
            float invMass = mass > 1e-6f ? 1.0f / mass : 0.0f;

            acceleration_ = Vec2f{externalForce_[0] * invMass, externalForce_[1] * invMass};

            auto dv = Vec2f{acceleration_[0] * dt, acceleration_[1] * dt};
            momentum_ = momentum_.apply(dv, std::plus<float>());

            auto dx = Vec2f{momentum_[0] * dt, momentum_[1] * dt};
            position_ = position_.apply(dx, std::plus<float>());

            externalForce_ = Vec2f{0, 0};
        }

        std::pair<Vec2f, Vec2f> getBounds() const {
            return geometry_->boundingBox(position_);
        }
    };

    // --- Interaction Metadata ---
    struct Interaction {
        bool contact = false;
        Vec2f separationAxis;
        float penetration = 0.0f;
        Entity* entityA = nullptr;
        Entity* entityB = nullptr;
        Vec2f contactPoint;
        float relativeVelocity = 0.0f;
        std::string type = "Undefined";
        std::chrono::high_resolution_clock::time_point detectionTime{}; // in-class init

        Interaction() = default;
    };

    // --- Helper functions ---
    inline bool separatedOnAxis(float minA, float maxA, float minB, float maxB) {
        return (maxA < minB) || (minA > maxB);
    }

    inline float calculateOverlap(float minA, float maxA, float minB, float maxB) {
        return std::min(maxA - minB, maxB - minA);
    }

    // --- Interaction Detection ---
    class InteractionDetector {
        using DetectFn = std::function<Interaction(Entity*, Entity*)>;
        static std::map<std::pair<GeometryType, GeometryType>, DetectFn> registry_;

        static Interaction detectQuadQuad(Entity* a, Entity* b) {
            Interaction interaction;
            interaction.entityA = a;
            interaction.entityB = b;
            interaction.type = "Quad-Quad";

            auto [minA, maxA] = a->getBounds();
            auto [minB, maxB] = b->getBounds();

            if (separatedOnAxis(minA[0], maxA[0], minB[0], maxB[0])) return interaction;
            if (separatedOnAxis(minA[1], maxA[1], minB[1], maxB[1])) return interaction;

            interaction.contact = true;

            float overlapX = calculateOverlap(minA[0], maxA[0], minB[0], maxB[0]);
            float overlapY = calculateOverlap(minA[1], maxA[1], minB[1], maxB[1]);

            if (overlapX <= overlapY) {
                interaction.penetration = overlapX;
                float dir = (a->getPosition()[0] < b->getPosition()[0]) ? -1.0f : 1.0f;
                interaction.separationAxis = Vec2f{ dir, 0 };
            }
            else {
                interaction.penetration = overlapY;
                float dir = (a->getPosition()[1] < b->getPosition()[1]) ? -1.0f : 1.0f;
                interaction.separationAxis = Vec2f{ 0, dir };
            }

            return interaction;
        }

        static Interaction detectSphereSphere(Entity* a, Entity* b) {
            Interaction interaction;
            interaction.entityA = a;
            interaction.entityB = b;
            interaction.type = "Spheroid-Spheroid";

            auto* sphA = static_cast<Geometry<GeometryType::Spheroid>*>(a->getGeometry());
            auto* sphB = static_cast<Geometry<GeometryType::Spheroid>*>(b->getGeometry());

            float rA = sphA->params();
            float rB = sphB->params();

            Vec2f delta = b->getPosition().apply(a->getPosition(), std::minus<float>());
            float dist = delta.norm();
            float rSum = rA + rB;

            if (dist >= rSum) return interaction;

            interaction.contact = true;
            interaction.penetration = rSum - dist;

            if (dist > 1e-6f) {
                interaction.separationAxis = delta.normalized();
            }
            else {
                interaction.separationAxis = Vec2f{1, 0};
            }

            return interaction;
        }

        static Interaction detectQuadSphere(Entity* quad, Entity* sph) {
            Interaction interaction;
            interaction.entityA = quad;
            interaction.entityB = sph;
            interaction.type = "Quad-Spheroid";

            auto* sphGeom = static_cast<Geometry<GeometryType::Spheroid>*>(sph->getGeometry());
            float radius = sphGeom->params();

            auto [minQ, maxQ] = quad->getBounds();
            Vec2f centerS = sph->getPosition();

            auto clamp = [](float v, float min, float max) {
                return std::max(min, std::min(v, max));
            };

            Vec2f closestPoint{
                clamp(centerS[0], minQ[0], maxQ[0]),
                clamp(centerS[1], minQ[1], maxQ[1])
            };

            Vec2f separation = centerS.apply(closestPoint, std::minus<float>());
            float dist = separation.norm();

            if (dist >= radius) return interaction;

            interaction.contact = true;
            interaction.penetration = radius - dist;

            if (dist > 1e-6f) {
                interaction.separationAxis = separation.normalized();
            }
            else {
                Vec2f centerQ = quad->getPosition();
                Vec2f delta = centerQ.apply(centerS, std::minus<float>());
                if (std::abs(delta[0]) > std::abs(delta[1])) {
                    interaction.separationAxis = Vec2f{delta[0] > 0 ? 1.f : -1.f, 0};
                }
                else {
                    interaction.separationAxis = Vec2f{0, delta[1] > 0 ? 1.f : -1.f};
                }
            }

            return interaction;
        }

    public:
        static Interaction detect(Entity* a, Entity* b) {
            auto geomA = a->getGeometry()->type;
            auto geomB = b->getGeometry()->type;

            auto key = std::make_pair(geomA, geomB);
            auto revKey = std::make_pair(geomB, geomA);

            if (auto it = registry_.find(key); it != registry_.end()) {
                return it->second(a, b);
            }

            if (auto it = registry_.find(revKey); it != registry_.end()) {
                Interaction res = it->second(b, a);
                res.separationAxis = Vec2f{ -res.separationAxis[0], -res.separationAxis[1] };
                return res;
            }

            return {};
        }

        static void initialize() {
            registry_[{GeometryType::Quadrilateral, GeometryType::Quadrilateral}] = detectQuadQuad;
            registry_[{GeometryType::Spheroid, GeometryType::Spheroid}] = detectSphereSphere;
            registry_[{GeometryType::Quadrilateral, GeometryType::Spheroid}] = detectQuadSphere;
        }
    };

    std::map<std::pair<GeometryType, GeometryType>, InteractionDetector::DetectFn> InteractionDetector::registry_;

    // --- Interaction Resolution Strategies ---
    class IResolutionStrategy {
    public:
        virtual ~IResolutionStrategy() = default;
        virtual void resolve(const Interaction& interaction) = 0;
    };

    class PositionCorrection : public IResolutionStrategy {
    public:
        void resolve(const Interaction& interaction) override {
            if (!interaction.contact) return;

            constexpr float adjustRatio = 0.85f;
            constexpr float tolerance = 0.02f;

            float mA = interaction.entityA->getProperty("mass");
            float mB = interaction.entityB->getProperty("mass");
            float invMassA = mA > 1e-6f ? 1.0f / mA : 0.0f;
            float invMassB = mB > 1e-6f ? 1.0f / mB : 0.0f;

            float corrMag = std::max(interaction.penetration - tolerance, 0.0f) / (invMassA + invMassB) * adjustRatio;

            Vec2f correctionVec{interaction.separationAxis[0] * corrMag, interaction.separationAxis[1] * corrMag};

            Vec2f adjA{correctionVec[0] * invMassA, correctionVec[1] * invMassA};
            Vec2f adjB{correctionVec[0] * invMassB, correctionVec[1] * invMassB};

            Vec2f newPosA = interaction.entityA->getPosition().apply(adjA, std::minus<float>());
            Vec2f newPosB = interaction.entityB->getPosition().apply(adjB, std::plus<float>());

            interaction.entityA->setPosition(newPosA);
            interaction.entityB->setPosition(newPosB);
        }
    };

    class MomentumExchange : public IResolutionStrategy {
    public:
        void resolve(const Interaction& interaction) override {
            if (!interaction.contact) return;

            Vec2f velocityA = interaction.entityA->getMomentum();
            Vec2f velocityB = interaction.entityB->getMomentum();

            Vec2f relativeV = velocityB.apply(velocityA, std::minus<float>());
            float velAlongN = relativeV[0] * interaction.separationAxis[0] + relativeV[1] * interaction.separationAxis[1];

            if (velAlongN > 0) return; // Separating

            float bounce = std::min(
                interaction.entityA->getProperty("bounce"),
                interaction.entityB->getProperty("bounce"));

            float invMassA = 1.0f / interaction.entityA->getProperty("mass");
            float invMassB = 1.0f / interaction.entityB->getProperty("mass");

            float impulse = -(1 + bounce) * velAlongN / (invMassA + invMassB);

            Vec2f impulseVec{interaction.separationAxis[0] * impulse, interaction.separationAxis[1] * impulse};

            Vec2f impulseA{impulseVec[0] * invMassA, impulseVec[1] * invMassA};
            Vec2f impulseB{impulseVec[0] * invMassB, impulseVec[1] * invMassB};

            Vec2f newVelA = velocityA.apply(impulseA, std::minus<float>());
            Vec2f newVelB = velocityB.apply(impulseB, std::plus<float>());

            interaction.entityA->setMomentum(newVelA);
            interaction.entityB->setMomentum(newVelB);
        }
    };

    // --- Diagnostic Visualization ---
    class Diagnostic {
        static bool active_;
        static std::vector<std::string> history_;
        static std::chrono::high_resolution_clock::time_point lastToggle_;

    public:
        static void toggle() {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<float>(now - lastToggle_).count();

            if (elapsed > 0.5f) {
                active_ = !active_;
                lastToggle_ = now;

                std::string status = active_ ? "Diagnostic ACTIVE" : "Diagnostic INACTIVE";

                history_.push_back(status);
                std::cout << status << '\n';

                if (history_.size() > 100)
                    history_.erase(history_.begin());
            }
        }

        static void renderEntity(const Entity& e) {
            if (!active_) return;

            static constexpr std::array<const char*, 4> states = {"Dormant", "Kinetic", "Static", "Transitioning"};

            GeometryBase* geom = e.getGeometry();

            if (geom->type == GeometryType::Quadrilateral) {
                auto [min, max] = e.getBounds();
                std::cout << "Quadrilateral[" << states[static_cast<size_t>(std::to_underlying(e.getState()))] << "]: bounds ["
                          << min[0] << ", " << min[1] << "] to [" << max[0] << ", " << max[1] << "]\n";
            } else if (geom->type == GeometryType::Spheroid) {
                auto* sph = static_cast<Geometry<GeometryType::Spheroid>*>(geom);
                std::cout << "Spheroid[" << states[static_cast<size_t>(std::to_underlying(e.getState()))] << "]: center ["
                          << e.getPosition()[0] << ", " << e.getPosition()[1] << "] radius ["
                          << sph->params() << "]\n";
            }
        }

        static void renderInteraction(const Interaction& interaction) {
            if (!active_ || !interaction.contact) return;

            auto age = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - interaction.detectionTime).count();

            std::cout << "Interaction[" << interaction.type << "]: "
                      << "penetration [" << interaction.penetration << "] "
                      << "axis [" << interaction.separationAxis[0] << ", " << interaction.separationAxis[1] << "] "
                      << "age [" << age * 1000 << " ms]\n";
        }

        static void renderKineticField(const Entity& e) {
            if (!active_) return;

            auto pos = e.getPosition();
            auto mom = e.getMomentum();
            float m = e.getProperty("mass");

            float kineticEnergy = 0.5f * m * (mom[0] * mom[0] + mom[1] * mom[1]);

            if (kineticEnergy > 0.1f) {
                std::cout << "Influence at [" << pos[0] << "," << pos[1] << "]: KE="
                          << kineticEnergy << ", momentum=[" << mom[0] << "," << mom[1] << "]\n";
            }
        }

        static void dumpHistory() {
            if (!active_) return;

            std::cout << "\n=== Diagnostic History ===\n";
            for (const auto& e : history_)
                std::cout << e << '\n';
            std::cout << "=========================\n\n";
        }
    };

    bool Diagnostic::active_ = false;
    std::vector<std::string> Diagnostic::history_;
    std::chrono::high_resolution_clock::time_point Diagnostic::lastToggle_ = std::chrono::high_resolution_clock::now();

    // --- Simulation Engine ---
    class Simulation {
        std::vector<std::unique_ptr<Entity>> entities_;
        Vec2f gravity_{0, -975.f};
        float timestep_ = 1.0f / 60.0f;
        std::unique_ptr<PositionCorrection> posRes_ = std::make_unique<PositionCorrection>();
        std::unique_ptr<MomentumExchange> momRes_ = std::make_unique<MomentumExchange>();
        std::vector<Interaction> interactions_;
        size_t cycleCount_ = 0;

    public:
        Simulation() {
            InteractionDetector::initialize();
        }

        void addEntity(std::unique_ptr<Entity> e) {
            entities_.push_back(std::move(e));
        }

        void setGravity(const Vec2f& g) { gravity_ = g; }

        void runCycle() {
            ++cycleCount_;

            for (auto& e : entities_) {
                if (e->getState() == EntityState::Static)
                    continue;
                float m = e->getProperty("mass");
                Vec2f gravForce{gravity_[0] * m, gravity_[1] * m};
                e->addForce(gravForce);
            }

            for (const auto& e : entities_)
                Diagnostic::renderKineticField(*e);

            for (auto& e : entities_)
                e->integrate(timestep_);

            interactions_.clear();
            const auto n = entities_.size();
            for (size_t i = 0; i < n; ++i) {
                for (size_t j = i + 1; j < n; ++j) {
                    auto interaction = InteractionDetector::detect(entities_[i].get(), entities_[j].get());
                    if (!interaction.contact) 
                        continue;

                    interactions_.push_back(interaction);
                    Diagnostic::renderInteraction(interaction);

                    posRes_->resolve(interaction);
                    momRes_->resolve(interaction);
                }
            }

            for (const auto& e : entities_)
                Diagnostic::renderEntity(*e);

            if (cycleCount_ % 300 == 0)
                Diagnostic::dumpHistory();
        }

        size_t entityCount() const { return entities_.size(); }
        Entity* entity(size_t idx) { return idx < entities_.size() ? entities_[idx].get() : nullptr; }
        size_t interactionCount() const { return interactions_.size(); }
    };

    // --- Main Simulator ---
    class ArcadePhysicsSimulator {
        std::unique_ptr<Simulation> sim_;  
        std::chrono::high_resolution_clock::time_point lastFrame_ = std::chrono::high_resolution_clock::now();
        enum class State { Init, Running, Paused, Terminate };
        State state_ = State::Init;

        void handleCommand(const std::string& cmd) {
            if (cmd == "diagnostic") {
                Diagnostic::toggle();
            } else if (cmd == "terminate") {
                state_ = State::Terminate;
            } else if (cmd == "pause") {
                if (state_ == State::Running) {
                    state_ = State::Paused;
                    std::cout << "Simulation paused.\n";
                } else if (state_ == State::Paused) {
                    state_ = State::Running;
                    std::cout << "Simulation resumed.\n";
                }
            }
        }

    public:
        ArcadePhysicsSimulator()
            : state_(State::Init) {
            sim_ = std::make_unique<Simulation>();
            setupScene();
            state_ = State::Running;
        }

        void setupScene() {
            // Ground
            sim_->addEntity(std::make_unique<Entity>(
                Vec2f{400, 50},
                std::make_unique<Geometry<GeometryType::Quadrilateral>>(std::pair{800.f, 100.f}),
                std::map<std::string, float, std::less<>>{
                    {"mass", 1.f},
                    {"bounce", 0.2f},
                    {"roughness", 0.9f},
                    {"kinetic", 0.f}
            }));

            // Walls
            sim_->addEntity(std::make_unique<Entity>(
                Vec2f{50, 300},
                std::make_unique<Geometry<GeometryType::Quadrilateral>>(std::pair{100.f, 600.f}),
                std::map<std::string, float, std::less<>>{{"mass", 1.f}, {"bounce", 0.1f}, {"roughness", 0.8f}, {"kinetic", 0.f}}));

            sim_->addEntity(std::make_unique<Entity>(
                Vec2f{750, 300},
                std::make_unique<Geometry<GeometryType::Quadrilateral>>(std::pair{100.f, 600.f}),
                std::map<std::string, float, std::less<>>{{"mass", 1.f}, {"bounce", 0.1f}, {"roughness", 0.8f}, {"kinetic", 0.f}}));

            // Dynamic Cube
            sim_->addEntity(std::make_unique<Entity>(
                Vec2f{200, 400},
                std::make_unique<Geometry<GeometryType::Quadrilateral>>(std::pair{40.f, 40.f}),
                std::map<std::string, float, std::less<>>{{"mass", 2.5f}, {"bounce", 0.65f}, {"roughness", 0.4f}, {"kinetic", 1.f}}));

            // Dynamic Spheroid
            auto spheroid = std::make_unique<Entity>(
                Vec2f{600, 500},
                std::make_unique<Geometry<GeometryType::Spheroid>>(20.f),
                std::map<std::string, float, std::less<>>{{"mass", 1.8f}, {"bounce", 0.92f}, {"roughness", 0.2f}, {"kinetic", 1.f}});
            spheroid->setMomentum(Vec2f{-175.f, 135.f});
            sim_->addEntity(std::move(spheroid));
        }

        void runMainLoop() {
            std::cout << "Arcade Physics Simulator (60Hz). Commands: 'diagnostic', 'terminate', 'pause'\n";
            size_t frameCount = 0;

            while (state_ != State::Terminate) {
                auto now = std::chrono::high_resolution_clock::now();
                float delta = std::chrono::duration<float>(now - lastFrame_).count();

                if (delta < 1.0f / 60 || state_ != State::Running) {
                    // Not time to run frame or paused
                    if (std::cin.rdbuf()->in_avail() > 0) {
                        std::string cmd;
                        std::cin >> cmd;
                        handleCommand(cmd);
                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(500));
                    continue;
                }

                sim_->runCycle();
                lastFrame_ = now;
                ++frameCount;

                if (frameCount % 60 == 0) {
                    std::cout << "\n=== Cycle " << frameCount
                              << " | Entities: " << sim_->entityCount()
                              << " | Interactions: " << sim_->interactionCount()
                              << " ===\n";
                }

                if (std::cin.rdbuf()->in_avail() > 0) {
                    std::string cmd;
                    std::cin >> cmd;
                    handleCommand(cmd);
                }

                std::this_thread::sleep_for(std::chrono::microseconds(500));
            }
        }
    };

} // namespace PhysicsSim

int main() {
    PhysicsSim::ArcadePhysicsSimulator simulator;
    simulator.runMainLoop();
    return 0;
}
