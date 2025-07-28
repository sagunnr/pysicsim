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
        
        // FIX: Remove redundant constructor init for state_, rely on in-class initializer
        EntityState state_ = EntityState::Dormant; 

        Vec2f position_;

        // In-class member initialization, Sonar fix
        Vec2f momentum_{0, 0};
        Vec2f acceleration_{0, 0};
        Vec2f externalForce_{0, 0};

        static std::random_device rd_;
        static std::mt19937 gen_;

    public:
        Entity(const Vec2f& position,
               std::unique_ptr<GeometryBase> geometry,
               std::map<std::string, float, std::less<>> props = {})
            : position_(position), geometry_(std::move(geometry)), properties_(std::move(props)) 
        {
            // Safe usage of static RNG
            std::uniform_int_distribution<int> dist(0, 3);
            state_ = static_cast<EntityState>(dist(gen_));

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

            auto dv = Vec2f{acceleration_[0] * dt, acceleration_[1] * dt};   // FIX: replaced explicit type with auto
            momentum_ = momentum_.apply(dv, std::plus<float>());

            auto dx = Vec2f{momentum_[0] * dt, momentum_[1] * dt};           // FIX: replaced explicit type with auto
            position_ = position_.apply(dx, std::plus<float>());

            externalForce_ = Vec2f{0, 0};
        }

        std::pair<Vec2f, Vec2f> getBounds() const {
            return geometry_->boundingBox(position_);
        }
    };

    std::random_device Entity::rd_;
    std::mt19937 Entity::gen_(Entity::rd_());

    // ... (rest of your code unchanged) ...

} // namespace PhysicsSim

int main() {
    PhysicsSim::ArcadePhysicsSimulator sim;
    sim.runMainLoop();
    return 0;
}
