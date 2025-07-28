Below is a single‐file C++23 example using SDL2 that implements a very simple 2D rigid‐body system with gravity, restitution and friction, AABB‐vs‐AABB and circle‐vs‐circle collision detection, impulse resolution, and a debug‐draw toggle (press ‘D’). It runs at 60 FPS, is modular enough to extend (e.g. add polygon support), and draws outlines, contact normals, and velocity/force vectors when debug is on.

```cpp
// physics_demo.cpp
// Compile with: g++ -std=c++23 physics_demo.cpp -lSDL2 -O2 -o physics_demo

#include <SDL2/SDL.h>
#include <vector>
#include <cmath>
#include <algorithm>

// Simple 2D vector
struct Vec2 {
    float x{}, y{};
    Vec2() = default;
    Vec2(float X, float Y): x(X), y(Y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s) const { return {x*s, y*s}; }
    Vec2 operator/(float s) const { return {x/s, y/s}; }
};
inline float dot(const Vec2& a, const Vec2& b){ return a.x*b.x + a.y*b.y; }
inline float length(const Vec2& v){ return std::sqrt(dot(v,v)); }
inline Vec2 normalize(const Vec2& v){ float L=length(v); return L>0?Vec2(v.x/L, v.y/L):Vec2(0,0); }

// Shapes
enum class Shape { CIRCLE, AABB };

// Rigid body
struct Body {
    Shape shape;
    bool isStatic = false;
    Vec2 pos, vel{0,0}, force{0,0};
    float mass = 1.0f, invMass = 1.0f;
    float restitution = 0.5f;   // bounciness
    float friction = 0.2f;      // simple friction coefficient
    // shape data:
    float radius = 0;           // for circles
    Vec2 halfSize{0,0};         // for AABB

    Body(Shape s): shape(s){}
};

// Collision manifold
struct Manifold {
    Body *A, *B;
    Vec2 normal;
    float penetration;
    Vec2 contactPoint;
};

// The world
struct PhysicsWorld {
    std::vector<Body*> bodies;
    std::vector<Manifold> contacts;
    Vec2 gravity{0, 500.0f};  // pixels/sec² downward
    float dt = 1.0f/60.0f;

    void addBody(Body* b){
        if(b->isStatic || b->mass<=0){
            b->invMass = 0;
            b->isStatic = true;
        } else {
            b->invMass = 1.0f / b->mass;
        }
        bodies.push_back(b);
    }

    void step(){
        contacts.clear();
        // integrate forces -> velocities
        for(auto b: bodies){
            if(b->isStatic) continue;
            // gravity
            b->vel.x += gravity.x * dt;
            b->vel.y += gravity.y * dt;
        }
        // broad + narrow phase: O(n²) for this demo
        for(size_t i=0; i<bodies.size(); ++i){
            for(size_t j=i+1; j<bodies.size(); ++j){
                Body *A = bodies[i], *B = bodies[j];
                if(A->invMass==0 && B->invMass==0) continue; // both static
                Manifold m{A,B,{},0,{}};
                if(collide(m)){
                    contacts.push_back(m);
                }
            }
        }
        // resolve collisions
        for(auto& m: contacts) resolveCollision(m);
        // positional correction to prevent sinking
        for(auto& m: contacts) positionalCorrection(m);
        // integrate velocities -> positions
        for(auto b: bodies){
            if(b->isStatic) continue;
            b->pos = b->pos + b->vel * dt;
        }
    }

  private:
    // detect & fill manifold
    bool collide(Manifold& m){
        if(m.A->shape==Shape::CIRCLE && m.B->shape==Shape::CIRCLE){
            return circleVsCircle(m);
        }
        if(m.A->shape==Shape::AABB && m.B->shape==Shape::AABB){
            return aabbVsAabb(m);
        }
        return false; // cross‐shape not implemented
    }

    bool circleVsCircle(Manifold& m){
        Vec2 d = m.B->pos - m.A->pos;
        float r = m.A->radius + m.B->radius;
        float dist2 = dot(d,d);
        if(dist2 >= r*r) return false;
        float dist = std::sqrt(dist2);
        // penetration
        m.penetration = r - dist;
        m.normal = dist>0 ? d*(1.0f/dist) : Vec2(1,0);
        // approximate contact point
        m.contactPoint = m.A->pos + m.normal * (m.A->radius - m.penetration*0.5f);
        return true;
    }

    bool aabbVsAabb(Manifold& m){
        Vec2 aMin = m.A->pos - m.A->halfSize;
        Vec2 aMax = m.A->pos + m.A->halfSize;
        Vec2 bMin = m.B->pos - m.B->halfSize;
        Vec2 bMax = m.B->pos + m.B->halfSize;
        float overlapX = std::min(aMax.x, bMax.x) - std::max(aMin.x, bMin.x);
        float overlapY = std::min(aMax.y, bMax.y) - std::max(aMin.y, bMin.y);
        if(overlapX>0 && overlapY>0){
            // choose axis of least penetration
            if(overlapX < overlapY){
                m.penetration = overlapX;
                m.normal = (m.A->pos.x < m.B->pos.x) ? Vec2(-1,0) : Vec2(1,0);
            } else {
                m.penetration = overlapY;
                m.normal = (m.A->pos.y < m.B->pos.y) ? Vec2(-0, -1) : Vec2(0,1);
            }
            m.contactPoint = (m.A->pos + m.B->pos)*0.5f;
            return true;
        }
        return false;
    }

    void resolveCollision(Manifold& m){
        Body *A = m.A, *B = m.B;
        // relative velocity
        Vec2 rv = B->vel - A->vel;
        float velAlongNormal = dot(rv, m.normal);
        if(velAlongNormal > 0) return; // separating

        // calculate restitution
        float e = std::min(A->restitution, B->restitution);
        // j = impulse scalar
        float invMassSum = A->invMass + B->invMass;
        float j = -(1.0f + e) * velAlongNormal / invMassSum;
        Vec2 impulse = m.normal * j;
        if(!A->isStatic) A->vel = A->vel - impulse * A->invMass;
        if(!B->isStatic) B->vel = B->vel + impulse * B->invMass;

        // friction
        rv = B->vel - A->vel;
        Vec2 tangent = rv - m.normal * dot(rv, m.normal);
        tangent = normalize(tangent);
        float mu = std::sqrt(A->friction * B->friction);
        float jt = -dot(rv, tangent) / invMassSum;
        jt = std::clamp(jt, -j * mu, j * mu);
        Vec2 frictionImpulse = tangent * jt;
        if(!A->isStatic) A->vel = A->vel - frictionImpulse * A->invMass;
        if(!B->isStatic) B->vel = B->vel + frictionImpulse * B->invMass;
    }

    void positionalCorrection(const Manifold& m){
        const float percent = 0.2f; // usually 20% to 80%
        const float slop = 0.01f;   // usually 0.01 to 0.1
        float invMassSum = m.A->invMass + m.B->invMass;
        if(invMassSum==0) return;
        float corr = std::max(m.penetration - slop, 0.0f)/(invMassSum) * percent;
        Vec2 correction = m.normal * corr;
        if(!m.A->isStatic) m.A->pos = m.A->pos - correction * m.A->invMass;
        if(!m.B->isStatic) m.B->pos = m.B->pos + correction * m.B->invMass;
    }
};

// Draw a filled circle by drawing horizontal spans
void DrawFilledCircle(SDL_Renderer* ren, int cx, int cy, int r){
    for(int dy=-r; dy<=r; dy++){
        int dx = (int)std::sqrt(r*r - dy*dy);
        SDL_RenderDrawLine(ren, cx-dx, cy+dy, cx+dx, cy+dy);
    }
}

// Draw circle outline
void DrawCircle(SDL_Renderer* ren, int cx, int cy, int r){
    const int segments = 32;
    float theta = 0, step = 2.0f * M_PI / segments;
    int x0 = cx + (int)(r * std::cos(0)), y0 = cy + (int)(r * std::sin(0));
    for(int i=1; i<=segments; ++i){
        theta += step;
        int x1 = cx + (int)(r * std::cos(theta)), y1 = cy + (int)(r * std::sin(theta));
        SDL_RenderDrawLine(ren, x0, y0, x1, y1);
        x0 = x1; y0 = y1;
    }
}

int main(int argc, char* argv[]){
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* win = SDL_CreateWindow("2D Physics Demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);

    PhysicsWorld world;

    // create floor (static)
    Body floor(Shape::AABB);
    floor.isStatic = true;
    floor.pos = {400, 580};
    floor.halfSize = {400, 20};
    world.addBody(&floor);

    // create some boxes
    for(int i=0;i<5;i++){
        Body* box = new Body(Shape::AABB);
        box->pos = {200.0f + i*80, 100.0f};
        box->halfSize = {30,30};
        box->mass = 2.0f;
        box->restitution = 0.4f;
        box->friction = 0.3f;
        world.addBody(box);
    }
    // create some circles
    for(int i=0;i<5;i++){
        Body* c = new Body(Shape::CIRCLE);
        c->pos = {200.0f + i*80, 0.0f};
        c->radius = 20.0f;
        c->mass = 1.0f;
        c->restitution = 0.6f;
        c->friction = 0.2f;
        world.addBody(c);
    }

    bool running = true;
    bool debugDraw = false;
    Uint32 prevTicks = SDL_GetTicks();

    while(running){
        // -- events --
        SDL_Event e;
        while(SDL_PollEvent(&e)){
            if(e.type==SDL_QUIT) running=false;
            else if(e.type==SDL_KEYDOWN){
                if(e.key.keysym.sym==SDLK_ESCAPE) running=false;
                else if(e.key.keysym.sym==SDLK_d) debugDraw = !debugDraw;
            }
        }
        // -- physics step --
        world.step();

        // -- render --
        SDL_SetRenderDrawColor(ren, 20, 20, 20, 255);
        SDL_RenderClear(ren);

        // draw bodies
        for(auto b: world.bodies){
            if(b->shape==Shape::AABB){
                SDL_Rect r{ int(b->pos.x - b->halfSize.x),
                            int(b->pos.y - b->halfSize.y),
                            int(b->halfSize.x*2),
                            int(b->halfSize.y*2) };
                if(b->isStatic) SDL_SetRenderDrawColor(ren, 100,100,255,255);
                else SDL_SetRenderDrawColor(ren, 200,50,50,255);
                SDL_RenderFillRect(ren, &r);
                if(debugDraw){
                    SDL_SetRenderDrawColor(ren, 255,255,255,255);
                    SDL_RenderDrawRect(ren, &r);
                    // velocity
                    Vec2 vtip = b->pos + b->vel*0.1f;
                    SDL_SetRenderDrawColor(ren, 255,255,0,255);
                    SDL_RenderDrawLine(ren, int(b->pos.x), int(b->pos.y),
                                       int(vtip.x), int(vtip.y));
                }
            } else {
                if(b->isStatic) SDL_SetRenderDrawColor(ren, 100,100,255,255);
                else SDL_SetRenderDrawColor(ren, 200,200,50,255);
                DrawFilledCircle(ren, int(b->pos.x), int(b->pos.y), int(b->radius));
                if(debugDraw){
                    SDL_SetRenderDrawColor(ren, 255,255,255,255);
                    DrawCircle(ren, int(b->pos.x), int(b->pos.y), int(b->radius));
                    Vec2 vtip = b->pos + b->vel*0.1f;
                    SDL_SetRenderDrawColor(ren, 255,255,0,255);
                    SDL_RenderDrawLine(ren, int(b->pos.x), int(b->pos.y),
                                       int(vtip.x), int(vtip.y));
                }
            }
        }

        // draw contact normals
        if(debugDraw){
            for(auto& m: world.contacts){
                SDL_SetRenderDrawColor(ren, 0,255,0,255);
                Vec2 p = m.contactPoint;
                Vec2 q = p + m.normal * 30.0f;
                SDL_RenderDrawLine(ren, int(p.x), int(p.y), int(q.x), int(q.y));
            }
        }

        SDL_RenderPresent(ren);

        // cap at ~60fps
        Uint32 now = SDL_GetTicks();
        Uint32 frameTime = now - prevTicks;
        if(frameTime < 16) SDL_Delay(16 - frameTime);
        prevTicks = now;
    }

    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
```

How it works in brief:

1. **Body** holds shape type, position, velocity, mass, restitution & friction, and shape parameters (radius or halfSize).
2. **PhysicsWorld** steps at a fixed `dt=1/60`.  
   - Applies gravity,  
   - Does naive O(n²) pair checks (skips static–static),  
   - Detects circle–circle & AABB–AABB collisions,  
   - Builds a small `Manifold` for each collision,  
   - Resolves impulses (including friction),  
   - Applies a positional correction to avoid sinking,  
   - Integrates velocities into positions.
3. **Rendering** uses SDL2:  
   - Fills bodies,  
   - On pressing `D`, toggles debug: draws outlines, velocity vectors (yellow), and collision normals (green).
4. **Modularity**: you can add more shape‐vs‐shape routines, stack up more solvers, change integrators, etc.

Compile and link with SDL2 (`-lSDL2`), run, and press `D` to toggle the physics debug overlay. Enjoy!
