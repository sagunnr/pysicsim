#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <limits>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
#include <format>

constexpr int SCREEN_W = 64, SCREEN_H = 32; // for ascii output
constexpr float DT = 1.0f/30.0f; // 30 fps
constexpr float GRAVITY = 400.0f;
constexpr float EPSILON = 1e-3f;
constexpr int MAX_BODIES = 32;

enum class ShapeType { Circle, AABB };

struct CircleData { float radius; };
struct AABBData   { float w, h; };

struct Vec2 {
    float x{}, y{};
    Vec2() = default;
    Vec2(float X, float Y) : x(X), y(Y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s) const { return {x*s, y*s}; }
    Vec2 operator/(float s) const { return {x/s, y/s}; }
    float dot(const Vec2& o) const { return x*o.x+y*o.y; }
    float len2() const { return x*x+y*y; }
    float len() const { return std::sqrt(len2()); }
    Vec2 norm() const { float l=len(); return l>0 ? *this/l : Vec2{}; }
};

struct BodySoA {
    int count=0;
    ShapeType shape[MAX_BODIES];
    CircleData circ[MAX_BODIES];
    AABBData   box[MAX_BODIES];
    Vec2  pos[MAX_BODIES], vel[MAX_BODIES], force[MAX_BODIES];
    float mass[MAX_BODIES], inv_mass[MAX_BODIES], restitution[MAX_BODIES], friction[MAX_BODIES];
    bool  isStatic[MAX_BODIES];
};

void addCircle(BodySoA& w, Vec2 pos, float r, float m, float rest, float fric, bool isStatic) {
    assert(w.count<MAX_BODIES);
    int id=w.count++;
    w.shape[id]=ShapeType::Circle; w.circ[id].radius=r;
    w.pos[id]=pos; w.vel[id]={0,0}; w.force[id]={0,0};
    w.mass[id]=m; w.inv_mass[id]=isStatic?0.0f:1.0f/m;
    w.restitution[id]=rest; w.friction[id]=fric; w.isStatic[id]=isStatic;
}
void addAABB(BodySoA& w, Vec2 pos, float w_, float h_, float m, float rest, float fric, bool isStatic) {
    assert(w.count<MAX_BODIES);
    int id=w.count++;
    w.shape[id]=ShapeType::AABB; w.box[id]={w_,h_};
    w.pos[id]=pos; w.vel[id]={0,0}; w.force[id]={0,0};
    w.mass[id]=m; w.inv_mass[id]=isStatic?0.0f:1.0f/m;
    w.restitution[id]=rest; w.friction[id]=fric; w.isStatic[id]=isStatic;
}

// Simple ASCII visualization (fixed grid, not to scale)
void drawBodies(const BodySoA& w, bool debug=false) {
    char grid[SCREEN_H][SCREEN_W+1] = {};
    for(int y=0;y<SCREEN_H;++y) for(int x=0;x<SCREEN_W;++x) grid[y][x]=' ';
    for(int i=0;i<w.count;++i) {
        int px=int(w.pos[i].x/800.0f*SCREEN_W);
        int py=int(w.pos[i].y/600.0f*SCREEN_H);
        if(px>=0 && px<SCREEN_W && py>=0 && py<SCREEN_H) {
            grid[py][px]=w.shape[i]==ShapeType::Circle?'o':'#';
        }
    }
    std::cout << "+";
    for(int x=0;x<SCREEN_W;++x) std::cout<<"-";
    std::cout<<"+\n";
    for(int y=0;y<SCREEN_H;++y) {
        std::cout<<"|";
        for(int x=0;x<SCREEN_W;++x) std::cout<<grid[y][x];
        std::cout<<"|\n";
    }
    std::cout << "+";
    for(int x=0;x<SCREEN_W;++x) std::cout<<"-";
    std::cout<<"+\n";
    if(debug) {
        for(int i=0;i<w.count;++i) {
            std::cout << std::format("#{}: pos({:.1f},{:.1f}) vel({:.1f},{:.1f}) {}\n",
                    i,w.pos[i].x,w.pos[i].y,w.vel[i].x,w.vel[i].y,w.isStatic[i]?"STATIC":"");
        }
    }
}

// Returns: (colliding, normal-from-A-to-B, penetration)
std::tuple<bool, Vec2, float> collide(const BodySoA& w, int ia, int ib) {
    if(w.shape[ia]==ShapeType::Circle&&w.shape[ib]==ShapeType::Circle) {
        float rA=w.circ[ia].radius, rB=w.circ[ib].radius;
        Vec2 d=w.pos[ib]-w.pos[ia]; float dist2=d.len2(), rSum=rA+rB;
        if(dist2<rSum*rSum) {
            float dist=std::sqrt(dist2);
            Vec2 n = dist>EPSILON ? d/dist : Vec2{1,0};
            return {true, n, rSum-dist};
        }
    } else if(w.shape[ia]==ShapeType::AABB && w.shape[ib]==ShapeType::AABB) {
        Vec2 minA=w.pos[ia]-Vec2{w.box[ia].w/2,w.box[ia].h/2}, maxA=w.pos[ia]+Vec2{w.box[ia].w/2,w.box[ia].h/2};
        Vec2 minB=w.pos[ib]-Vec2{w.box[ib].w/2,w.box[ib].h/2}, maxB=w.pos[ib]+Vec2{w.box[ib].w/2,w.box[ib].h/2};
        if(minA.x<=maxB.x && maxA.x>=minB.x && minA.y<=maxB.y && maxA.y>=minB.y) {
            float dx1=maxA.x-minB.x, dx2=maxB.x-minA.x, dy1=maxA.y-minB.y, dy2=maxB.y-minA.y;
            float px=std::min(dx1,dx2), py=std::min(dy1,dy2);
            if(px<py)
                return {true,{dx1<dx2?-1.f:1.f,0},px};
            else
                return {true,{0,dy1<dy2?-1.f:1.f},py};
        }
    } else {
        // Always circle as ia, box as ib
        int icirc=ia, ibox=ib;
        if(w.shape[ib]==ShapeType::Circle) std::swap(icirc,ibox);
        // circle -> ia, box -> ib
        Vec2 boxmin=w.pos[ibox]-Vec2{w.box[ibox].w/2,w.box[ibox].h/2},boxmax=w.pos[ibox]+Vec2{w.box[ibox].w/2,w.box[ibox].h/2};
        Vec2 c=w.pos[icirc];
        Vec2 closest{
            std::clamp(c.x, boxmin.x, boxmax.x),
            std::clamp(c.y, boxmin.y, boxmax.y)
        };
        Vec2 n=c-closest;
        float d2=n.len2(), r=w.circ[icirc].radius;
        if(d2<r*r) {
            float d=std::sqrt(d2);
            Vec2 norm = d>EPSILON ? n/d : Vec2{1,0};
            return {true,icirc==ia?norm:-norm,r-d};
        }
    }
    return {false,{},0};
}

// Impulse collision resolution (A->B)
void applyImpulse(BodySoA& w, int ia, int ib, Vec2 normal) {
    Vec2 rv=w.vel[ib]-w.vel[ia];
    float velAlongNormal = rv.dot(normal);
    if(velAlongNormal > 0) return;
    float e=std::min(w.restitution[ia],w.restitution[ib]);
    float invMass = w.inv_mass[ia]+w.inv_mass[ib];
    if(invMass==0) return;
    float j = -(1+e)*velAlongNormal/invMass;
    Vec2 impulse = normal*j;
    if(!w.isStatic[ia]) w.vel[ia]-=impulse*w.inv_mass[ia];
    if(!w.isStatic[ib]) w.vel[ib]+=impulse*w.inv_mass[ib];
    // Simple friction
    rv = w.vel[ib]-w.vel[ia];
    Vec2 tangent = rv - normal*rv.dot(normal);
    float tl=tangent.len();
    if(tl>EPSILON) tangent=tangent/tl; else return;
    float mu=std::sqrt(w.friction[ia]*w.friction[ib]);
    float jt = -rv.dot(tangent)/invMass;
    jt=std::clamp(jt,-j*mu,j*mu);
    Vec2 frictionImpulse = tangent*jt;
    if(!w.isStatic[ia]) w.vel[ia]-=frictionImpulse*w.inv_mass[ia];
    if(!w.isStatic[ib]) w.vel[ib]+=frictionImpulse*w.inv_mass[ib];
}

// Integrate and apply gravity
void integrate(BodySoA& w) {
    for(int i=0;i<w.count;++i) {
        if(!w.isStatic[i]) {
            w.force[i].y+=w.mass[i]*GRAVITY;
            w.vel[i]+=w.force[i]*w.inv_mass[i]*DT;
            w.pos[i]+=w.vel[i]*DT;
        }
        w.force[i]={0,0};
    }
}

// World step (pairwise collision)
void worldStep(BodySoA& w) {
    integrate(w);
    // broadphase not implemented for clarity (O(N^2) brute-force)
    for(int ia=0;ia<w.count;++ia)
        for(int ib=ia+1;ib<w.count;++ib) {
            if(w.inv_mass[ia]+w.inv_mass[ib]==0) continue; // both static
            auto [coll,norm,pen]=collide(w,ia,ib);
            if(coll) {
                applyImpulse(w,ia,ib,norm);
                // Positional correction to separate overlapping
                float percent=0.3f, slop=0.005f;
                float corr = std::max(pen-slop,0.0f)/(w.inv_mass[ia]+w.inv_mass[ib])*percent;
                if(!w.isStatic[ia]) w.pos[ia]-=norm*corr*w.inv_mass[ia];
                if(!w.isStatic[ib]) w.pos[ib]+=norm*corr*w.inv_mass[ib];
            }
        }
}


int main() {
    BodySoA world;
    // Static ground
    addAABB(world, {400,590}, 800,20, 0, 0.2f,0.3f,true);
    // Walls
    addAABB(world, {10,300}, 20,600,0,0.2f,0.3f,true);
    addAABB(world, {790,300},20,600,0,0.2f,0.3f,true);
    // Dynamic circles and AABB
    addCircle(world, {200,100},30,8.0f,0.8f,0.2f,false);
    addCircle(world, {600,70},24,6.0f,0.7f,0.2f,false);
    addAABB(world, {400,150}, 40,40,15.0f, 0.4f,0.2f, false);
    // Interactive console loop
    bool debug=false, running=true;
    int frame=0;
    while(running) {
        worldStep(world);
        drawBodies(world, debug);
        std::cout<<"Frame "<<frame++<<"  [D=debug, Q=quit, C=drop circle, B=drop box]: ";
        std::string cmd; std::getline(std::cin,cmd);
        if(!cmd.empty()) {
            for(char ch:cmd) switch(ch) {
                case 'q': case 'Q': running=false; break;
                case 'd': case 'D': debug=!debug; break;
                case 'c': case 'C': addCircle(world, {100+std::rand()%600, 50}, 20+std::rand()%25, 5.0f+std::rand()%6, 0.9f,0.3f,false); break;
                case 'b': case 'B': addAABB(world, {100+std::rand()%600, 50}, 30+std::rand()%17, 30+std::rand()%17, 8.0f+std::rand()%8, 0.7f, 0.3f, false); break;
            }
        }
    }
    return 0;
}
