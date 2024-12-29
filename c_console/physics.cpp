#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <random>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <unistd.h>

const int WINDOW_WIDTH = 1200;
const int WINDOW_HEIGHT = 800;
const float GRAVITY = 9.81f;
const float TIME_STEP = 0.01f;
const float BOUNCE_DAMPING = 0.7f;
const float PARTICLE_MASS = 10.0f;
const float PARTICLE_RADIUS = 50.0f;

// Structure to represent a 2D vector
struct Vec2 {
    float x, y;
    
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }
    
    Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }
    
    float length() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }
};

// Particle class representing a physical object
class Particle {
public:
    Vec2 position;
    Vec2 velocity;
    float mass;
    float radius;
    unsigned long color;
    
    Particle(Vec2 pos, Vec2 vel, float m, float r, unsigned long c) 
        : position(pos), velocity(vel), mass(m), radius(r), color(c) {}
    
    void update() {
        velocity.y += GRAVITY * TIME_STEP;
        position = position + velocity * TIME_STEP;
        handleCollisions();
    }
    
private:
    void handleCollisions() {
        if (position.y >= WINDOW_HEIGHT - radius) {
            position.y = WINDOW_HEIGHT - radius;
            velocity.y = -velocity.y * BOUNCE_DAMPING;
        }
        
        if (position.y <= radius) {
            position.y = radius;
            velocity.y = -velocity.y * BOUNCE_DAMPING;
        }
        
        if (position.x >= WINDOW_WIDTH - radius) {
            position.x = WINDOW_WIDTH - radius;
            velocity.x = -velocity.x * BOUNCE_DAMPING;
        }
        
        if (position.x <= radius) {
            position.x = radius;
            velocity.x = -velocity.x * BOUNCE_DAMPING;
        }
    }
};

class PhysicsSimulation {
private:
    bool isDragging;
    int draggedParticleIndex;
    Vec2 lastMousePos;
    Vec2 mouseVelocity;
    bool isMouseDown;
    std::chrono::steady_clock::time_point lastParticleTime;
    const std::chrono::milliseconds PARTICLE_SPAWN_DELAY{100};
    
    std::vector<Particle> particles;
    Display* display;
    Window window;
    GC gc;
    Colormap colormap;
    XEvent event;
    bool running;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> vel_dist;
    Vec2 dragStartPos;
    std::vector<std::pair<Vec2, Vec2>> collisionVectors;  // Store collision points and forces
    const float VECTOR_SCALE = 5.0f;  // Scale factor for vector visualization
    const int VECTOR_LIFETIME_MS = 200;  // How long collision vectors stay visible
    struct CollisionVector {
        Vec2 start;
        Vec2 end;
        unsigned long color;
        std::chrono::steady_clock::time_point timestamp;
    };
    std::vector<CollisionVector> collisionHistory;
    
public:
    PhysicsSimulation() : 
        gen(rd()), 
        vel_dist(-10.0, 10.0),
        isDragging(false),
        draggedParticleIndex(-1),
        lastMousePos(0, 0),
        mouseVelocity(0, 0),
        isMouseDown(false),
        dragStartPos(0, 0) {
        display = XOpenDisplay(nullptr);
        if (!display) {
            std::cerr << "Cannot open display\n";
            exit(1);
        }
        
        int screen = DefaultScreen(display);
        Window root = RootWindow(display, screen);
        colormap = DefaultColormap(display, screen);
        
        // Create black background
        XSetWindowAttributes attributes;
        attributes.background_pixel = BlackPixel(display, screen);
        
        // Create the window
        window = XCreateWindow(display, root,
                             0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, 0,
                             DefaultDepth(display, screen),
                             InputOutput, DefaultVisual(display, screen),
                             CWBackPixel, &attributes);
        
        // Set window title
        XStoreName(display, window, "Physics Simulation - Click to add particles!");
        
        // Select kinds of events we are interested in
        XSelectInput(display, window, 
            ExposureMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask);
        
        // Create graphics context
        gc = XCreateGC(display, window, 0, nullptr);
        
        // Show the window
        XMapWindow(display, window);
        XFlush(display);
        
        running = true;
    }
    
    ~PhysicsSimulation() {
        XFreeGC(display, gc);
        XDestroyWindow(display, window);
        XCloseDisplay(display);
    }
    
    void addParticle(int x, int y) {
        XColor color;
        // Generate random color
        color.red = rand() % 65535;
        color.green = rand() % 65535;
        color.blue = rand() % 65535;
        XAllocColor(display, colormap, &color);
        
        particles.emplace_back(
            Vec2(x, y),                          // position
            Vec2(vel_dist(gen), vel_dist(gen)),  // random velocity
            PARTICLE_MASS,                                // mass
            PARTICLE_RADIUS,                               // radius
            color.pixel                          // color
        );
    }
    
    void update() {
        // Update mouse velocity
        if (isDragging && draggedParticleIndex >= 0) {
            particles[draggedParticleIndex].position = lastMousePos;
            particles[draggedParticleIndex].velocity = mouseVelocity;
        }
        
        // Handle particle collisions
        for (size_t i = 0; i < particles.size(); i++) {
            for (size_t j = i + 1; j < particles.size(); j++) {
                handleParticleCollision(particles[i], particles[j]);
            }
        }
        
        // Update particle positions
        for (auto& particle : particles) {
            particle.update();
        }
    }
    
    void draw() {
        // Clear window
        XSetForeground(display, gc, BlackPixel(display, DefaultScreen(display)));
        XFillRectangle(display, window, gc, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
        
        // Draw particles
        for (const auto& particle : particles) {
            XSetForeground(display, gc, particle.color);
            XFillArc(display, window, gc,
                    particle.position.x - particle.radius,
                    particle.position.y - particle.radius,
                    particle.radius * 2,
                    particle.radius * 2,
                    0, 360 * 64);
        }

        // Draw drag vector if dragging
        if (isDragging && draggedParticleIndex >= 0) {
            XSetForeground(display, gc, 0xFF0000);  // Red color for drag vector
            XDrawLine(display, window, gc,
                     dragStartPos.x, dragStartPos.y,
                     lastMousePos.x, lastMousePos.y);
        }

        // Draw recent collision vectors
        auto currentTime = std::chrono::steady_clock::now();
        auto it = collisionHistory.begin();
        while (it != collisionHistory.end()) {
            if (currentTime - it->timestamp > std::chrono::milliseconds(VECTOR_LIFETIME_MS)) {
                it = collisionHistory.erase(it);
            } else {
                XSetForeground(display, gc, it->color);
                XDrawLine(display, window, gc,
                         it->start.x, it->start.y,
                         it->end.x, it->end.y);
                ++it;
            }
        }

        XFlush(display);
    }
    
    void run() {
        while (running) {
            while (XPending(display)) {
                XNextEvent(display, &event);
                switch (event.type) {
                    case ButtonPress:
                        if (event.xbutton.button == Button1) {
                            draggedParticleIndex = findClickedParticle(event.xbutton.x, event.xbutton.y);
                            if (draggedParticleIndex >= 0) {
                                isDragging = true;
                                dragStartPos = Vec2(event.xbutton.x, event.xbutton.y);
                                lastMousePos = dragStartPos;
                            } else {
                                isMouseDown = true;
                                lastParticleTime = std::chrono::steady_clock::now();
                                addParticle(event.xbutton.x, event.xbutton.y);
                            }
                        }
                        break;
                    
                    case ButtonRelease:
                        if (event.xbutton.button == Button1) {
                            if (isDragging && draggedParticleIndex >= 0) {
                                Vec2 dragVector = dragStartPos - lastMousePos;
                                float dragDistance = dragVector.length();
                                float velocityScale = std::min(dragDistance * 0.4f, 150.0f);
                                
                                particles[draggedParticleIndex].velocity = dragVector * (velocityScale / dragDistance);
                            }
                            isDragging = false;
                            draggedParticleIndex = -1;
                            isMouseDown = false;
                        }
                        break;
                    
                    case MotionNotify:
                        {
                            Vec2 currentMousePos(event.xmotion.x, event.xmotion.y);
                            if (isDragging && draggedParticleIndex >= 0) {
                                particles[draggedParticleIndex].position = currentMousePos;
                                mouseVelocity = Vec2(0, 0);
                            } else {
                                mouseVelocity = (currentMousePos - lastMousePos) * (1.0f / TIME_STEP);
                            }
                            lastMousePos = currentMousePos;

                            if (isMouseDown && draggedParticleIndex < 0) {
                                auto currentTime = std::chrono::steady_clock::now();
                                if (currentTime - lastParticleTime >= PARTICLE_SPAWN_DELAY) {
                                    addParticle(event.xmotion.x, event.xmotion.y);
                                    lastParticleTime = currentTime;
                                }
                            }
                        }
                        break;
                }
            }
            
            update();
            draw();
            
            // Control simulation speed
            std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
        }
    }
    
private:
    void handleParticleCollision(Particle& p1, Particle& p2) {
        Vec2 delta = p1.position - p2.position;
        float distance = delta.length();
        float minDistance = p1.radius + p2.radius;
        
        if (distance < minDistance) {
            // Collision detected
            Vec2 collisionNormal = Vec2(delta.x / distance, delta.y / distance);
            
            // Move particles apart
            float overlap = minDistance - distance;
            p1.position = p1.position + collisionNormal * (overlap * 0.5f);
            p2.position = p2.position - collisionNormal * (overlap * 0.5f);
            
            // Calculate relative velocity
            Vec2 relativeVel = p1.velocity - p2.velocity;
            
            // Calculate impulse
            float velAlongNormal = relativeVel.x * collisionNormal.x + 
                                 relativeVel.y * collisionNormal.y;
            
            if (velAlongNormal > 0) return;
            
            float restitution = BOUNCE_DAMPING;
            float impulseScalar = -(1.0f + restitution) * velAlongNormal;
            impulseScalar /= 1.0f/p1.mass + 1.0f/p2.mass;
            
            Vec2 impulse = collisionNormal * impulseScalar;
            
            // Add collision vector to visualization
            Vec2 collisionPoint = p1.position - collisionNormal * p1.radius;
            Vec2 vectorEnd = collisionPoint + collisionNormal * (impulseScalar * VECTOR_SCALE);
            
            // Add collision vector with yellow color
            collisionHistory.push_back({
                collisionPoint,
                vectorEnd,
                0xFFFF00,  // Yellow color
                std::chrono::steady_clock::now()
            });

            p1.velocity = p1.velocity + impulse * (1.0f/p1.mass);
            p2.velocity = p2.velocity - impulse * (1.0f/p2.mass);
        }
    }
    
    int findClickedParticle(int x, int y) {
        Vec2 mousePos(x, y);
        for (int i = particles.size() - 1; i >= 0; i--) {
            Vec2 delta = particles[i].position - mousePos;
            if (delta.length() <= particles[i].radius) {
                return i;
            }
        }
        return -1;
    }
};

int main() {
    PhysicsSimulation sim;
    sim.run();
    return 0;
}