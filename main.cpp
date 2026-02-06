#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

struct Particle {
    sf::Vector2f pos;
    sf::Vector2f oldPos;
    sf::Vector2f acc;
    float radius;
};

struct FastGrid {
    float cellSize;
    int cols, rows;
    std::vector<int> head;
    std::vector<int> next;

    void init(int w, int h, float c) {
        cellSize = c;
        cols = (w / c) + 2;
        rows = (h / c) + 2;
        head.assign(cols * rows, -1);
    }

    void clear() { std::fill(head.begin(), head.end(), -1); }

    void insert(int i, const sf::Vector2f& p) {
        int gx = std::max(0, std::min(cols - 1, (int)(p.x / cellSize)));
        int gy = std::max(0, std::min(rows - 1, (int)(p.y / cellSize)));
        int cellIdx = gy * cols + gx;
        next[i] = head[cellIdx];
        head[cellIdx] = i;
    }
};

int main() {
    float pRad = 2.5f;
    constexpr float gravity = 1000.0f;
    constexpr float substeps = 8;
    constexpr float damping = 0.999f;
    constexpr float pressureConstant = 500.0f;

    sf::RenderWindow window(sf::VideoMode(500, 500), "Fluid Simulation", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);

    sf::Vector2u winSize = window.getSize();
    float boxLeft = winSize.x * 0.1f, boxWidth = winSize.x * 0.8f;
    float boxTop = winSize.y * 0.4f, boxHeight = winSize.y * 0.5f;
    float boxRight = boxLeft + boxWidth, boxBottom = boxTop + boxHeight;

    std::vector<Particle> particles;
    FastGrid grid;
    grid.init(winSize.x, winSize.y, pRad * 3.0f);

    sf::CircleShape shape(pRad);
    shape.setOrigin(pRad, pRad);

    float interDist = pRad * 15.0f;
    sf::CircleShape mouseCircle(interDist);
    mouseCircle.setOrigin(interDist, interDist);
    mouseCircle.setFillColor(sf::Color::Transparent);
    mouseCircle.setOutlineColor(sf::Color(255, 255, 255, 80));
    mouseCircle.setOutlineThickness(1.0f);

    sf::Font font;
    font.loadFromFile("/System/Library/Fonts/Supplemental/STIXGeneralBol.otf");

    sf::Clock physicsClock;
    sf::Clock fpsClock;
    float fps = 0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::R) {
                    particles.clear();
                    grid.next.clear();
                }
                bool isCmdDown = sf::Keyboard::isKeyPressed(sf::Keyboard::LSystem) || sf::Keyboard::isKeyPressed(sf::Keyboard::RSystem);
                if (isCmdDown) {
                    if (event.key.code == sf::Keyboard::Equal) pRad += 0.5f;
                    if (event.key.code == sf::Keyboard::Dash)  pRad -= 0.5f;
                    pRad = std::max(1.0f, std::min(pRad, 20.0f));
                    grid.init(winSize.x, winSize.y, pRad * 3.0f);
                    shape.setRadius(pRad);
                    shape.setOrigin(pRad, pRad);
                    interDist = pRad * 15.0f;
                    mouseCircle.setRadius(interDist);
                    mouseCircle.setOrigin(interDist, interDist);
                    for(auto& p : particles) p.radius = pRad;
                }
            }
        }

        sf::Vector2f mPos = sf::Vector2f(sf::Mouse::getPosition(window));
        bool isClicking = sf::Mouse::isButtonPressed(sf::Mouse::Left);
        bool isAttracting = sf::Mouse::isButtonPressed(sf::Mouse::Right);

        // FIXED: Reduced spawn count and wider spread to prevent "explosions"
        if (isClicking) {
            for(int i = 0; i < 4; ++i) {
                Particle p;
                p.pos = mPos + sf::Vector2f(rand() % 60 - 20, rand() % 60 - 20);
                p.oldPos = p.pos;
                p.radius = pRad;
                particles.push_back(p);
                grid.next.push_back(-1);
            }
        }

        float dt = physicsClock.restart().asSeconds();
        float sub_dt = std::min(dt, 0.016f) / substeps;

        for (int s = 0; s < substeps; ++s) {
            for (auto& p : particles) {
                p.acc = {0, gravity};
                sf::Vector2f diff = p.pos - mPos;
                float distSq = diff.x * diff.x + diff.y * diff.y;

                if (distSq < interDist * interDist && distSq > 0.1f) {
                    float dist = std::sqrt(distSq);
                    sf::Vector2f dir = diff / dist;
                    if (isAttracting) p.acc -= dir * 25000.0f;
                    else if (!isClicking) p.acc += dir * 8000.0f;
                }
            }

            for (auto& p : particles) {
                sf::Vector2f vel = (p.pos - p.oldPos) * damping;
                p.oldPos = p.pos;
                p.pos = p.pos + vel + p.acc * (sub_dt * sub_dt);
            }

            grid.clear();
            for (int i = 0; i < (int)particles.size(); ++i) grid.insert(i, particles[i].pos);

            for (int i = 0; i < (int)particles.size(); ++i) {
                int gx = (int)(particles[i].pos.x / grid.cellSize);
                int gy = (int)(particles[i].pos.y / grid.cellSize);
                for (int nx = gx - 1; nx <= gx + 1; ++nx) {
                    for (int ny = gy - 1; ny <= gy + 1; ++ny) {
                        if (nx < 0 || nx >= grid.cols || ny < 0 || ny >= grid.rows) continue;
                        int j = grid.head[ny * grid.cols + nx];
                        while (j != -1) {
                            if (i != j) {
                                sf::Vector2f axis = particles[i].pos - particles[j].pos;
                                float distSq = axis.x * axis.x + axis.y * axis.y;
                                float minDist = particles[i].radius + particles[j].radius;
                                if (distSq < minDist * minDist && distSq > 0.0001f) {
                                    float dist = std::sqrt(distSq);
                                    sf::Vector2f n = axis / dist;
                                    float overlap = minDist - dist;
                                    float pressureForce = (overlap / minDist) * pressureConstant;
                                    sf::Vector2f impulse = n * pressureForce * sub_dt;
                                    particles[i].pos += impulse;
                                    particles[j].pos -= impulse;
                                    float ratio = 0.5f * overlap / dist;
                                    particles[i].pos += axis * ratio * 0.8f;
                                    particles[j].pos -= axis * ratio * 0.8f;
                                }
                            }
                            j = grid.next[j];
                        }
                    }
                }
            }

            for (auto& p : particles) {
                if (p.pos.x > boxLeft - p.radius && p.pos.x < boxRight + p.radius) {
                    if (p.pos.y > boxTop - p.radius && p.pos.y < boxBottom + p.radius) {
                        if (p.pos.y > boxBottom - p.radius) p.pos.y = boxBottom - p.radius;
                        if (p.pos.x < boxLeft + p.radius)  p.pos.x = boxLeft + p.radius;
                        if (p.pos.x > boxRight - p.radius) p.pos.x = boxRight - p.radius;
                    }
                }
            }
        }

        particles.erase(std::remove_if(particles.begin(), particles.end(), [&](const Particle& p) {
            return p.pos.y > winSize.y + 100 || p.pos.x < -100 || p.pos.x > winSize.x + 100;
        }), particles.end());
        grid.next.resize(particles.size(), -1);

        window.clear(sf::Color(0, 0, 0));

        mouseCircle.setPosition(mPos);
        window.draw(mouseCircle);

        sf::VertexArray lines(sf::LinesStrip, 4);
        lines[0].position = {boxLeft, boxTop}; lines[1].position = {boxLeft, boxBottom};
        lines[2].position = {boxRight, boxBottom}; lines[3].position = {boxRight, boxTop};
        for(int i = 0; i < 4; ++i) lines[i].color = sf::Color::White;
        window.draw(lines);

        for (const auto& p : particles) {
            sf::Vector2f vel = p.pos - p.oldPos;
            float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y) * 120.0f;
            int r = std::min(255, (int)(100 + speed));
            int g = std::min(255, (int)(220 + speed));
            shape.setFillColor(sf::Color(r, g, 255));
            shape.setPosition(p.pos);
            window.draw(shape);
        }

        fps = 0.9f * fps + 0.1f * (1.0f / fpsClock.restart().asSeconds());
        sf::Text uiText("Particles: " + std::to_string(particles.size()) + "\nFPS: " + std::to_string((int)fps), font, 14);
        uiText.setFillColor(sf::Color::White);
        uiText.setPosition(10, 10);
        window.draw(uiText);

        window.display();
    }
    return 0;
}