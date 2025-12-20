#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <random>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include "geometry.h"
#include "path_finder.h"

// Global variables
FreeSpace* freeSpace = nullptr;
PathFinder pathFinder;
std::vector<Point_2> queryPoints;
PathFinderResult currentPath;
std::vector<Polygon_2> obstacles;
Polygon_2 boundingBox;
std::string statusMessage = "Click two points to find a path";
bool pathFound = false;
std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

// Generate guaranteed non-convex polygon
Polygon_2 generateNonConvexPolygon(int numVertices, double centerX, double centerY, double radius) {
    std::vector<Point_2> points;
    
    // Always create non-convex shapes
    if (numVertices >= 5) {
        // Star shape (guaranteed non-convex)
        for (int i = 0; i < numVertices; i++) {
            double angle = 2 * 3.14159 * i / numVertices;
            double r = (i % 2 == 0) ? radius : radius * 0.6;
            double x = centerX + r * cos(angle);
            double y = centerY + r * sin(angle);
            points.push_back(Point_2(x, y));
        }
    } else {
        // L-shape for small polygons
        points.push_back(Point_2(centerX - radius/2, centerY - radius/2));
        points.push_back(Point_2(centerX + radius/2, centerY - radius/2));
        points.push_back(Point_2(centerX + radius/2, centerY));
        points.push_back(Point_2(centerX, centerY));
        points.push_back(Point_2(centerX, centerY + radius/2));
        points.push_back(Point_2(centerX - radius/2, centerY + radius/2));
    }
    
    return Polygon_2(points.begin(), points.end());
}

void initializeScene() {
    // Create bounding box
    std::vector<Point_2> bboxPoints = {
        Point_2(50, 50),
        Point_2(750, 50),
        Point_2(750, 550),
        Point_2(50, 550)
    };
    boundingBox = Polygon_2(bboxPoints.begin(), bboxPoints.end());
    
    // Generate GUARANTEED non-convex obstacles
    obstacles.clear();
    std::uniform_real_distribution<> posX(120, 680);
    std::uniform_real_distribution<> posY(120, 480);
    std::uniform_int_distribution<> vertexCount(6, 10);
    std::uniform_real_distribution<> sizeDist(40, 70);
    
    // Create 4-5 obstacles that are GUARANTEED non-convex
    int numObstacles = 4 + (rng() % 2);
    
    // Keep track of obstacle positions to avoid overlap
    std::vector<std::pair<double, double>> obstaclePositions;
    
    for (int i = 0; i < numObstacles; i++) {
        double x, y;
        bool validPosition;
        int attempts = 0;
        
        do {
            validPosition = true;
            x = posX(rng);
            y = posY(rng);
            
            // Check minimum distance from existing obstacles
            for (const auto& pos : obstaclePositions) {
                double dx = x - pos.first;
                double dy = y - pos.second;
                if (dx * dx + dy * dy < 10000) { // 100px minimum distance
                    validPosition = false;
                    break;
                }
            }
            
            // Also keep away from edges
            if (x < 150 || x > 650 || y < 150 || y > 450) {
                validPosition = false;
            }
            
            attempts++;
        } while (!validPosition && attempts < 50);
        
        if (validPosition) {
            obstaclePositions.push_back({x, y});
            double size = sizeDist(rng);
            int vertices = vertexCount(rng);
            obstacles.push_back(generateNonConvexPolygon(vertices, x, y, size));
        }
    }
    
    // Create free space
    if (freeSpace) delete freeSpace;
    freeSpace = new FreeSpace(boundingBox, obstacles);
    
    // Collect points for visibility graph:
    std::vector<Point_2> graphPoints;
    
    // 1. All obstacle vertices
    for (const auto& obstacle : obstacles) {
        for (auto it = obstacle.vertices_begin(); it != obstacle.vertices_end(); ++it) {
            graphPoints.push_back(*it);
        }
    }
    
    // 2. Add points along edges (every 20 pixels)
    for (const auto& obstacle : obstacles) {
        auto vertices = obstacle.container();
        for (size_t i = 0; i < vertices.size(); i++) {
            const Point_2& p1 = vertices[i];
            const Point_2& p2 = vertices[(i + 1) % vertices.size()];
            
            double length = sqrt(CGAL::to_double(
                (p1.x() - p2.x()) * (p1.x() - p2.x()) + 
                (p1.y() - p2.y()) * (p1.y() - p2.y())
            ));
            
            int segments = std::max(2, (int)(length / 20));
            for (int j = 1; j < segments; j++) {
                double t = (double)j / segments;
                Point_2 mid(
                    p1.x() * (1 - t) + p2.x() * t,
                    p1.y() * (1 - t) + p2.y() * t
                );
                graphPoints.push_back(mid);
            }
        }
    }
    
    // 3. Add grid points in free space (every 40 pixels)
    for (int x = 60; x < 750; x += 40) {
        for (int y = 60; y < 550; y += 40) {
            Point_2 point(x, y);
            if (freeSpace->isPointFree(point)) {
                graphPoints.push_back(point);
            }
        }
    }
    
    // 4. Add points around obstacle perimeters (for navigation)
    for (const auto& obstacle : obstacles) {
        auto vertices = obstacle.container();
        for (size_t i = 0; i < vertices.size(); i++) {
            const Point_2& p = vertices[i];
            
            // Add points slightly offset from vertices in 4 directions
            std::vector<std::pair<double, double>> offsets = {
                {15, 15}, {15, -15}, {-15, 15}, {-15, -15},
                {20, 0}, {0, 20}, {-20, 0}, {0, -20}
            };
            
            for (const auto& offset : offsets) {
                Point_2 offsetPoint(
                    p.x() + offset.first,
                    p.y() + offset.second
                );
                
                if (freeSpace->isPointFree(offsetPoint)) {
                    graphPoints.push_back(offsetPoint);
                }
            }
        }
    }
    
    // Remove duplicates (or near-duplicates)
    std::vector<Point_2> uniquePoints;
    for (const auto& p : graphPoints) {
        bool duplicate = false;
        for (const auto& q : uniquePoints) {
            double dx = CGAL::to_double(p.x() - q.x());
            double dy = CGAL::to_double(p.y() - q.y());
            if (dx * dx + dy * dy < 25) { // 5px tolerance
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            uniquePoints.push_back(p);
        }
    }
    
    // Build path finder graph
    pathFinder.buildGraph(uniquePoints, obstacles);
    
    // Clear query points
    queryPoints.clear();
    currentPath.pathExists = false;
    pathFound = false;
    statusMessage = "Scene with " + std::to_string(obstacles.size()) + 
                   " non-convex obstacles. " + std::to_string(uniquePoints.size()) + 
                   " navigation points. Click two points.";
}

void drawStatusPanel() {
    // Status panel background
    glColor4f(0.1f, 0.1f, 0.2f, 0.9f);
    glBegin(GL_QUADS);
    glVertex2f(10, 560);
    glVertex2f(790, 560);
    glVertex2f(790, 590);
    glVertex2f(10, 590);
    glEnd();
    
    // Border
    glColor3f(0.4f, 0.6f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(10, 560);
    glVertex2f(790, 560);
    glVertex2f(790, 590);
    glVertex2f(10, 590);
    glEnd();
    
    // Status indicator (left side)
    if (pathFound) {
        glColor3f(0.2f, 0.8f, 0.2f); // Green for path found
    } else if (queryPoints.size() == 2 && !currentPath.pathExists) {
        glColor3f(0.8f, 0.2f, 0.2f); // Red for no path
    } else {
        glColor3f(0.6f, 0.6f, 0.6f); // Gray for neutral
    }
    
    glBegin(GL_QUADS);
    glVertex2f(15, 565);
    glVertex2f(25, 565);
    glVertex2f(25, 585);
    glVertex2f(15, 585);
    glEnd();
    
    // Status text area background
    glColor4f(0.15f, 0.15f, 0.25f, 0.8f);
    glBegin(GL_QUADS);
    glVertex2f(40, 565);
    glVertex2f(785, 565);
    glVertex2f(785, 585);
    glVertex2f(40, 585);
    glEnd();
}

void drawControlsPanel() {
    // Controls panel background
    glColor4f(0.1f, 0.1f, 0.2f, 0.9f);
    glBegin(GL_QUADS);
    glVertex2f(10, 10);
    glVertex2f(790, 10);
    glVertex2f(790, 50);
    glVertex2f(10, 50);
    glEnd();
    
    // Border
    glColor3f(0.4f, 0.6f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(10, 10);
    glVertex2f(790, 10);
    glVertex2f(790, 50);
    glVertex2f(10, 50);
    glEnd();
    
    // Control indicators
    float x = 20;
    float y = 20;
    float size = 8;
    
    // R key indicator
    glColor3f(0.4f, 0.6f, 1.0f);
    glBegin(GL_QUADS);
    glVertex2f(x, y - size);
    glVertex2f(x + size * 1.5, y - size);
    glVertex2f(x + size * 1.5, y + size);
    glVertex2f(x, y + size);
    glEnd();
    
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y - size);
    glVertex2f(x + size * 1.5, y - size);
    glVertex2f(x + size * 1.5, y + size);
    glVertex2f(x, y + size);
    glEnd();
}

void drawPolygon(const Polygon_2& poly, float r, float g, float b, bool filled = true) {
    if (filled) {
        glBegin(GL_POLYGON);
        for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
            float intensity = 0.7f + 0.3f * (CGAL::to_double(it->x()) / 800.0f);
            glColor3f(r * intensity, g * intensity, b * intensity);
            glVertex2f(CGAL::to_double(it->x()), CGAL::to_double(it->y()));
        }
        glEnd();
    } else {
        glColor3f(r * 0.7f, g * 0.7f, b * 0.7f);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
            glVertex2f(CGAL::to_double(it->x()), CGAL::to_double(it->y()));
        }
        glEnd();
    }
}

void drawPoint(const Point_2& point, float r, float g, float b, float size = 10.0f) {
    glPointSize(size * 1.5f);
    glColor4f(r, g, b, 0.3f);
    glBegin(GL_POINTS);
    glVertex2f(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
    glEnd();
    
    glPointSize(size);
    glColor3f(r, g, b);
    glBegin(GL_POINTS);
    glVertex2f(CGAL::to_double(point.x()), CGAL::to_double(point.y()));
    glEnd();
}

void drawSegment(const Segment_2& segment, float r, float g, float b, float width = 4.0f) {
    glLineWidth(width + 2.0f);
    glColor4f(r, g, b, 0.3f);
    glBegin(GL_LINES);
    glVertex2f(CGAL::to_double(segment.source().x()), CGAL::to_double(segment.source().y()));
    glVertex2f(CGAL::to_double(segment.target().x()), CGAL::to_double(segment.target().y()));
    glEnd();
    
    glLineWidth(width);
    glColor3f(r, g, b);
    glBegin(GL_LINES);
    glVertex2f(CGAL::to_double(segment.source().x()), CGAL::to_double(segment.source().y()));
    glVertex2f(CGAL::to_double(segment.target().x()), CGAL::to_double(segment.target().y()));
    glEnd();
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_R:
                initializeScene();
                std::cout << "\n=== SCENE REGENERATED ===" << std::endl;
                break;
            case GLFW_KEY_C:
                queryPoints.clear();
                currentPath.pathExists = false;
                pathFound = false;
                statusMessage = "Points cleared. Click two new points.";
                std::cout << "Points cleared." << std::endl;
                break;
            case GLFW_KEY_V:
                // Toggle visibility of navigation points (for debugging)
                static bool showNavPoints = false;
                showNavPoints = !showNavPoints;
                std::cout << "Navigation points display: " << (showNavPoints ? "ON" : "OFF") << std::endl;
                break;
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
        }
    }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        ypos = height - ypos;
        
        Point_2 point(xpos, ypos);
        
        if (freeSpace && freeSpace->isPointFree(point)) {
            if (queryPoints.size() >= 2) {
                queryPoints.clear();
                pathFound = false;
                statusMessage = "Cleared old points. Click first point.";
            }
            
            queryPoints.push_back(point);
            
            if (queryPoints.size() == 1) {
                std::stringstream ss;
                ss << "First point at (" << std::fixed << std::setprecision(0) 
                   << xpos << ", " << ypos << "). Click second point.";
                statusMessage = ss.str();
            }
            
            if (queryPoints.size() == 2 && freeSpace) {
                statusMessage = "Finding optimal path...";
                glfwSwapBuffers(window);
                
                currentPath = pathFinder.findPath(queryPoints[0], queryPoints[1]);
                pathFound = currentPath.pathExists;
                
                if (currentPath.pathExists) {
                    std::stringstream ss;
                    ss << "✓ PATH FOUND! Length: " << std::fixed << std::setprecision(2) 
                       << currentPath.totalLength;
                    statusMessage = ss.str();
                    std::cout << "\n=== PATH FOUND ===" << std::endl;
                    std::cout << "Path length: " << currentPath.totalLength << std::endl;
                    std::cout << "Path points: " << currentPath.pathPoints.size() << std::endl;
                } else {
                    statusMessage = "✗ NO PATH FOUND! Try different points.";
                    std::cout << "\n=== NO PATH FOUND ===" << std::endl;
                    std::cout << "The algorithm cannot find a path with current navigation points." << std::endl;
                    std::cout << "Try points that are farther from obstacles or press R to regenerate." << std::endl;
                }
            }
        } else {
            statusMessage = "Invalid point! Must be in free space.";
            std::cout << "Point rejected - inside obstacle or outside boundary." << std::endl;
        }
    }
}

int main(void) {
    if (!glfwInit()) {
        fprintf(stderr, "Failed to init GLFW\n");
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Path Planning - Robust Version", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    
    initializeScene();

    // Set up OpenGL
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glPointSize(10.0f);
    glLineWidth(3.0f);

    // Print controls
    std::cout << "\n=== ROBUST PATH PLANNING ===" << std::endl;
    std::cout << "CONTROLS:" << std::endl;
    std::cout << "  R: Regenerate obstacles" << std::endl;
    std::cout << "  C: Clear points" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    std::cout << "  Click: Place start and end points" << std::endl;
    std::cout << "\nSTATUS INDICATOR (top-left):" << std::endl;
    std::cout << "  Green: Path found" << std::endl;
    std::cout << "  Red: No path found" << std::endl;
    std::cout << "  Gray: Waiting for points" << std::endl;
    std::cout << "\nALGORITHM: Uses dense navigation points for reliable path finding" << std::endl;
    std::cout << "All obstacles are GUARANTEED NON-CONVEX" << std::endl;

    while (!glfwWindowShouldClose(window)) {
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        
        glViewport(0, 0, width, height);
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width, 0, height, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Draw grid
        glColor4f(0.15f, 0.15f, 0.2f, 1.0f);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (int x = 0; x <= width; x += 20) {
            glVertex2f(x, 0);
            glVertex2f(x, height);
        }
        for (int y = 0; y <= height; y += 20) {
            glVertex2f(0, y);
            glVertex2f(width, y);
        }
        glEnd();
        
        // Draw bounding box
        glColor4f(0.2f, 0.4f, 0.8f, 0.3f);
        glLineWidth(6.0f);
        glBegin(GL_LINE_LOOP);
        for (auto it = boundingBox.vertices_begin(); it != boundingBox.vertices_end(); ++it) {
            glVertex2f(CGAL::to_double(it->x()), CGAL::to_double(it->y()));
        }
        glEnd();
        
        glColor3f(0.4f, 0.6f, 1.0f);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        for (auto it = boundingBox.vertices_begin(); it != boundingBox.vertices_end(); ++it) {
            glVertex2f(CGAL::to_double(it->x()), CGAL::to_double(it->y()));
        }
        glEnd();
        
        // Draw obstacles
        for (const auto& obstacle : obstacles) {
            drawPolygon(obstacle, 0.8f, 0.2f, 0.2f, true);
            drawPolygon(obstacle, 0.6f, 0.1f, 0.1f, false);
        }
        
        // Draw query points
        for (const auto& point : queryPoints) {
            drawPoint(point, 0.2f, 0.6f, 1.0f, 12.0f);
        }
        
        // Draw path if exists
        if (currentPath.pathExists && currentPath.pathPoints.size() >= 2) {
            for (size_t i = 0; i < currentPath.pathPoints.size() - 1; i++) {
                Segment_2 seg(currentPath.pathPoints[i], currentPath.pathPoints[i + 1]);
                drawSegment(seg, 0.2f, 0.9f, 0.3f, 5.0f);
            }
            
            for (const auto& point : currentPath.pathPoints) {
                drawPoint(point, 0.1f, 0.8f, 0.2f, 8.0f);
            }
        }
        
        // Draw UI panels
        drawStatusPanel();
        drawControlsPanel();
        
        // Update console status
        static std::string lastStatus = "";
        if (statusMessage != lastStatus) {
            std::cout << "Status: " << statusMessage << std::endl;
            lastStatus = statusMessage;
        }
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    if (freeSpace) delete freeSpace;
    glfwTerminate();
    return 0;
}
