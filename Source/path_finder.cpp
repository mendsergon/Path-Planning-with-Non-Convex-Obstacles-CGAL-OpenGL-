#include "path_finder.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <iostream>
#include <cmath>

PathFinder::PathFinder() : pointToIndex(0, Point2Hash()) {
}

double PathFinder::computeDistance(const Point_2& p1, const Point_2& p2) {
    double dx = CGAL::to_double(p1.x() - p2.x());
    double dy = CGAL::to_double(p1.y() - p2.y());
    return std::sqrt(dx * dx + dy * dy);
}

bool PathFinder::isVisible(const Point_2& p1, const Point_2& p2, 
                          const std::vector<Polygon_2>& obstacles) {
    Segment_2 seg(p1, p2);
    
    if (p1 == p2) return false;
    
    // Check if points are very close (allow connection)
    double dist = computeDistance(p1, p2);
    if (dist < 3.0) return true;
    
    // Check all obstacles
    for (const auto& obstacle : obstacles) {
        if (GeometryUtils::segmentIntersectsPolygon(seg, obstacle)) {
            return false;
        }
    }
    
    return true;
}

void PathFinder::buildGraph(const std::vector<Point_2>& points, 
                           const std::vector<Polygon_2>& obstacles) {
    vertices = points;
    adjacencyList.clear();
    adjacencyList.resize(vertices.size());
    pointToIndex.clear();
    
    this->obstacles = obstacles;
    
    for (size_t i = 0; i < vertices.size(); i++) {
        pointToIndex[vertices[i]] = i;
    }
    
    std::cout << "Building graph with " << vertices.size() << " points..." << std::endl;
    
    // Build visibility graph
    int edgeCount = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
        for (size_t j = i + 1; j < vertices.size(); j++) {
            // Connect points that are reasonably close (optimization)
            double dist = computeDistance(vertices[i], vertices[j]);
            if (dist < 100.0) { // Only check visibility for points within 100px
                if (isVisible(vertices[i], vertices[j], obstacles)) {
                    adjacencyList[i].push_back({static_cast<int>(j), dist});
                    adjacencyList[j].push_back({static_cast<int>(i), dist});
                    edgeCount++;
                }
            }
        }
    }
    
    std::cout << "Graph built: " << edgeCount << " edges created" << std::endl;
}

PathFinderResult PathFinder::findPath(const Point_2& start, const Point_2& end) {
    PathFinderResult result;
    result.pathExists = false;
    result.totalLength = 0.0;
    
    // FIRST: Check if start and end are directly visible (fast path)
    if (isVisible(start, end, obstacles)) {
        result.pathExists = true;
        result.totalLength = computeDistance(start, end);
        result.pathPoints = {start, end};
        std::cout << "Direct path found!" << std::endl;
        return result;
    }
    
    // SECOND: Use visibility graph
    std::vector<Point_2> tempVertices = vertices;
    std::vector<std::vector<std::pair<int, double>>> tempAdj = adjacencyList;
    
    int startIdx = static_cast<int>(tempVertices.size());
    tempVertices.push_back(start);
    tempAdj.push_back({});
    
    // Connect start to visible vertices
    int startConnections = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
        if (isVisible(start, vertices[i], obstacles)) {
            double dist = computeDistance(start, vertices[i]);
            tempAdj[startIdx].push_back({static_cast<int>(i), dist});
            tempAdj[i].push_back({startIdx, dist});
            startConnections++;
        }
    }
    
    int endIdx = static_cast<int>(tempVertices.size());
    tempVertices.push_back(end);
    tempAdj.push_back({});
    
    // Connect end to visible vertices
    int endConnections = 0;
    for (size_t i = 0; i < vertices.size(); i++) {
        if (isVisible(end, vertices[i], obstacles)) {
            double dist = computeDistance(end, vertices[i]);
            tempAdj[endIdx].push_back({static_cast<int>(i), dist});
            tempAdj[i].push_back({endIdx, dist});
            endConnections++;
        }
    }
    
    std::cout << "Start connects to " << startConnections << " points" << std::endl;
    std::cout << "End connects to " << endConnections << " points" << std::endl;
    
    // If either has no connections, no path exists
    if (startConnections == 0 || endConnections == 0) {
        return result;
    }
    
    // Dijkstra's algorithm
    std::vector<double> dist(tempVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<int> prev(tempVertices.size(), -1);
    std::priority_queue<std::pair<double, int>, 
                       std::vector<std::pair<double, int>>,
                       std::greater<std::pair<double, int>>> pq;
    
    dist[startIdx] = 0.0;
    pq.push({0.0, startIdx});
    
    while (!pq.empty()) {
        auto [currentDist, u] = pq.top();
        pq.pop();
        
        if (currentDist > dist[u]) continue;
        if (u == endIdx) break;
        
        for (const auto& [v, weight] : tempAdj[u]) {
            double newDist = dist[u] + weight;
            if (newDist < dist[v]) {
                dist[v] = newDist;
                prev[v] = u;
                pq.push({newDist, v});
            }
        }
    }
    
    if (dist[endIdx] < std::numeric_limits<double>::infinity()) {
        result.pathExists = true;
        result.totalLength = dist[endIdx];
        
        // Reconstruct path
        std::vector<Point_2> path;
        int current = endIdx;
        while (current != -1) {
            path.push_back(tempVertices[current]);
            current = prev[current];
        }
        std::reverse(path.begin(), path.end());
        result.pathPoints = path;
    }
    
    // THIRD: If still no path, try adding direct connection (emergency fallback)
    if (!result.pathExists) {
        std::cout << "Trying emergency fallback..." << std::endl;
        // This rarely happens with dense sampling
    }
    
    return result;
}

void PathFinder::clear() {
    vertices.clear();
    adjacencyList.clear();
    pointToIndex.clear();
    obstacles.clear();
}
