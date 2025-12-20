#pragma once

#include "geometry.h"
#include <queue>
#include <unordered_map>
#include <functional>
#include <set>
#include <cmath>

struct PathFinderResult {
    bool pathExists;
    std::vector<Point_2> pathPoints;
    double totalLength;
};

// Hash function for CGAL::Point_2
struct Point2Hash {
    std::size_t operator()(const Point_2& p) const {
        double x = CGAL::to_double(p.x());
        double y = CGAL::to_double(p.y());
        return std::hash<double>{}(x) ^ (std::hash<double>{}(y) << 1);
    }
};

class PathFinder {
private:
    std::vector<Point_2> vertices;
    std::vector<std::vector<std::pair<int, double>>> adjacencyList;
    std::unordered_map<Point_2, int, Point2Hash> pointToIndex;

    std::vector<Polygon_2> obstacles;
    
    double computeDistance(const Point_2& p1, const Point_2& p2);
    bool isVisible(const Point_2& p1, const Point_2& p2, 
                  const std::vector<Polygon_2>& obstacles);
    
public:
    PathFinder();
    
    void buildGraph(const std::vector<Point_2>& points, 
                   const std::vector<Polygon_2>& obstacles);
    
    PathFinderResult findPath(const Point_2& start, const Point_2& end);
    
    void clear();
};
