#include "geometry.h"
#include <algorithm>
#include <cmath>
#include <CGAL/Boolean_set_operations_2.h>

bool GeometryUtils::isPointInsidePolygon(const Point_2& point, const Polygon_2& polygon) {
    CGAL::Bounded_side side = polygon.bounded_side(point);
    return (side == CGAL::ON_BOUNDED_SIDE || side == CGAL::ON_BOUNDARY);
}

bool GeometryUtils::isPointInObstacle(const Point_2& point, const std::vector<Polygon_2>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (isPointInsidePolygon(point, obstacle)) {
            return true;
        }
    }
    return false;
}

bool GeometryUtils::segmentIntersectsPolygon(const Segment_2& segment, const Polygon_2& polygon) {
    // Check if either endpoint is inside the polygon
    if (polygon.bounded_side(segment.source()) == CGAL::ON_BOUNDED_SIDE ||
        polygon.bounded_side(segment.target()) == CGAL::ON_BOUNDED_SIDE) {
        return true;
    }
    
    // Check if the segment is COMPLETELY INSIDE the polygon
    Point_2 mid(
        (segment.source().x() + segment.target().x()) / 2,
        (segment.source().y() + segment.target().y()) / 2
    );
    
    if (polygon.bounded_side(mid) == CGAL::ON_BOUNDED_SIDE) {
        return true;
    }
    
    // Check intersection with polygon edges (proper intersection, not just touching)
    auto vertices = polygon.container();
    
    for (size_t i = 0; i < vertices.size(); i++) {
        const Point_2& p1 = vertices[i];
        const Point_2& p2 = vertices[(i + 1) % vertices.size()];
        Segment_2 edge(p1, p2);
        
        // Skip if segment shares endpoints with edge (allows boundary traversal)
        if (segment.source() == p1 || segment.source() == p2 ||
            segment.target() == p1 || segment.target() == p2) {
            continue;
        }
        
        if (CGAL::do_intersect(segment, edge)) {
            auto result = CGAL::intersection(segment, edge);
            if (const Point_2* p = std::get_if<Point_2>(&*result)) {
                // Check if it's a proper intersection (not at endpoints)
                if (*p != segment.source() && *p != segment.target() &&
                    *p != p1 && *p != p2) {
                    return true;
                }
            } else if (std::get_if<Segment_2>(&*result)) {
                // Segments overlap (collinear)
                return true;
            }
        }
    }
    
    return false;
}

bool GeometryUtils::isSegmentVisible(const Segment_2& segment, const std::vector<Polygon_2>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (segmentIntersectsPolygon(segment, obstacle)) {
            return false;
        }
    }
    return true;
}

std::vector<Segment_2> GeometryUtils::computeVisibilityGraph(
    const std::vector<Point_2>& points,
    const std::vector<Polygon_2>& obstacles) {
    
    std::vector<Segment_2> visibleEdges;
    
    for (size_t i = 0; i < points.size(); i++) {
        for (size_t j = i + 1; j < points.size(); j++) {
            Segment_2 seg(points[i], points[j]);
            if (isSegmentVisible(seg, obstacles)) {
                visibleEdges.push_back(seg);
            }
        }
    }
    
    return visibleEdges;
}

Polygon_2 GeometryUtils::simplifyPolygon(const Polygon_2& polygon) {
    std::vector<Point_2> simplified;
    auto vertices = polygon.container();
    
    if (vertices.size() < 3) return polygon;
    
    for (size_t i = 0; i < vertices.size(); i++) {
        const Point_2& prev = vertices[(i - 1 + vertices.size()) % vertices.size()];
        const Point_2& curr = vertices[i];
        const Point_2& next = vertices[(i + 1) % vertices.size()];
        
        if (CGAL::orientation(prev, curr, next) != CGAL::COLLINEAR) {
            simplified.push_back(curr);
        }
    }
    
    return Polygon_2(simplified.begin(), simplified.end());
}

FreeSpace::FreeSpace(const Polygon_2& bbox, const std::vector<Polygon_2>& obs) 
    : boundingBox(bbox), obstacles(obs) {
    for (const auto& obstacle : obstacles) {
        for (auto it = obstacle.vertices_begin(); it != obstacle.vertices_end(); ++it) {
            visibilityPoints.push_back(*it);
        }
    }
}

void FreeSpace::addObstacle(const Polygon_2& obstacle) {
    obstacles.push_back(obstacle);
    for (auto it = obstacle.vertices_begin(); it != obstacle.vertices_end(); ++it) {
        visibilityPoints.push_back(*it);
    }
}

bool FreeSpace::isPointFree(const Point_2& point) const {
    if (boundingBox.bounded_side(point) == CGAL::ON_UNBOUNDED_SIDE) {
        return false;
    }
    
    return !GeometryUtils::isPointInObstacle(point, obstacles);
}

bool FreeSpace::isPathFree(const Segment_2& path) const {
    if (!isPointFree(path.source()) || !isPointFree(path.target())) {
        return false;
    }
    
    return GeometryUtils::isSegmentVisible(path, obstacles);
}

std::vector<Point_2> FreeSpace::getVisibilityPoints() const {
    return visibilityPoints;
}
