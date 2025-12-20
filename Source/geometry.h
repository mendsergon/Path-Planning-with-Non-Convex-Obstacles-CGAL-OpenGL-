#pragma once

#include <vector>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::FT FT;
typedef CGAL::Polygon_2<Kernel> Polygon_2;

class GeometryUtils {
public:
    static bool isPointInsidePolygon(const Point_2& point, const Polygon_2& polygon);
    static bool isPointInObstacle(const Point_2& point, const std::vector<Polygon_2>& obstacles);
    static bool segmentIntersectsPolygon(const Segment_2& segment, const Polygon_2& polygon);
    static bool isSegmentVisible(const Segment_2& segment, const std::vector<Polygon_2>& obstacles);
    static std::vector<Segment_2> computeVisibilityGraph(const std::vector<Point_2>& points,
                                                       const std::vector<Polygon_2>& obstacles);
    static Polygon_2 simplifyPolygon(const Polygon_2& polygon);
};

class FreeSpace {
private:
    Polygon_2 boundingBox;
    std::vector<Polygon_2> obstacles;
    std::vector<Point_2> visibilityPoints;
    
public:
    FreeSpace(const Polygon_2& bbox, const std::vector<Polygon_2>& obs);
    void addObstacle(const Polygon_2& obstacle);
    bool isPointFree(const Point_2& point) const;
    bool isPathFree(const Segment_2& path) const;
    std::vector<Point_2> getVisibilityPoints() const;
    const Polygon_2& getBoundingBox() const { return boundingBox; }
    const std::vector<Polygon_2>& getObstacles() const { return obstacles; }
};
