#pragma once
#include "geometry.hpp"
#include <optional>

namespace geometry::intersections {

class IntersectionVisitor {
public:
    GeometryResult<std::vector<Point2D>> operator()(const LineSegment &segment1, const LineSegment &segment2) const;

    GeometryResult<std::vector<Point2D>> operator()(const LineSegment &segment, const Circle &circle) const;

    GeometryResult<std::vector<Point2D>> operator()(const Circle &circle, const LineSegment &segment) const {
        return this->operator()(segment, circle);
    }

    GeometryResult<std::vector<Point2D>> operator()(const Circle &circle1, const Circle &circle2) const;

    GeometryResult<std::vector<Point2D>> operator()(const auto &, const auto &) const { throw std::logic_error("Unsupported intersection"); }
};

GeometryResult<std::vector<Point2D>> GetIntersectionPoints(const Shape &shape1, const Shape &shape2);

}  // namespace geometry::intersections
