#pragma once
#include "geometry.hpp"
#include <optional>

namespace geometry::intersections {

class IntersectionVisitor {
public:
    std::vector<Point2D> operator()(const LineSegment &segment1, const LineSegment &segment2);

    std::vector<Point2D> operator()(const LineSegment &segment, const Circle &circle);

    std::vector<Point2D> operator()(const Circle &circle, const LineSegment &segment) {
        return this->operator()(segment, circle);
    }

    std::vector<Point2D> operator()(const Circle &circle1, const Circle &circle2);

    std::vector<Point2D> operator()(const auto &, const auto &) { throw std::logic_error("Unsupported intersection"); }
};

std::optional<std::vector<Point2D>> GetIntersectionPoints(const Shape &shape1, const Shape &shape2);

}  // namespace geometry::intersections
