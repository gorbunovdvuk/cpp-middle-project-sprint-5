#pragma once
#include "geometry.hpp"
#include "intersections.hpp"
#include "type_traits.hpp"

#include <algorithm>
#include <optional>
#include <variant>

namespace geometry::queries {

template <class... Ts>
struct Overloaded : Ts... {
    explicit Overloaded(Ts&&... args): Ts(std::move(args))... {}

    using Ts::operator()...;
};

struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double operator()(const LineSegment& segment) const {
        Point2D segment_vector = segment.pt2 - segment.pt1;
        Point2D vector_to_point = point - segment.pt1;
        double len2 = segment_vector.Length2();
        if (std::abs(len2) < 1e-10) {
            return point.DistanceTo(segment.pt1);
        }
        double fraq2 = vector_to_point.DotProduct(segment_vector);
        if (fraq2 < 1e-10) {
            return point.DistanceTo(segment.pt1);
        }
        if (fraq2 + 1e-10 > len2) {
            return point.DistanceTo(segment.pt2);
        }
        return point.DistanceTo(segment.pt1 + segment_vector * fraq2 / len2);
    }

    template<typename Shape>
    requires traits::is_one_of<Shape, Triangle, Rectangle, RegularPolygon, Polygon>
    double operator()(const Shape& shape) const {
        return std::ranges::min(shape.Edges() | std::views::transform([this](const LineSegment& segment) {
            return (*this)(segment);
        }));
    }

    double operator()(const Circle& circle) const {
        return std::abs(point.DistanceTo(circle.center) - circle.radius);
    }
};

struct ShapeToShapeDistanceVisitor {
    template<typename Shape>
    double operator()(const Point2D& point, const Shape& shape) const {
        return PointToShapeDistanceVisitor(point)(shape);
    }

    template<typename Shape>
    double operator()(const Shape& shape, const Point2D& point) const {
        return PointToShapeDistanceVisitor(point)(shape);
    }

    std::optional<double> operator()(const LineSegment& segment1, const LineSegment& segment2) const {
        if (!intersections::IntersectionVisitor{}(segment1, segment2).empty()) {
            return 0;
        }
        return std::min(PointToShapeDistanceVisitor(segment1.pt1)(segment2), PointToShapeDistanceVisitor(segment1.pt2)(segment2));
    }

    std::optional<double> operator()(const Circle& circle1, const Circle& circle2) const {
        if (!intersections::IntersectionVisitor{}(circle1, circle2).empty()) {
            return 0;
        }
        return std::abs(circle1.center.DistanceTo(circle2.center) - circle1.radius - circle2.radius);
    }
};

inline double DistanceToPoint(const Shape &shape, const Point2D &point) {
    return std::visit(PointToShapeDistanceVisitor{point}, shape);
}

inline BoundingBox GetBoundBox(const Shape &shape) {
    return std::visit([](const auto& shape) {
        return shape.BoundBox();
    }, shape);
}

inline double GetUnusualHeight(const Shape &shape) {
    return std::visit([](const auto& shape) {
        return shape.UnusualHeight();
    }, shape);
}

inline bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2) {
    BoundingBox box1 = GetBoundBox(shape1), box2 = GetBoundBox(shape2);
    return !(box1.max_x < box2.min_x || box2.max_x < box1.min_x || box1.max_y < box2.min_y || box1.min_y > box2.max_y);
}

std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2) {
    return std::visit(Overloaded{
        ShapeToShapeDistanceVisitor{},
        [](const auto&, const auto&) -> std::optional<double> { return std::nullopt; }
    }, shape1, shape2);
}

}  // namespace geometry::queries