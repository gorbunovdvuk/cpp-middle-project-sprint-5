#pragma once
#include "geometry.hpp"
#include "type_traits.hpp"

#include <algorithm>
#include <optional>

namespace geometry::queries {

template <class... Ts>
struct Overloaded : Ts... {
    explicit Overloaded(Ts &&...args) : Ts(std::move(args))... {}

    using Ts::operator()...;
};

struct PointToShapeDistanceVisitor {
    Point2D point;

    explicit PointToShapeDistanceVisitor(const Point2D &p) : point(p) {}

    double operator()(const LineSegment &segment) const;

    template <typename Shape>
        requires traits::is_one_of<Shape, Triangle, Rectangle, RegularPolygon, Polygon>
    double operator()(const Shape &shape) const {
        return std::ranges::min(shape.Edges() |
                                std::views::transform([this](const LineSegment &segment) { return (*this)(segment); }));
    }

    double operator()(const Circle &circle) const { return std::abs(point.DistanceTo(circle.center) - circle.radius); }
};

struct ShapeToShapeDistanceVisitor {
    std::optional<double> operator()(const auto&, const auto&) const { return std::nullopt; }

    template <typename Shape>
    std::optional<double> operator()(const Point2D &point, const Shape &shape) const {
        return PointToShapeDistanceVisitor(point)(shape);
    }

    template <typename Shape>
    std::optional<double> operator()(const Shape &shape, const Point2D &point) const {
        return PointToShapeDistanceVisitor(point)(shape);
    }

    std::optional<double> operator()(const LineSegment &segment1, const LineSegment &segment2) const;

    std::optional<double> operator()(const Circle &circle1, const Circle &circle2) const;
};

double DistanceToPoint(const Shape &shape, const Point2D &point);

BoundingBox GetBoundBox(const Shape &shape);

double GetUnusualHeight(const Shape &shape);

bool BoundingBoxesOverlap(const Shape &shape1, const Shape &shape2);

std::optional<double> DistanceBetweenShapes(const Shape &shape1, const Shape &shape2);

}  // namespace geometry::queries