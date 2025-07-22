#pragma once
#include "geometry.hpp"
#include <format>
#include <vector>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    [[nodiscard]] bool ContainsPoint(const Point2D &p) const;
    [[nodiscard]] Point2D Circumcenter() const;
    [[nodiscard]] double Circumradius() const;
    [[nodiscard]] bool SharesEdge(const DelaunayTriangle &other) const;
    [[nodiscard]] std::vector<Point2D> vertices() const { return {a, b, c}; }
    [[nodiscard]] bool HasVertex(const Point2D &p) const;
};

std::vector<DelaunayTriangle> DelaunayTriangulation(std::span<Point2D> points);

}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
