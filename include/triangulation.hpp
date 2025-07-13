#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <format>
#include <set>
#include <vector>
#include <map>

namespace geometry::triangulation {

struct DelaunayTriangle {
    Point2D a, b, c;

    DelaunayTriangle(Point2D a, Point2D b, Point2D c) : a(a), b(b), c(c) {}

    bool ContainsPoint(const Point2D &p) const;

    Point2D Circumcenter() const;

    double Circumradius() const;

    bool SharesEdge(const DelaunayTriangle &other) const;

    std::vector<Point2D> vertices() const { return {a, b, c}; }
};

std::vector<DelaunayTriangle> DelaunayTriangulation(std::vector<Point2D> points);

}  // namespace geometry::triangulation

template <>
struct std::formatter<geometry::triangulation::DelaunayTriangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::triangulation::DelaunayTriangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "DelaunayTriangle({}, {}, {})", t.a, t.b, t.c);
    }
};
