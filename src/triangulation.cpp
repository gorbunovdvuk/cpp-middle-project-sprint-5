//
// Created by Dmitry Gorbunov on 13.07.2025.
//

#include "triangulation.hpp"
#include <map>

namespace geometry::triangulation {

bool DelaunayTriangle::ContainsPoint(const Point2D &p) const {
    Point2D center = Circumcenter();
    double radius = Circumradius();
    return center.DistanceTo(p) <= radius + 1e-10;
}

Point2D DelaunayTriangle::Circumcenter() const {
    double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if (std::abs(d) < 1e-10) {
        return {(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3};
    }

    double ux = ((a.x * a.x + a.y * a.y) * (b.y - c.y) + (b.x * b.x + b.y * b.y) * (c.y - a.y) +
                 (c.x * c.x + c.y * c.y) * (a.y - b.y)) /
                d;

    double uy = ((a.x * a.x + a.y * a.y) * (c.x - b.x) + (b.x * b.x + b.y * b.y) * (a.x - c.x) +
                 (c.x * c.x + c.y * c.y) * (b.x - a.x)) /
                d;

    return {ux, uy};
}

double DelaunayTriangle::Circumradius() const {
    Point2D center = Circumcenter();
    return center.DistanceTo(a);
}

bool DelaunayTriangle::SharesEdge(const DelaunayTriangle &other) const {
    std::vector this_points = {a, b, c};
    std::vector other_points = {other.a, other.b, other.c};

    int shared_count = 0;
    for (const Point2D &p1 : this_points) {
        for (const Point2D &p2 : other_points) {
            if (std::abs(p1.x - p2.x) < 1e-10 && std::abs(p1.y - p2.y) < 1e-10) {
                shared_count++;
                break;
            }
        }
    }

    return shared_count == 2;
}

bool DelaunayTriangle::HasVertex(const Point2D &p) const {
    return std::ranges::any_of(std::array{a, b, c}, [&p](const auto& pt) {
        return std::abs(p.x - pt.x) < 1e-10 && std::abs(p.y - pt.y) < 1e-10;
    });
}

std::vector<DelaunayTriangle> DelaunayTriangulation(std::span<Point2D> points) {
    struct Edge {
        Point2D p1, p2;

        Edge(Point2D p1, Point2D p2) : p1(p1), p2(p2) {
            if (p1.x > p2.x + 1e-10 || (std::abs(p1.x - p2.x) < 1e-10 && p1.y > p2.y + 1e-10)) {
                std::swap(this->p1, this->p2);
            }
        }

        bool operator<(const Edge &other) const {
            if (std::abs(p1.x - other.p1.x) > 1e-10)
                return p1.x < other.p1.x;
            if (std::abs(p1.y - other.p1.y) > 1e-10)
                return p1.y < other.p1.y;
            if (std::abs(p2.x - other.p2.x) > 1e-10)
                return p2.x < other.p2.x;
            return p2.y < other.p2.y;
        }

        bool operator==(const Edge &other) const {
            return std::abs(p1.x - other.p1.x) < 1e-10 && std::abs(p1.y - other.p1.y) < 1e-10 &&
                   std::abs(p2.x - other.p2.x) < 1e-10 && std::abs(p2.y - other.p2.y) < 1e-10;
        }
    };

    std::ranges::sort(points, [](const auto &pt1, const auto &pt2) {
        if (std::abs(pt1.x - pt2.x) < 1e-10) {
            return pt1.y + 1e-10 < pt2.y;
        }
        return pt1.x + 1e-10 < pt2.x;
    });

    points = points.first(
        std::ranges::unique(points, [](const auto &pt1, const auto &pt2) { return pt1.Equals(pt2); }).begin() - points.begin()
    );

    if (points.size() < 3) {
        return {};
    }

    BoundingBox box{points};
    Point2D middle = box.Center();
    double dmx = std::max(box.Height(), box.Width()) * 10;

    Point2D pt1(middle.x - dmx, middle.y - dmx);
    Point2D pt2(middle.x, middle.y + dmx);
    Point2D pt3(middle.x + dmx, middle.y - dmx);

    std::vector<DelaunayTriangle> result = {{pt1, pt2, pt3}};

    for (const auto &pt : points) {
        std::vector<DelaunayTriangle> bad;
        std::map<Edge, size_t> edges;

        result.erase(std::ranges::remove_if(result,
                                            [&edges, &pt](const auto &triangle) {
                                                if (pt.DistanceTo(triangle.Circumcenter()) <
                                                    triangle.Circumradius() + 1e-10) {
                                                    ++edges[Edge{triangle.a, triangle.b}];
                                                    ++edges[Edge{triangle.b, triangle.c}];
                                                    ++edges[Edge{triangle.a, triangle.c}];
                                                    return true;
                                                }
                                                return false;
                                            })
                         .begin(),
                     result.end());
        for (const auto &[edge, count] : edges) {
            if (count == 1) {
                result.emplace_back(pt, edge.p1, edge.p2);
            }
        }
    }

    result.erase(std::ranges::remove_if(result,
                                        [&pt1, &pt2, &pt3](const auto &triangle) {
                                            return triangle.HasVertex(pt1) || triangle.HasVertex(pt2) ||
                                                   triangle.HasVertex(pt3);
                                        })
                     .begin(),
                 result.end());

    return result;
}

}  // namespace geometry::triangulation
