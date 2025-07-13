#include "convex_hull.hpp"
#include <algorithm>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2) {
    auto new_p1 = p1 - middle;
    auto new_p2 = p2 - middle;
    return new_p1.CrossProduct(new_p2);
}

std::vector<Point2D> GrahamScan(std::vector<Point2D> points) {
    std::ranges::sort(points, [](const Point2D &p1, const Point2D &p2) {
        if (std::abs(p1.x - p2.x) < 1e-10) {
            return p1.y + 1e-10 < p2.y;
        }
        return p1.x + 1e-10 < p2.x;
    });
    points.erase(
        std::unique(points.begin(), points.end(), [](const Point2D &p1, const Point2D &p2) { return p1.Equals(p2); }),
        points.end());

    const auto pt0 = points.front();
    std::ranges::sort(points.begin() + 1, points.end(), [&pt0](const Point2D &pt1, const Point2D &pt2) {
        double cr = (pt1 - pt0).CrossProduct(pt2 - pt0);
        if (std::abs(cr) < 1e-10) {
            return pt0.DistanceTo(pt1) > pt0.DistanceTo(pt2);
        }
        return cr > 0;
    });

    std::vector filtered{std::from_range,
                         ranges::views::adjacent_filter(points, [&pt0](const Point2D &pt1, const Point2D &pt2) {
                             return std::abs((pt1 - pt0).CrossProduct(pt2 - pt0)) > 1e-10;
                         })};

    StackForGrahamScan<Point2D> stack;
    stack.reserve(filtered.size());
    stack.push(filtered[0]);
    stack.push(filtered[1]);
    for (size_t i = 2; i < filtered.size(); ++i) {
        while (stack.size() >= 2 && (stack.top() - stack.pre_top()).CrossProduct(filtered[i] - stack.top()) < 1e-10) {
            stack.pop();
        }
        stack.push(filtered[i]);
    }

    return stack.container();
}

}  // namespace geometry::convex_hull