//
// Created by Dmitry Gorbunov on 13.07.2025.
//

#include "intersections.hpp"

namespace geometry::intersections {

std::vector<Point2D> IntersectionVisitor::operator()(const LineSegment &segment1, const LineSegment &segment2) const {
    double a1 = (segment1.pt2 - segment1.pt1).NormalVector().x, b1 = (segment1.pt2 - segment1.pt1).NormalVector().y;
    double a2 = (segment2.pt2 - segment2.pt1).NormalVector().x, b2 = (segment2.pt2 - segment2.pt1).NormalVector().y;
    double c1 = -(a1 * segment1.pt1.x + b1 * segment1.pt1.y), c2 = -(a2 * segment2.pt1.x + b2 * segment2.pt1.y);
    double det = a1 * b2 - a2 * b1;
    if (std::abs(det) < 1e-10) {
        return {};
    }
    double dx = -(c1 * b2 - c2 * b1), dy = -(a1 * c2 - a2 * c1);
    Point2D intersection{dx / det, dy / det};
    if (segment1.Contains(intersection) && segment2.Contains(intersection)) {
        return {intersection};
    }
    return {};
}
std::vector<Point2D> IntersectionVisitor::operator()(const LineSegment &segment, const Circle &circle) const {
    Point2D direction = segment.pt2 - segment.pt1;
    Point2D center_vector = segment.pt1 - circle.center;

    double a = direction.Length2();
    double b = 2 * center_vector.DotProduct(direction);
    double c = center_vector.Length2() - circle.radius * circle.radius;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < -1e-10) {
        return {};
    }

    std::vector<Point2D> intersections;

    if (std::abs(discriminant) < 1e-10) {
        intersections.emplace_back(segment.pt1 + direction * (-b / (2 * a)));
    } else {
        double sqrt_discriminant = std::sqrt(discriminant);
        intersections.emplace_back(segment.pt1 + direction * (-b - sqrt_discriminant) / (2 * a));
        intersections.emplace_back(segment.pt1 + direction * (-b + sqrt_discriminant) / (2 * a));
    }

    intersections.erase(
        std::ranges::remove_if(intersections, [&segment](const auto &pt) { return !segment.Contains(pt); }).begin(),
        intersections.end());

    return intersections;
}
std::vector<Point2D> IntersectionVisitor::operator()(const Circle &circle1, const Circle &circle2) const {
    double d = (circle2.center - circle1.center).Length();
    if (d > circle2.radius + circle1.radius + 1e-10) {
        return {};
    }
    if (d < 1e-10 && std::abs(circle1.radius - circle2.radius) < 1e-10) {
        return {};
    }
    double x = (d * d + circle1.radius * circle1.radius - circle2.radius * circle2.radius) / (2 * d);
    double y = std::sqrt(circle1.radius * circle1.radius - x * x);
    Point2D middle = circle1.center + (circle2.center - circle1.center) / d * x;
    if (std::abs(y) < 1e-10) {
        return {middle};
    }
    Point2D norm = ((circle2.center - circle1.center) / d).NormalVector();
    return {middle - norm * y, middle + norm * y};
}

std::optional<std::vector<Point2D>> GetIntersectionPoints(const Shape &shape1, const Shape &shape2) {
    try {
        return std::visit(IntersectionVisitor{}, shape1, shape2);
    } catch (std::logic_error &e) {
        return std::nullopt;
    }
}

}  // namespace geometry::intersections
