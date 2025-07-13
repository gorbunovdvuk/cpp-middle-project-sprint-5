#include "convex_hull.hpp"
#include "geometry.hpp"
#include "intersections.hpp"
#include "queries.hpp"
#include "shape_utils.hpp"
#include "triangulation.hpp"
#include "visualization.hpp"

#include <algorithm>
#include <iostream>
#include <print>
#include <range/v3/all.hpp>
#include <ranges>

using namespace geometry;

void PrintAllIntersections(const Shape &shape, const std::vector<Shape>& others) {
    std::visit([](const auto& shape) {
        std::println("\n=== Intersections for {}===", shape);
    }, shape);
    std::ranges::for_each(others | std::views::filter([&shape](const auto& other) { return Distinct(shape, other); }) | std::views::transform([&shape](const auto& other) {
        return intersections::GetIntersectionPoints(shape, other).transform([&shape, &other](const auto& pts) {
            return std::tuple{shape, other, pts};
        });
    }), [](const auto& intersection) {
        intersection.and_then([](const auto& tuple) {
            const auto& [shape1, shape2, pts] = tuple;
            std::visit([&pts](const auto& shape1, const auto& shape2) {
                std::println("Intersections for {} and {} are {}", shape1, shape2, pts);
            }, shape1, shape2);
            return std::optional{0};
        });
    });
}

void PrintDistancesFromPointToShapes(Point2D p, std::vector<Shape> shapes) {
    std::println("\n=== Distance from Point Test ===");
    std::println("Testing point: {} ", p);

    std::vector<Shape> filtered;

    ranges::sample(shapes, std::back_inserter(filtered), 5);

    for (const auto& shape : filtered) {
        std::visit([&p](const auto& shape) {
            std::println("Distance from {} to {} is {}", p, shape, queries::DistanceToPoint(shape, p));
        }, shape);
    }
}

void PerformShapeAnalysis(std::vector<Shape> shapes) {
    std::println("\n=== Shape Analysis ===");

    for (const auto& [shape1, shape2] : utils::FindAllCollisions(shapes)) {
        std::visit([](const auto& shape1, const auto& shape2) {
            std::println("Found intersection between BoundingBoxes of {} and {}", shape1, shape2);
        }, shape1, shape2);
    }

    auto highest_shape = std::ranges::max(shapes, std::less<>{}, &queries::GetUnusualHeight);

    std::visit([](const auto& shape) {
        std::println("The highest shape is {} with height {}", shape, queries::GetUnusualHeight(shape));
    }, highest_shape);

    const auto [shape1, shape2] = (ranges::views::cartesian_product(shapes, shapes) | ranges::views::filter([](const auto& shapes_tuple) {
        const auto &[shape1, shape2] = shapes_tuple;
        return Distinct(shape1, shape2) && queries::DistanceBetweenShapes(shape1, shape2).has_value();
    }) | ranges::views::take(1)).front();

    std::visit([](const auto& shape1, const auto& shape2) {
        std::println("Distance between {} and {} is {}", shape1, shape2, queries::DistanceBetweenShapes(shape1, shape2).value());
    }, shape1, shape2);
}

void PerformExtraShapeAnalysis(std::vector<Shape> shapes) {
    std::println("\n=== Shape Extra Analysis ===");

    for (const auto& shape : shapes | std::views::filter([](const auto& shape) { return queries::GetUnusualHeight(shape) > 50.0; }) | std::views::take(3)) {
        std::visit([](const auto& shape) {
            std::println("Found shape {} with height > 50", shape);
        }, shape);
    }

    auto min_height_shape = std::ranges::min(shapes, std::less<>{}, &queries::GetUnusualHeight);
    auto max_height_shape = std::ranges::max(shapes, std::less<>{}, &queries::GetUnusualHeight);

    std::visit([](const auto& shape) {
        std::println("Shape with minimum height is {}, height: {}", shape, queries::GetUnusualHeight(shape));
    }, min_height_shape);

    std::visit([](const auto& shape) {
        std::println("Shape with maximum height is {}, height: {}", shape, queries::GetUnusualHeight(shape));
    }, max_height_shape);
}

int main() {
    utils::ShapeGenerator generator(-50.0, 50.0, 5.0, 25.0);
    std::vector shapes = generator.GenerateShapes(15);

    std::println("Generated {} random shapes", shapes.size());

    for (const auto& [index, shape] : ranges::views::enumerate(shapes)) {
        std::visit([&index](const auto& shape) {
            std::println("Shape[{}]: {}, Height: {:.2f}", index, shape, shape.UnusualHeight());
        }, shape);
    }

    for (const auto& shape : shapes) {
        PrintAllIntersections(shape, shapes);
    }

    PrintDistancesFromPointToShapes(Point2D{10.0, 10.0}, shapes);

    PerformShapeAnalysis(shapes);

    PerformExtraShapeAnalysis(shapes);

    std::vector shapes_points(std::from_range, shapes | std::views::transform([](const auto& shape) {
        return std::visit([](const auto& shape) {
            return std::vector{std::from_range, shape.Vertices()};
        }, shape);
    }) | std::views::join);

    std::vector convex_hull = convex_hull::GrahamScan(shapes_points);
    shapes.emplace_back(Polygon{convex_hull});

    visualization::Draw(shapes);
    auto triangulation = triangulation::DelaunayTriangulation(convex_hull);

    visualization::Draw(triangulation);
    return 0;
}