#pragma once
#include "geometry.hpp"
#include "queries.hpp"
#include <random>
#include <range/v3/all.hpp>
#include <ranges>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    explicit ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0,
                            double max_size = 20.0)
        : gen(20), angle_dist(0, 2 * M_PI), coord_dist(min_coord, max_coord), size_dist(min_size, max_size),
          sides_dist(3, 12), type_dist(0, 4) {}

    Shape GenerateRandomShape() {
        Point2D center{coord_dist(gen), coord_dist(gen)};
        double size = size_dist(gen);

        switch (type_dist(gen)) {
        case 0: {
            double angle = angle_dist(gen);
            Point2D end{center.x + size * std::sin(angle), center.y + size * std::cos(angle)};
            return LineSegment{center, end};
        }
        case 1: {
            double angle = angle_dist(gen);
            Point2D a{center.x, center.y};
            Point2D b{center.x + size * std::sin(angle), center.y};
            Point2D c{center.x + size * std::sin(angle) / 2, center.y + size * std::cos(angle)};
            return Triangle{a, b, c};
        }
        case 2: {
            return Rectangle{center, size, size * 0.8};
        }
        case 3: {
            int sides = sides_dist(gen);
            return RegularPolygon{center, size, sides};
        }
        case 4: {
            return Circle{center, size};
        }
        }
        return Circle{center, size};
    }

    std::vector<Shape> GenerateShapes(size_t count) {
        std::vector<Shape> shapes;
        shapes.reserve(count);

        for (auto _ : std::views::iota(0u, count)) {
            shapes.push_back(GenerateRandomShape());
        }

        return shapes;
    }

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> angle_dist;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

inline std::vector<std::tuple<Shape, Shape>> FindAllCollisions(const std::vector<Shape> &shapes) {
    return std::vector{
        std::from_range,
        ranges::views::cartesian_product(shapes, shapes) |
            ranges::views::filter([](const auto &shape_pair) {
                const auto &[shape1, shape2] = shape_pair;
                return queries::BoundingBoxesOverlap(shape1, shape2);
            })
    };
}

inline std::optional<size_t> FindUnusuallyHighestShape(const std::vector<Shape>& shapes) {
    if (shapes.empty()) {
        return std::nullopt;
    }
    return ranges::max_element(shapes, std::less<>{}, [](const auto& shape) {
        return queries::GetUnusualHeight(shape);
    }) - shapes.begin();
}

}  // namespace geometry::utils