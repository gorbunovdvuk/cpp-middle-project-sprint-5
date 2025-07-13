#pragma once
#include "geometry.hpp"
#include "queries.hpp"
#include <random>
#include <vector>

namespace geometry::utils {

class ShapeGenerator {
public:
    explicit ShapeGenerator(double min_coord = -100.0, double max_coord = 100.0, double min_size = 1.0,
                            double max_size = 20.0);

    Shape GenerateRandomShape();

    std::vector<Shape> GenerateShapes(size_t count);

private:
    std::mt19937 gen;
    std::uniform_real_distribution<double> angle_dist;
    std::uniform_real_distribution<double> coord_dist;
    std::uniform_real_distribution<double> size_dist;
    std::uniform_int_distribution<int> sides_dist;
    std::uniform_int_distribution<int> type_dist;
};

std::vector<std::tuple<Shape, Shape>> FindAllCollisions(std::span<const Shape> shapes);

std::optional<size_t> FindUnusuallyHighestShape(std::span<const Shape> shapes);

}  // namespace geometry::utils