#pragma once

#include "geometry.hpp"
#include "triangulation.hpp"
#include <span>

namespace geometry::visualization {

void Draw(std::span<Shape> shapes);

void Draw(std::span<const triangulation::DelaunayTriangle> triangles);

}  // namespace geometry::visualization
