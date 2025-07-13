#pragma once
#include "geometry.hpp"
#include <vector>

namespace geometry::convex_hull {

std::vector<Point2D> GrahamScan(std::vector<Point2D> points);

}  // namespace geometry::convex_hull