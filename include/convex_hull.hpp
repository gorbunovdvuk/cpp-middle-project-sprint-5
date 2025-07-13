#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <range/v3/view/adjacent_filter.hpp>
#include <ranges>
#include <stack>
#include <vector>

namespace geometry::convex_hull {

std::vector<Point2D> GrahamScan(std::vector<Point2D> points);

}  // namespace geometry::convex_hull