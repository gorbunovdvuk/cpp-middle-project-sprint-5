#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <ranges>
#include <stack>
#include <vector>

namespace geometry::convex_hull {

double CrossProduct(Point2D p1, Point2D middle, Point2D p2);

template<template<typename> typename Underlying = std::vector>
class StackForGrahamScan {
public:
    StackForGrahamScan() = default;

    void push_back(const Point2D& point) {
        stack_.push_back(point);
    }

    void pop_back() {
        stack_.pop_back();
    }

    const Point2D& back() const {
        return *stack_.rbegin();
    }

    const Point2D& pre_back() const {
        return *std::next(stack_.rbegin());
    }

    void reserve(size_t N) {
        stack_.reserve(N);
    }

    size_t size() const {
        return stack_.size();
    }

    Underlying<Point2D> underlying() const {
        return stack_;
    }

private:
    Underlying<Point2D> stack_;
};

std::vector<Point2D> GrahamScan(std::vector<Point2D> points) {
    std::ranges::sort(points, [](const Point2D& p1, const Point2D& p2) {
        if (std::abs(p1.x - p2.x) < 1e-10) {
            return p1.y + 1e-10 < p2.y;
        }
        return p1.x + 1e-10 < p2.x;
    });
    points.erase(std::unique(points.begin(), points.end(), [](const Point2D& p1, const Point2D& p2) {
        return p1.Equals(p2);
    }), points.end());

    const auto pt0 = points.front();
    std::ranges::sort(points.begin() + 1, points.end(), [&pt0](const Point2D& pt1, const Point2D& pt2) {
        double cr = (pt1 - pt0).CrossProduct(pt2 - pt0);
        if (std::abs(cr) < 1e-10) {
            return pt0.DistanceTo(pt1) > pt0.DistanceTo(pt2);
        }
        return cr > 0;
    });

    std::vector<Point2D> filtered;
    filtered.reserve(points.size());
    filtered.push_back(pt0);
    for (size_t i = 1; i < points.size();) {
        size_t j = i + 1;
        while (j < points.size() && std::abs((points[j] - pt0).CrossProduct(points[j - 1] - pt0)) < 1e-10) {
            j++;
        }
        filtered.push_back(points[i]);
        i = j;
    }

    StackForGrahamScan stack;
    stack.reserve(filtered.size());
    stack.push_back(filtered[0]);
    stack.push_back(filtered[1]);
    for (size_t i = 2; i < filtered.size(); ++i) {
        while (stack.size() >= 2 && (stack.back() - stack.pre_back()).CrossProduct(filtered[i] - stack.back()) < 1e-10) {
            stack.pop_back();
        }
        stack.push_back(filtered[i]);
    }

    return stack.underlying();
}

}  // namespace geometry::convex_hull