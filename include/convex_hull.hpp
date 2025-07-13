#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <range/v3/view/adjacent_filter.hpp>
#include <ranges>
#include <stack>
#include <vector>

namespace geometry::convex_hull {

template<class T, class Container = std::vector<T>>
class StackForGrahamScan {
public:
    using container_type = Container;
    using value_type = typename container_type::value_type;
    using size_type = typename container_type::size_type;
    using reference = typename container_type::reference;
    using const_reference = typename container_type::const_reference;

    StackForGrahamScan() = default;

    void push(const Point2D& point) {
        container_.push_back(point);
    }

    void pop() {
        container_.pop_back();
    }

    const_reference top() const {
        return *container_.rbegin();
    }

    const_reference pre_top() const {
        return *std::next(container_.rbegin());
    }

    void reserve(size_type N) {
        container_.reserve(N);
    }

    size_type size() const {
        return container_.size();
    }

    const Container& container() const {
        return container_;
    }

private:
    Container container_;
};

std::vector<Point2D> GrahamScan(std::vector<Point2D> points);

}  // namespace geometry::convex_hull