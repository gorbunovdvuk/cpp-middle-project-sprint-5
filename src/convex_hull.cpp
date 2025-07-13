#include "convex_hull.hpp"
#include <algorithm>
#include <range/v3/all.hpp>

namespace geometry::convex_hull {

template <class T, class Container = std::vector<T>>
class StackForGrahamScan {
public:
    using container_type = Container;
    using value_type = typename container_type::value_type;
    using size_type = typename container_type::size_type;
    using reference = typename container_type::reference;
    using const_reference = typename container_type::const_reference;

    StackForGrahamScan() = default;

    void push(const Point2D &point) { container_.push_back(point); }

    void pop() { container_.pop_back(); }

    const_reference top() const { return *container_.rbegin(); }

    const_reference pre_top() const { return *std::next(container_.rbegin()); }

    void reserve(size_type N) { container_.reserve(N); }

    size_type size() const { return container_.size(); }

    const Container &container() const { return container_; }

private:
    Container container_;
};

std::vector<Point2D> GrahamScan(std::vector<Point2D> points) {
    std::ranges::sort(points, [](const Point2D &p1, const Point2D &p2) {
        if (std::abs(p1.x - p2.x) < 1e-10) {
            return p1.y + 1e-10 < p2.y;
        }
        return p1.x + 1e-10 < p2.x;
    });
    points.erase(
        std::ranges::unique(points, [](const Point2D &p1, const Point2D &p2) { return p1.Equals(p2); }).begin(),
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