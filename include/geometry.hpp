#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <numbers>
#include <ranges>
#include <variant>
#include <vector>
#include <expected>

namespace geometry {

struct Point2D {
    double x, y;

    constexpr Point2D() : x(0), y(0) {}
    constexpr Point2D(double x, double y) : x(x), y(y) {}

    bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }

    // Binary math operators
    Point2D operator+(const Point2D &other) const { return {x + other.x, y + other.y}; }
    Point2D operator-(const Point2D &other) const { return {x - other.x, y - other.y}; }
    Point2D operator*(double value) const { return {x * value, y * value}; }
    Point2D operator/(double value) const { return {x / value, y / value}; }

    // Binary geometry operations
    [[nodiscard]] double DotProduct(const Point2D &other) const { return x * other.x + y * other.y; }
    [[nodiscard]] double CrossProduct(const Point2D &other) const { return x * other.y - y * other.x; }
    [[nodiscard]] double Length() const { return std::sqrt(Length2()); }
    [[nodiscard]] double Length2() const { return x * x + y * y; }
    [[nodiscard]] double DistanceTo(const Point2D &other) const { return (*this - other).Length(); }

    [[nodiscard]] Point2D Normalize() const {
        const double len = Length();
        return len > 0 ? Point2D{x / len, y / len} : Point2D{0, 0};
    }

    [[nodiscard]] Point2D NormalVector() const { return {-y, x}; }

    [[nodiscard]] bool Equals(const Point2D &other) const { return std::abs(x - other.x) < 1e-10 && std::abs(y - other.y) < 1e-10; }
};

struct BoundingBox {
    double min_x, min_y, max_x, max_y;

    constexpr BoundingBox(double x1, double y1, double x2, double y2)
        : min_x(std::min(x1, x2)), min_y(std::min(y1, y2)), max_x(std::max(x1, x2)), max_y(std::max(y1, y2)) {}

    template <std::ranges::range R>
    constexpr explicit BoundingBox(R &&pts)
        : BoundingBox(std::ranges::min(pts, std::less<>{}, [](const auto &pt) { return pt.x; }).x,
                      std::ranges::min(pts, std::less<>{}, [](const auto &pt) { return pt.y; }).y,
                      std::ranges::max(pts, std::less<>{}, [](const auto &pt) { return pt.x; }).x,
                      std::ranges::max(pts, std::less<>{}, [](const auto &pt) { return pt.y; }).y) {}

    constexpr BoundingBox(std::initializer_list<Point2D> pts)
        : BoundingBox(std::ranges::subrange(pts.begin(), pts.end())) {}

    [[nodiscard]] double Width() const { return max_x - min_x; }
    [[nodiscard]] double Height() const { return max_y - min_y; }

    [[nodiscard]] Point2D Center() const { return {(min_x + max_x) / 2, (min_y + max_y) / 2}; }

    [[nodiscard]] bool Contains(const Point2D &pt) const {
        return min_x - 1e-10 < pt.x && pt.x < max_x + 1e-10 && min_y - 1e-10 < pt.y && pt.y < max_y + 1e-10;
    }

    [[nodiscard]] bool Overlaps(const BoundingBox &other) const {
        return !(max_x < other.min_x || other.max_x < min_x || max_y < other.min_y || other.max_y < min_y);
    }
};

struct LineSegment {
    Point2D pt1, pt2;

    bool operator==(const LineSegment &other) const { return pt1 == other.pt1 && pt2 == other.pt2; }

    [[nodiscard]] double Length() const { return pt2.DistanceTo(pt1); }

    [[nodiscard]] Point2D Direction() const { return pt2 - pt1; }

    [[nodiscard]] BoundingBox BoundBox() const { return BoundingBox{pt1, pt2}; }

    [[nodiscard]] double Width() const { return BoundBox().Width(); }

    [[nodiscard]] double Height() const { return BoundBox().Height(); }

    [[nodiscard]] double UnusualHeight() const { return BoundBox().max_y; }

    [[nodiscard]] Point2D Center() const { return BoundBox().Center(); }

    [[nodiscard]] std::array<Point2D, 2> Vertices() const { return {pt1, pt2}; }

    [[nodiscard]] std::array<LineSegment, 2> Edges() const { return {LineSegment{pt1, pt2}, LineSegment{pt2, pt1}}; }

    [[nodiscard]] bool Contains(const Point2D &pt) const {
        return std::abs(Direction().CrossProduct(pt - pt1)) < 1e-10 && BoundBox().Contains(pt);
    }
};

struct Triangle {
    std::array<Point2D, 3> pts;

    constexpr Triangle(std::initializer_list<Point2D> list) {
        if (list.size() != 3) {
            throw std::logic_error("Gotta have 3 points in triangle");
        }
        std::ranges::copy(list, pts.begin());
    }

    bool operator==(const Triangle &other) const { return pts == other.pts; }

    [[nodiscard]] double Area() const {
        const Point2D a = pts[1] - pts[0];
        const Point2D b = pts[2] - pts[0];
        return std::abs(a.CrossProduct(b)) / 2;
    }

    [[nodiscard]] double UnusualHeight() const { return BoundBox().max_y; }

    [[nodiscard]] Point2D Center() const { return std::ranges::fold_left(pts, Point2D{0, 0}, std::plus<>{}) / 3; }

    [[nodiscard]] BoundingBox BoundBox() const { return BoundingBox{pts[0], pts[1], pts[2]}; }

    [[nodiscard]] std::array<Point2D, 3> Vertices() const { return pts; }

    [[nodiscard]] std::array<LineSegment, 3> Edges() const {
        return {LineSegment{pts[0], pts[1]}, LineSegment{pts[1], pts[2]}, LineSegment{pts[2], pts[0]}};
    }
};

struct Rectangle {
    Point2D pt1, pt2;

    constexpr Rectangle(Point2D pt1, double width, double height) : pt1(pt1), pt2(pt1.x + width, pt1.y + height) {}
    constexpr Rectangle(Point2D pt1, Point2D pt2) : pt1(pt1), pt2(pt2) {}

    bool operator==(const Rectangle &other) const { return pt1 == other.pt1 && pt2 == other.pt2; }

    [[nodiscard]] double Width() const { return std::abs(pt2.x - pt1.x); }
    [[nodiscard]] double Height() const { return std::abs(pt2.y - pt1.y); }
    [[nodiscard]] double Area() const { return Width() * Height(); }
    [[nodiscard]] BoundingBox BoundBox() const { return BoundingBox{pt1, pt2}; }
    [[nodiscard]] Point2D Center() const { return BoundBox().Center(); }
    [[nodiscard]] std::array<Point2D, 4> Vertices() const { return {pt1, Point2D{pt2.x, pt1.y}, pt2, Point2D{pt1.x, pt2.y}}; }

    [[nodiscard]] std::array<LineSegment, 4> Edges() const {
        return {
            LineSegment{pt1, Point2D{pt2.x, pt1.y}},
            LineSegment{Point2D{pt2.x, pt1.y}, pt2},
            LineSegment{pt2, Point2D{pt1.x, pt2.y}},
            LineSegment{Point2D{pt1.x, pt2.y}, pt1},
        };
    }

    [[nodiscard]] double UnusualHeight() const { return BoundBox().max_y; }
};

struct RegularPolygon {
    Point2D center_p;
    double radius;
    int sides;

    constexpr RegularPolygon(Point2D center, double radius, int sides)
        : center_p(center), radius(radius), sides(sides) {}

    bool operator==(const RegularPolygon &other) const {
        return center_p == other.center_p && radius == other.radius && sides == other.sides;
    }

    [[nodiscard]] std::vector<Point2D> Vertices() const {
        std::vector<Point2D> points;
        points.reserve(sides);

        for (int i = 0; i < sides; ++i) {
            const double angle = 2 * std::numbers::pi * i / sides;
            points.emplace_back(center_p.x + radius * std::cos(angle), center_p.y + radius * std::sin(angle));
        }
        return points;
    }

    [[nodiscard]] BoundingBox BoundBox() const { return BoundingBox{Vertices()}; }

    [[nodiscard]] Point2D Center() const { return center_p; }

    [[nodiscard]] std::vector<LineSegment> Edges() const {
        auto vertices = Vertices();
        std::vector<LineSegment> edges;
        edges.reserve(sides);
        for (int i = 0; i < sides; ++i) {
            edges.emplace_back(vertices[i], vertices[(i + 1) % sides]);
        }
        return edges;
    }

    [[nodiscard]] double UnusualHeight() const { return BoundBox().max_y; }
};

struct Circle {
    Point2D center;
    double radius;

    constexpr Circle(Point2D center, double radius) : center(center), radius(radius) {
        if (radius < -1e-10) {
            throw std::invalid_argument("Radius must be positive");
        }
    }

    bool operator==(const Circle &other) const { return center == other.center && radius == other.radius; }

    [[nodiscard]] BoundingBox BoundBox() const {
        return {center.x - radius, center.y - radius, center.x + radius, center.y + radius};
    }

    [[nodiscard]] double UnusualHeight() const { return center.y + radius; }
    [[nodiscard]] Point2D Center() const { return center; }

    [[nodiscard]] std::vector<Point2D> Vertices(int N = 100) const {
        std::vector<Point2D> points;
        points.reserve(N);

        for (int i = 0; i < N; ++i) {
            const double angle = 2 * std::numbers::pi * i / N;
            points.emplace_back(center.x + radius * std::cos(angle), center.y + radius * std::sin(angle));
        }
        return points;
    }

    [[nodiscard]] std::vector<LineSegment> Edges(size_t N = 100) const {
        auto vertices = Vertices(N);
        std::vector<LineSegment> edges;
        edges.reserve(N);
        for (size_t i = 0; i < N; ++i) {
            edges.emplace_back(vertices[i], vertices[(i + 1) % N]);
        }
        return edges;
    }
};

class Polygon {
public:
    Polygon() = default;

    explicit Polygon(std::vector<Point2D> vertices) : vertices_(std::move(vertices)) {
        if (vertices_.size() < 3) {
            throw std::logic_error("Gotta have at least 3 points in polygon");
        }
    }

    bool operator==(const Polygon &other) const { return vertices_ == other.vertices_; }

    [[nodiscard]] BoundingBox BoundBox() const { return BoundingBox{vertices_}; }

    [[nodiscard]] Point2D Center() const {
        return std::ranges::fold_left(vertices_, Point2D{0, 0}, std::plus<>{}) / vertices_.size();
    }

    [[nodiscard]] double UnusualHeight() const { return BoundBox().max_y; }

    [[nodiscard]] double Area() const {
        double area = 0;
        for (size_t i = 0; i < vertices_.size(); ++i) {
            size_t next = i + 1 == vertices_.size() ? 0 : i + 1;
            area += (vertices_[i] - vertices_.front()).CrossProduct(vertices_[next] - vertices_.front()) / 2;
        }
        return std::abs(area) / 2;
    }

    [[nodiscard]] std::vector<Point2D> Vertices() const { return vertices_; }

    [[nodiscard]] std::vector<LineSegment> Edges() const {
        std::vector<LineSegment> edges;
        edges.reserve(vertices_.size());
        for (size_t i = 0; i < vertices_.size(); ++i) {
            edges.emplace_back(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
        }
        return edges;
    }

private:
    std::vector<Point2D> vertices_;
};

using Shape = std::variant<LineSegment, Triangle, Rectangle, RegularPolygon, Circle, Polygon>;

inline bool Distinct(const Shape &shape1, const Shape &shape2) {
    return std::visit(
        [](const auto &shape1, const auto &shape2) {
            if constexpr (std::is_same_v<decltype(shape1), decltype(shape2)>) {
                return !(shape1 == shape2);
            } else {
                return false;
            }
        },
        shape1, shape2);
}

enum class GeometryError { Unsupported, NoIntersection, InvalidInput, DegenerateCase, InsufficientPoints };

template <typename T>
using GeometryResult = std::expected<T, GeometryError>;

}  // namespace geometry

template <>
struct std::formatter<geometry::Point2D> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    constexpr auto format(geometry::Point2D p, FormatContext &ctx) const {
        return format_to(ctx.out(), "({:.2f}, {:.2f})", p.x, p.y);
    }
};

template <>
struct std::formatter<std::vector<geometry::Point2D>> {
    bool use_new_line = false;

    constexpr auto parse(std::format_parse_context &ctx) {
        constexpr std::string_view kTag = "new_line}";

        auto it = ctx.begin();
        if (it != ctx.end() && *it == ':') {
            ++it;
            if (std::ranges::equal(it, ctx.end(), kTag.begin(), kTag.end())) {
                use_new_line = true;
                it = ctx.end();
            } else {
                throw std::format_error("Only {:new_line} is supported");
            }
        }
        return it;
    }

    template <typename FormatContext>
    auto format(const std::vector<geometry::Point2D> &v, FormatContext &ctx) const {
        auto out = ctx.out();
        if (use_new_line) {
            out = std::format_to(out, "[\n");
            for (const auto &p : v) {
                out = std::format_to(out, "\t{},\n", p);
            }
            out = std::format_to(out, "]");
        } else {
            out = std::format_to(out, "[");
            bool first = true;
            for (const auto &p : v) {
                if (!first) {
                    out = std::format_to(out, ", ");
                }
                first = false;
                out = std::format_to(out, "{}", p);
            }
            out = std::format_to(out, "]");
        }
        return out;
    }
};

template <>
struct std::formatter<geometry::LineSegment> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::LineSegment &l, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Line({}, {})", l.pt1, l.pt2);
    }
};

template <>
struct std::formatter<geometry::Circle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Circle &c, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Circle(center={}, r={:.2f})", c.center, c.radius);
    }
};

template <>
struct std::formatter<geometry::Rectangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Rectangle &r, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Rectangle(pt1={}, pt2={})", r.pt1, r.pt2);
    }
};

template <>
struct std::formatter<geometry::RegularPolygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::RegularPolygon &p, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "RegularPolygon(center={}, r={:.2f}, sides={})", p.center_p, p.radius,
                              p.sides);
    }
};
template <>
struct std::formatter<geometry::Triangle> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Triangle &t, FormatContext &ctx) const {
        return std::format_to(ctx.out(), "Triangle({}, {}, {})", t.pts[0], t.pts[1], t.pts[2]);
    }
};
template <>
struct std::formatter<geometry::Polygon> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    template <typename FormatContext>
    auto format(const geometry::Polygon &poly, FormatContext &ctx) const {
        auto out = ctx.out();
        out = std::format_to(out, "Polygon[{} points]: [", poly.Vertices().size());

        for (const auto &p : poly.Vertices()) {
            out = std::format_to(out, "{} ", p);
        }

        return std::format_to(out, "]");
    }
};
