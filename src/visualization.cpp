#include "visualization.hpp"
#include "geometry.hpp"

#include <matplot/matplot.h>
#include <print>
#include <range/v3/all.hpp>

namespace geometry::visualization {

auto xs = std::views::transform([](const auto& line) { return std::vector{line.pt1.x, line.pt2.x}; }) | std::views::join | std::ranges::to<std::vector>();
auto ys = std::views::transform([](const auto& line) { return std::vector{line.pt1.y, line.pt2.y}; }) | std::views::join | std::ranges::to<std::vector>();

template <class... Ts>
struct Multilambda : Ts... {
    using Ts::operator()...;
};

void Draw(std::span<geometry::Shape> shapes) {
    using namespace geometry;
    using namespace matplot;

    // Disable gnuplot warnings
    auto f = figure(false);
    f->backend()->run_command("unset warnings");
    f->ioff();
    f->size(900, 900);

    hold(on);     // Multiple plots mode
    axis(equal);  // Squre view
    grid(on);     // Enable grid by default

    for (const auto &[index, shape] : ranges::views::enumerate(shapes)) {
        std::visit(Multilambda{[&](const LineSegment &line) {
                                   const auto lines = line.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("yellow");
                               },
                               [&](const Triangle &tri) {
                                   const auto lines = tri.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("blue");
                               },
                               [&](const Rectangle &rect) {
                                   const auto lines = rect.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("green");
                               },
                               [&](const RegularPolygon &poly) {
                                   const auto lines = poly.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("magenta");
                               },
                               [&](const Circle &circle) {
                                   const auto lines = circle.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("red");
                               },
                               [&](const Polygon &poly) {
                                   const auto lines = poly.Edges();
                                   plot(lines | xs, lines | ys)->line_width(2).color("cyan");
                               }},
                   shape);

        // Add shape number
        const auto center = std::visit([](auto &&s) { return s.Center(); }, shape);
        auto t = text(center.x, center.y, std::to_string(index));
        t->font_size(14);
        t->color("black");
    }

    // Display plot
    f->show();
}

void Draw(std::span<const triangulation::DelaunayTriangle> triangles) {
    std::vector<Shape> shapes{std::from_range, triangles | std::views::transform([](const triangulation::DelaunayTriangle& triangle) {
        return Triangle{triangle.a, triangle.b, triangle.c};
    })};
    Draw(shapes);
}

}  // namespace geometry::visualization
