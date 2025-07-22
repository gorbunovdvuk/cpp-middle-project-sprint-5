//
// Created by Dmitry Gorbunov on 22.07.2025.
//

#include "geometry.hpp"
#include <gtest/gtest.h>

using namespace geometry;

TEST(GeometryTest, Point2DTest) {
    Point2D p1{3, 0}, p2{0, 4};
    EXPECT_EQ(p1 + p2, Point2D(3, 4));
    EXPECT_EQ(p1 - p2, Point2D(3, -4));

    EXPECT_EQ(p1 * 2, Point2D(6, 0));
    EXPECT_EQ(p2 / 2, Point2D(0, 2));

    EXPECT_DOUBLE_EQ(p1.DotProduct(p2), 0.0);
    EXPECT_DOUBLE_EQ(p1.CrossProduct(p2), 12);

    EXPECT_EQ(p1.Length2(), 9);
    EXPECT_EQ(p1.Length(), 3);
    EXPECT_TRUE(p1.Normalize().Equals(Point2D(1, 0)));

    EXPECT_DOUBLE_EQ(p1.DistanceTo(p2), 5);
}

TEST(GeometryTest, BoundingBoxTest) {
    BoundingBox box{1.0, 2.0, 4.0, 6.0};

    EXPECT_DOUBLE_EQ(box.Width(), 3.0);
    EXPECT_DOUBLE_EQ(box.Height(), 4.0);
    EXPECT_EQ(box.Center(), Point2D(2.5, 4.0));

    EXPECT_TRUE(box.Contains(Point2D{2.0, 3.0}));
    EXPECT_FALSE(box.Contains(Point2D{0.0, 0.0}));
}

TEST(GeometryTest, LineSegmentTest) {
    LineSegment seg{Point2D{0.0, 0.0}, Point2D{3.0, 4.0}};

    EXPECT_DOUBLE_EQ(seg.Length(), 5.0);
    EXPECT_EQ(seg.Direction(), Point2D(3.0, 4.0));

    auto box = seg.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, 0.0);
    EXPECT_DOUBLE_EQ(box.min_y, 0.0);
    EXPECT_DOUBLE_EQ(box.max_x, 3.0);
    EXPECT_DOUBLE_EQ(box.max_y, 4.0);

    EXPECT_TRUE(seg.Contains(Point2D{1.5, 2.0}));
    EXPECT_FALSE(seg.Contains(Point2D{2.0, 3.0}));
}

TEST(GeometryTest, TriangleTest) {
    Triangle tri{Point2D{0.0, 0.0}, Point2D{4.0, 0.0}, Point2D{0.0, 3.0}};

    EXPECT_DOUBLE_EQ(tri.Area(), 6.0);
    EXPECT_EQ(tri.Center(), Point2D(4.0 / 3.0, 1.0));

    auto box = tri.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, 0.0);
    EXPECT_DOUBLE_EQ(box.min_y, 0.0);
    EXPECT_DOUBLE_EQ(box.max_x, 4.0);
    EXPECT_DOUBLE_EQ(box.max_y, 3.0);

    EXPECT_THROW((Triangle{Point2D(0.0, 0.0), Point2D(1.0, 1.0)}), std::logic_error);
}

TEST(GeometryTest, RectangleTest) {
    Rectangle rect{Point2D{1.0, 2.0}, 3.0, 4.0};

    EXPECT_DOUBLE_EQ(rect.Width(), 3.0);
    EXPECT_DOUBLE_EQ(rect.Height(), 4.0);
    EXPECT_DOUBLE_EQ(rect.Area(), 12.0);

    auto box = rect.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, 1.0);
    EXPECT_DOUBLE_EQ(box.max_x, 4.0);
}

TEST(GeometryTest, RegularPolygonTest) {
    constexpr int sides = 6;
    RegularPolygon hex{Point2D{0.0, 0.0}, 1.0, sides};

    EXPECT_EQ(hex.Vertices().size(), sides);
    EXPECT_EQ(hex.Edges().size(), sides);

    for (const auto &v : hex.Vertices()) {
        EXPECT_TRUE(std::abs(v.Length() - 1.0) < 1e-10);
    }

    auto box = hex.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, -1.0);
    EXPECT_DOUBLE_EQ(box.max_x, 1.0);
}

TEST(GeometryTest, PolygonTest) {
    Polygon poly{{
        {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}
    }};

    EXPECT_DOUBLE_EQ(poly.Area(), 4.0);
    EXPECT_EQ(poly.Center(), Point2D(1.0, 1.0));

    auto box = poly.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, 0.0);
    EXPECT_DOUBLE_EQ(box.max_x, 2.0);
}

TEST(GeometryTest, CircleTest) {
    Circle c{Point2D{1.0, 1.0}, 2.0};

    auto box = c.BoundBox();
    EXPECT_DOUBLE_EQ(box.min_x, -1.0);
    EXPECT_DOUBLE_EQ(box.min_y, -1.0);
    EXPECT_DOUBLE_EQ(box.max_x, 3.0);
    EXPECT_DOUBLE_EQ(box.max_y, 3.0);

    auto verts = c.Vertices(50);
    EXPECT_EQ(verts.size(), 50);
    for (const auto &v : verts) {
        EXPECT_TRUE(std::abs((v - Point2D{1.0, 1.0}).Length() - 2.0) < 1e-10);
    }
}