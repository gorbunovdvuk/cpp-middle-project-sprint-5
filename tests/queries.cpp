//
// Created by Dmitry Gorbunov on 12.07.2025.
//

#include "queries.hpp"
#include "geometry.hpp"

#include <gtest/gtest.h>

using namespace geometry;

TEST(QueriesTest, PointToSegmentDistanceTest) {
    Point2D point{2, 0};
    LineSegment segment{{0, 0}, {5, 5}};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(segment, point), std::sqrt(2));
}

TEST(QueriesTest, PointToSegmentVertexDistanceTest) {
    Point2D point{2, -3};
    LineSegment segment{{0, 0}, {5, 5}};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(segment, point), std::sqrt(13));
}

TEST(QueriesTest, PointToSegmentPointOnSegmentDistanceTest) {
    Point2D point{2, 2};
    LineSegment segment{{0, 0}, {5, 5}};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(segment, point), 0);
}

TEST(QueriesTest, PointToTriangleDistanceTest) {
    Point2D point{0, 0};
    Triangle triangle{{-1, 0}, {0, 1}, {2, -2}};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(triangle, point), 2. / std::sqrt(13));
}

TEST(QueriesTest, PointToCircleInsideDistanceTest) {
    Point2D point{0, 0};
    Circle circle{{0, 0}, 5};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle, point), 5);
}

TEST(QueriesTest, PointToCircleOutsideDistanceTest) {
    Point2D point{0, 0};
    Circle circle{{10, 10}, 5};
    EXPECT_DOUBLE_EQ(queries::DistanceToPoint(circle, point), std::sqrt(200) - 5);
}

TEST(QueriesTest, CircleToCircleDistanceTest) {
    Circle circle1{{10, 10}, 5};
    Circle circle2{{0, 0}, 5};
    EXPECT_DOUBLE_EQ(queries::DistanceBetweenShapes(circle1, circle2).value(), std::sqrt(200) - 10);
}

TEST(QueriesTest, CircleToCircleIntersectingDistanceTest) {
    Circle circle1{{10, 10}, 5};
    Circle circle2{{5, 5}, 5};
    EXPECT_DOUBLE_EQ(queries::DistanceBetweenShapes(circle1, circle2).value(), 0);
}

TEST(QueriesTest, SegmentToSegmentDistanceTest) {
    LineSegment segment1{{0, 0}, {5, 5}};
    LineSegment segment2{{6, 6}, {10, 10}};
    EXPECT_DOUBLE_EQ(queries::DistanceBetweenShapes(segment1, segment2).value(), std::sqrt(2));
}

TEST(QueriesTest, SegmentToSegmentIntersectingDistanceTest) {
    LineSegment segment1{{0, 0}, {5, 5}};
    LineSegment segment2{{5, 0}, {0, 5}};
    EXPECT_DOUBLE_EQ(queries::DistanceBetweenShapes(segment1, segment2).value(), 0);
}