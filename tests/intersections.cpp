//
// Created by Dmitry Gorbunov on 09.07.2025.
//

#include "intersections.hpp"

#include "geometry.hpp"

#include <gtest/gtest.h>

using namespace geometry;

TEST(IntersectionsTest, TestSegmentSegmentIntersecting) {
    LineSegment line1{{1, 1}, {5, 3}};
    LineSegment line2{{2, 4}, {4, 0}};
    auto result = intersections::GetIntersectionPoints(line1, line2).value();
    EXPECT_TRUE(result.front().Equals(geometry::Point2D{3, 2}));
}

TEST(IntersectionsTest, TestSegmentSegmentNonIntersecting) {
    LineSegment line1{{1, 1}, {3, 5}};
    LineSegment line2{{2, 2}, {5, 3}};
    auto result = intersections::GetIntersectionPoints(line1, line2);
    EXPECT_TRUE(result == std::unexpected{GeometryError::NoIntersection});
}

TEST(IntersectionsTest, TestSegmentSegmentSameVertex) {
    LineSegment line1{{1, 1}, {5, 3}};
    LineSegment line2{{1, 1}, {3, 5}};
    auto result = intersections::GetIntersectionPoints(line1, line2).value();
    EXPECT_TRUE(result.front().Equals(geometry::Point2D{1, 1}));
}

TEST(IntersectionsTest, TestSegmentSegmentParallel) {
    LineSegment line1{{1, 1}, {3, 3}};
    LineSegment line2{{4, 4}, {5, 5}};
    auto result = intersections::GetIntersectionPoints(line1, line2);
    EXPECT_TRUE(result == std::unexpected{GeometryError::DegenerateCase});
}

TEST(IntersectionsTest, TestSegmentSegmentNested) {
    LineSegment line1{{1, 1}, {3, 3}};
    LineSegment line2{{0, 0}, {4, 4}};
    auto result = intersections::GetIntersectionPoints(line1, line2);
    EXPECT_TRUE(result == std::unexpected{GeometryError::DegenerateCase});
}

TEST(IntersectionsTest, TestSegmentSegmentCommonSegment) {
    LineSegment line1{{1, 1}, {3, 3}};
    LineSegment line2{{2, 2}, {4, 4}};
    auto result = intersections::GetIntersectionPoints(line1, line2);
    EXPECT_TRUE(result == std::unexpected{GeometryError::DegenerateCase});
}

TEST(IntersectionsTest, TestCircleCircleDoubleIntersecting) {
    Circle circle1{{-3, 1}, 1}, circle2{{-3, 4}, 3};
    auto result = intersections::GetIntersectionPoints(circle1, circle2).value();
    EXPECT_TRUE(result.size() == 2);
    EXPECT_TRUE(result[0].Equals(geometry::Point2D{-2.0139867028167306, 1.1666666666666667}));
    EXPECT_TRUE(result[1].Equals(geometry::Point2D{-3.9860132971832694, 1.1666666666666667}));
}

TEST(IntersectionsTest, TestCircleCircleSingleIntersecting) {
    Circle circle1{{-3, 1}, 1}, circle2{{-3, 4}, 2};
    auto result = intersections::GetIntersectionPoints(circle1, circle2).value();
    EXPECT_TRUE(result.size() == 1);
    EXPECT_TRUE(result.front().Equals(geometry::Point2D{-3, 2}));
}

TEST(IntersectionsTest, TestCircleCircleSingleIntersectingFromInterior) {
    Circle circle1{{-3, 1}, 1}, circle2{{-3, 4}, 4};
    auto result = intersections::GetIntersectionPoints(circle1, circle2).value();
    EXPECT_TRUE(result.size() == 1);
    EXPECT_TRUE(result.front().Equals(geometry::Point2D{-3, 0}));
}

TEST(IntersectionsTest, TestCircleCircleNonIntersecting) {
    Circle circle1{{0, 0}, 1}, circle2{{3, 3}, 1};
    auto result = intersections::GetIntersectionPoints(circle1, circle2);
    EXPECT_TRUE(result == std::unexpected{GeometryError::NoIntersection});
}

TEST(IntersectionsTest, TestCircleSegmentTwoIntersections) {
    Circle circle{{0, 0}, 5};
    LineSegment line{{-10, -10}, {10, 10}};
    auto result = intersections::GetIntersectionPoints(line, circle).value();
    EXPECT_TRUE(result.size() == 2);
    EXPECT_TRUE(result[0].Equals(geometry::Point2D{-std::sqrt(25. / 2), -std::sqrt(25. / 2)}));
    EXPECT_TRUE(result[1].Equals(geometry::Point2D{std::sqrt(25. / 2), std::sqrt(25. / 2)}));
}

TEST(IntersectionsTest, TestCircleSegmentSingleIntersection) {
    Circle circle{{0, 0}, 5};
    LineSegment line{{-10, -10}, {0, 0}};
    auto result = intersections::GetIntersectionPoints(line, circle).value();
    EXPECT_TRUE(result.size() == 1);
    EXPECT_TRUE(result[0].Equals(geometry::Point2D{-std::sqrt(25. / 2), -std::sqrt(25. / 2)}));
}

TEST(IntersectionsTest, TestCircleSegmentSingleIntersection2) {
    Circle circle{{0, 0}, 5};
    LineSegment line{{-5, -10}, {-5, 10}};
    auto result = intersections::GetIntersectionPoints(line, circle).value();
    EXPECT_TRUE(result.size() == 1);
    EXPECT_TRUE(result[0].Equals(geometry::Point2D{-5, 0}));
}
