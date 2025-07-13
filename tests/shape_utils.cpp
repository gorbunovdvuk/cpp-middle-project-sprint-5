//
// Created by Dmitry Gorbunov on 13.07.2025.
//

#include "shape_utils.hpp"

#include <gtest/gtest.h>

using namespace geometry;

TEST(ShapeUtilsTest, TestFindAllCollisions) {
    std::vector<Shape> shapes = {
        Circle{{0, 0}, 3},
        Circle{{1, 1}, 3},
        Rectangle{{5, 5}, 2, 2},
        Rectangle{{5, 5}, 3, 3},
    };
    auto collisions = utils::FindAllCollisions(shapes);
    EXPECT_EQ(collisions.size(), 4);
    EXPECT_TRUE(!Distinct(std::get<0>(collisions[0]), shapes[0]));
    EXPECT_TRUE(!Distinct(std::get<1>(collisions[0]), shapes[1]));
    EXPECT_TRUE(!Distinct(std::get<0>(collisions[1]), shapes[1]));
    EXPECT_TRUE(!Distinct(std::get<1>(collisions[1]), shapes[0]));

    EXPECT_TRUE(!Distinct(std::get<0>(collisions[2]), shapes[2]));
    EXPECT_TRUE(!Distinct(std::get<1>(collisions[2]), shapes[3]));
    EXPECT_TRUE(!Distinct(std::get<0>(collisions[3]), shapes[3]));
    EXPECT_TRUE(!Distinct(std::get<1>(collisions[3]), shapes[2]));
}