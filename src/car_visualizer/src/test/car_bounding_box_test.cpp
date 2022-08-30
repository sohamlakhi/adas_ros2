#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>

#include "car_visualizer/car_bounding_box.hpp"

using namespace car_bounding_box;

static const Point2D testBoundOffset = std::make_pair(20, 5);
static const Point2D testBoundSize = std::make_pair(150, 75);

TEST(CarBoundingBox, Initialize)
{
    BoundingBox bb(testBoundOffset, testBoundSize);
}

TEST(CarBoundingBox, Except)
{
    ASSERT_THROW(BoundingBox(testBoundOffset, std::make_pair(-150.0, 75.0)), std::invalid_argument);
    ASSERT_THROW(BoundingBox(testBoundOffset, std::make_pair(150.0, -75.0)), std::invalid_argument);
}

TEST(CarBoundingBox, BoxSize)
{
    BoundingBox bb(testBoundOffset, testBoundSize);

    std::vector<geometry_msgs::Point> box = bb.get();

    EXPECT_EQ(box.size(), 5);
}

TEST(CarBoundingBox, BoxValues)
{
    BoundingBox bb(testBoundOffset, testBoundSize);

    std::vector<geometry_msgs::Point> box = bb.get();

    for (const geometry_msgs::Point &p: box)
    {
        EXPECT_DOUBLE_EQ(p.z, 0.0);
    }

    EXPECT_DOUBLE_EQ(box[0].x, testBoundOffset.first - testBoundSize.first / 2);
    EXPECT_DOUBLE_EQ(box[0].y, testBoundOffset.second - testBoundSize.second / 2);

    EXPECT_DOUBLE_EQ(box[1].x, testBoundOffset.first + testBoundSize.first / 2);
    EXPECT_DOUBLE_EQ(box[1].y, testBoundOffset.second - testBoundSize.second / 2);

    EXPECT_DOUBLE_EQ(box[2].x, testBoundOffset.first + testBoundSize.first / 2);
    EXPECT_DOUBLE_EQ(box[2].y, testBoundOffset.second + testBoundSize.second / 2);

    EXPECT_DOUBLE_EQ(box[3].x, testBoundOffset.first - testBoundSize.first / 2);
    EXPECT_DOUBLE_EQ(box[3].y, testBoundOffset.second + testBoundSize.second / 2);

    EXPECT_DOUBLE_EQ(box[4].x, testBoundOffset.first - testBoundSize.first / 2);
    EXPECT_DOUBLE_EQ(box[4].y, testBoundOffset.second - testBoundSize.second / 2);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
