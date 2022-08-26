#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>

#include "adas_common/occupancy_grid_utils.hpp"

using namespace occupancy_grid_utils;

const LineEquation testLine(5.4, -2.5, 3.7);
const tf::Point testPoint1(-3.5, 1.1, 0.0);
const tf::Point testPoint2(1.0, -5.7, 0.0);
const double testRange = 1e-8;

const double testMPerCell = 0.5;
const double testWidth = 10 / 0.5;
const double testHeight = 20 / 0.5;

TEST(LineEquation, LineFromPoints)
{
    LineEquation line = lineFromPoints(testPoint1, testPoint2);

    double A = std::get<0>(line);
    double B = std::get<1>(line);
    double C = std::get<2>(line);

    double point1Value = A * testPoint1.getX() + B * testPoint1.getY() + C;
    EXPECT_NEAR(point1Value, 0.0, testRange);
    double point2Value = A * testPoint2.getX() + B * testPoint2.getY() + C;
    EXPECT_NEAR(point2Value, 0.0, testRange);

    tf::Point midpoint(
        (testPoint1.getX() + testPoint2.getX()) / 2,
        (testPoint1.getY() + testPoint2.getY()) / 2,
        0.0);
    double midpointValue = A * midpoint.getX() + B * midpoint.getY() + C;
    EXPECT_NEAR(midpointValue, 0.0, testRange);
}

TEST(PointInRect, Basic)
{
    const Rect testRect{
        tf::Point(-2.5, 2.5, 0.0),
        tf::Point(-2.5, -2.5, 0.0),
        tf::Point(2.5, -2.5, 0.0),
        tf::Point(2.5, 2.5, 0.0)};

    EXPECT_EQ(pointInRect(tf::Point(3.5, 1.0, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(3.5, 2.6, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(-1.0, 2.6, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(-2.7, 3.5, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(-2.7, -0.3, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(-2.7, -4.0, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(-2.7, -4.0, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(1.2, -4.0, 0.0), testRect), false);
    EXPECT_EQ(pointInRect(tf::Point(4.5, -4.0, 0.0), testRect), false);

    EXPECT_EQ(pointInRect(tf::Point(2.0, 0.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(-2.0, 0.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(0.0, 2.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(0.0, -2.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(2.0, 2.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(-2.0, -2.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(2.0, -2.0, 0.0), testRect), true);
    EXPECT_EQ(pointInRect(tf::Point(-2.0, 2.0, 0.0), testRect), true);
}

TEST(GridInfo, ToRowCol)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);

    GridInfo::RowCol rowCol = gridInfo.toRowCol(tf::Point(1.8, 15.3, 0.0));
    EXPECT_EQ(rowCol.first, 30);
    EXPECT_EQ(rowCol.second, 3);

    rowCol = gridInfo.toRowCol(tf::Point(9.5, 19.5, 0.0));
    EXPECT_EQ(rowCol.first, 39);
    EXPECT_EQ(rowCol.second, 19);

    rowCol = gridInfo.toRowCol(tf::Point(10, 20, 0.0));
    EXPECT_EQ(rowCol.first, 40);
    EXPECT_EQ(rowCol.second, 20);

    rowCol = gridInfo.toRowCol(tf::Point(0.0, 0.0, 0.0));
    EXPECT_EQ(rowCol.first, 0);
    EXPECT_EQ(rowCol.second, 0);
}

TEST(GridInfo, ToCellIdx)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);

    GridInfo::RowCol rowCol = std::make_pair(0, 0);
    EXPECT_EQ(gridInfo.toCellIdx(rowCol), 0);

    rowCol = std::make_pair(39, 19);
    EXPECT_EQ(gridInfo.toCellIdx(rowCol), testWidth * testHeight - 1);

    rowCol = std::make_pair(20, 10);
    EXPECT_EQ(gridInfo.toCellIdx(rowCol), 20 * testWidth + 10);
}

TEST(GridInfo, ToPoint)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);

    GridInfo::RowCol rowCol = std::make_pair(0, 0);
    tf::Point point = gridInfo.toPoint(rowCol);
    EXPECT_DOUBLE_EQ(point.getX(), 0.25);
    EXPECT_DOUBLE_EQ(point.getY(), 0.25);

    rowCol = std::make_pair(0, 19);
    point = gridInfo.toPoint(rowCol);
    EXPECT_DOUBLE_EQ(point.getX(), 9.75);
    EXPECT_DOUBLE_EQ(point.getY(), 0.25);

    rowCol = std::make_pair(39, 0);
    point = gridInfo.toPoint(rowCol);
    EXPECT_DOUBLE_EQ(point.getX(), 0.25);
    EXPECT_DOUBLE_EQ(point.getY(), 19.75);

    rowCol = std::make_pair(39, 19);
    point = gridInfo.toPoint(rowCol);
    EXPECT_DOUBLE_EQ(point.getX(), 9.75);
    EXPECT_DOUBLE_EQ(point.getY(), 19.75);

    rowCol = std::make_pair(20, 10);
    point = gridInfo.toPoint(rowCol);
    EXPECT_DOUBLE_EQ(point.getX(), 5.25);
    EXPECT_DOUBLE_EQ(point.getY(), 10.25);
}

TEST(FloodFill, Basic)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    Rect rect{
        tf::Point(3.875, 0.725, 0.0),
        tf::Point(3.875, 2.875, 0.0),
        tf::Point(0.725, 2.875, 0.0),
        tf::Point(0.725, 0.725, 0.0)};

    std::vector<int8_t> world(gridInfo.width * gridInfo.height, 0);

    floodFill(rect, gridInfo, world);

    int rowMin = std::floor(0.725 / 0.5);
    int rowMax = std::floor(2.875 / 0.5);
    int colMin = std::floor(0.725 / 0.5);
    int colMax = std::floor(3.875 / 0.5);

    for (int col = 0; col < testWidth; col++)
    {
        for (int row = 0; row < testHeight; row++)
        {
            if (col < colMin || col > colMax || row < rowMin || row > rowMax)
            {
                EXPECT_EQ(world[row * gridInfo.width + col], 0);
            }
            else
            {
                EXPECT_EQ(world[row * gridInfo.width + col], 100);
            }
        }
    }
}

TEST(FloodFill, Erase)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    Rect rect{
        tf::Point(3.875, 0.725, 0.0),
        tf::Point(3.875, 2.875, 0.0),
        tf::Point(0.725, 2.875, 0.0),
        tf::Point(0.725, 0.725, 0.0)};

    std::vector<int8_t> world(gridInfo.width * gridInfo.height, 0);

    floodFill(rect, gridInfo, world);
    floodFill(rect, gridInfo, world, 0);

    for (int col = 0; col < testWidth; col++)
    {
        for (int row = 0; row < testHeight; row++)
        {
            EXPECT_EQ(world[row * gridInfo.width + col], 0);
        }
    }
}

TEST(FloodFill, OutOfBound)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    Rect rect{
        tf::Point(testWidth + 1, testHeight, 0.0),
        tf::Point(testWidth + 1, testHeight + 1, 0.0),
        tf::Point(testWidth, testHeight + 1, 0.0),
        tf::Point(testWidth, testHeight, 0.0)};

    std::vector<int8_t> world(gridInfo.width * gridInfo.height, 0);

    floodFill(rect, gridInfo, world);

    for (int col = 0; col < testWidth; col++)
    {
        for (int row = 0; row < testHeight; row++)
        {
            EXPECT_EQ(world[row * gridInfo.width + col], 0);
        }
    }
}


TEST(FloodFill, Error)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    Rect rect{
        tf::Point(0.75, 0.75, 0.0),
        tf::Point(0.75, 2.75, 0.0),
        tf::Point(3.75, 2.75, 0.0),
        tf::Point(3.75, 0.75, 0.0)};

    std::vector<int8_t> world;

    ASSERT_THROW(floodFill(rect, gridInfo, world), std::invalid_argument);
}

const double testMinDistance = 2 * testMPerCell;
const double testMaxDistance = 7 * testMPerCell;
const int8_t testDetectionThreshold = 50;

std::vector<int8_t> getTestWorld(
    const GridInfo &gridInfo,
    const tf::Point &point,
    const int8_t fill)
{
    std::vector<int8_t> world(gridInfo.width * gridInfo.height, 0);

    GridInfo::RowCol rowCol = gridInfo.toRowCol(point);
    EXPECT_TRUE(rowCol.first >= 0 && rowCol.first < static_cast<int>(gridInfo.height));
    EXPECT_TRUE(rowCol.second >= 0 && rowCol.second < static_cast<int>(gridInfo.width));

    world[gridInfo.toCellIdx(rowCol)] = fill;

    return world;
}

TEST(CastDistance, NoObstacle)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world(gridInfo.width * gridInfo.height, 0);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        false);
    EXPECT_DOUBLE_EQ(result, testMaxDistance * 2);
}

TEST(CastDistance, ObstacleOutsideMax)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world = getTestWorld(gridInfo, tf::Point(testMaxDistance + testMPerCell, 0, 0), testDetectionThreshold + 1);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        false);
    EXPECT_DOUBLE_EQ(result, testMaxDistance * 2);
}

TEST(CastDistance, ObstacleInsideMin)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world = getTestWorld(gridInfo, tf::Point(0.75, 0, 0), testDetectionThreshold + 1);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        false);
    EXPECT_DOUBLE_EQ(result, testMaxDistance * 2);
}

TEST(CastDistance, ObstacleUnkownNotDetected)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world = getTestWorld(gridInfo, tf::Point(testMaxDistance - 2 * testMPerCell, 0, 0), -1);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        false);
    EXPECT_DOUBLE_EQ(result, testMaxDistance * 2);
}

TEST(CastDistance, ObstacleDetected)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world = getTestWorld(gridInfo, tf::Point(testMaxDistance - 2 * testMPerCell, 0, 0), testDetectionThreshold + 1);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        false);
    EXPECT_DOUBLE_EQ(result, 2.25);
}

TEST(CastDistance, UnknownDetected)
{
    GridInfo gridInfo(testMPerCell, testWidth, testHeight);
    tf::Pose pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(testMPerCell / 2, testMPerCell / 2, 0));
    std::vector<int8_t> world = getTestWorld(gridInfo, tf::Point(testMaxDistance - 2 * testMPerCell, 0, 0), -1);

    double result = castDistance(
        pose,
        testMinDistance,
        testMaxDistance,
        gridInfo,
        world,
        testDetectionThreshold,
        true);
    EXPECT_DOUBLE_EQ(result, 2.25);
}

int main(int argc, char *argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
