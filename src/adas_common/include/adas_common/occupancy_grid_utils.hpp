#ifndef OCCUPANCY_GRID_UTILS_HPP_
#define OCCUPANCY_GRID_UTILS_HPP_

//#include <tf/transform_datatypes.h>
#include "tf2/utils.h"

#include <array>
#include <vector>
#include <tuple>
#include <algorithm>

namespace occupancy_grid_utils {

/**
 * @brief Rectangular data type (templated). Wound CCW -> right hand co-ordinate system
 * 
 * @tparam T 
 */
template <typename T> using RectDataType = std::array<T, 4>;
// CCW winded coordinates of corners
using Rect = RectDataType<tf2::Vector3>;
// Ax + By + C = 0
using LineEquation = std::tuple<double, double, double>;

/**
 * @brief calculate co-efficients of cartesian equation of a line 
 * 
 * @param p1 point1 
 * @param p2 point2
 * @return LineEquation  std::tuple<double, double, double> -> tuple storing co-efficients of cartesian equation
 */
LineEquation lineFromPoints(const tf2::Vector3 &p1, const tf2::Vector3 &p2);

/**
 * @brief determines whether a point is within the bounding rectangle of the occupancy grid
 * 
 * @param point point of concern
 * @param rect 4 points containing rectangle (data type defines above)
 * @return true point is inside
 * @return false point is not inside
 */
bool pointInRect(const tf2::Vector3 &point, const Rect & rect);

/**
 * @brief struct containing grid metadata
 * 
 */
struct GridInfo
{
    /**
     * @brief Construct a new Grid Info object
     * 
     * @param mPerCell grid resolution meters per cell
     * @param width 
     * @param height 
     */
    GridInfo(const double mPerCell, const uint32_t width, const uint32_t height);
    
    /**
     * @brief functions to transform between cell ID and RowCol = std::pair<int,int>
     * 
     */
    using RowCol = std::pair<int, int>;
    RowCol toRowCol(const tf2::Vector3 &point) const;
    int toCellIdx(const RowCol &rowCol) const;
    tf2::Vector3 toPoint(const RowCol &rowCol) const;

    const double mPerCell;
    const uint32_t width;
    const uint32_t height;
};

/**
 * @brief flood fill occupancy grid based on seed value (see: https://en.wikipedia.org/wiki/Flood_fill -> version - moving recursion into data structure)
 * 
 * @param rect 
 * @param gridInfo 
 * @param world 
 * @param fillValue 
 */

void floodFill(
    const Rect &rect,
    const GridInfo &gridInfo,
    std::vector<int8_t> &world,
    const int8_t fillValue = 100);

/**
 * @brief not sure
 * 
 * @param start 
 * @param minDistance 
 * @param maxDistance 
 * @param gridInfo 
 * @param world 
 * @param detectionThreshold 
 * @param unknownIsBlocked 
 * @return double 
 */
double castDistance(const tf2::Transform &start,
                    const double minDistance,
                    const double maxDistance,
                    const GridInfo &gridInfo,
                    const std::vector<int8_t> &world,
                    const int8_t detectionThreshold,
                    const bool unknownIsBlocked);

Rect rectFromCarInfo(
    const std::pair<double, double> &offset,
    const std::pair<double, double> &size);

}

#endif
/**
 * changed tf::Pose to tf2::Transform
 * 
 * tf2::Transform is analogous to geometry_msgs/msg/Pose. Encodes the position and orientation information
 * tf2::Transform stores the rotation matrix/quaternion & position and has member functions to manipulate the data
 * 
 * 
 * TODO: need to figure out typedef for tf::Point
 *       change to tf2::Vector3
 */
