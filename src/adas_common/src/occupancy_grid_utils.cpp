#include "adas_common/occupancy_grid_utils.hpp"

#include <stack>
#include <set>

namespace occupancy_grid_utils
{

LineEquation lineFromPoints(const tf2::Vector3 &p1, const tf2::Vector3 &p2)
{
    double A = -(p2.getY() - p1.getY());
    double B = p2.getX() - p1.getX();
    double C = -(A * p1.getX() + B * p1.getY());

    return LineEquation(A, B, C);
}

bool pointInRect(const tf2::Vector3 &point, const Rect & rect)
{
    // Calculate line equations Ax + By + C = 0
    using LineEquation = std::tuple<double, double, double>;
    RectDataType<LineEquation> lineEquations;

    lineEquations[0] = lineFromPoints(rect[0], rect[1]);
    lineEquations[1] = lineFromPoints(rect[1], rect[2]);
    lineEquations[2] = lineFromPoints(rect[2], rect[3]);
    lineEquations[3] = lineFromPoints(rect[3], rect[0]);

    // Assume CCW winding - Check if point lies to the left of all lines
    for (const LineEquation &eqn: lineEquations)
    {
        /**
         * @brief solve line equation for point. If it is >= 0, the point lies geometrically to the left of the line (in right hand co-ordinate system)
         * 
         */
        double lineResult = std::get<0>(eqn) * point.getX() + std::get<1>(eqn) * point.getY() + std::get<2>(eqn);

        if (lineResult < 0)
        {
            // Point lies to the right.
            return false;
        }
    }

    return true;
}

GridInfo::GridInfo(const double mPerCell, const uint32_t width, const uint32_t height)
    : mPerCell(mPerCell),
      width(width),
      height(height)
{}

GridInfo::RowCol GridInfo::toRowCol(const tf2::Vector3 &point) const
{
    int colIdx = std::floor(point.getX() / mPerCell);
    int rowIdx = std::floor(point.getY() / mPerCell);

    return std::make_pair(rowIdx, colIdx);
}

int GridInfo::toCellIdx(const RowCol &rowCol) const
{
    return rowCol.first * width + rowCol.second;
}

tf2::Vector3 GridInfo::toPoint(const RowCol &rowCol) const
{
    return tf2::Vector3(
        mPerCell * rowCol.second + mPerCell / 2,
        mPerCell * rowCol.first + mPerCell / 2,
        0.0);
}

namespace
{

struct RowColCompare
{
    bool operator()(const GridInfo::RowCol &l, const GridInfo::RowCol &r)
    {
        if (l.first == r.first)
        { return l.second < r.second; }
        return l.first < r.first;
    }
};

}

void floodFill(const Rect & rect, const GridInfo &gridInfo, std::vector<int8_t> &world, const int8_t fillValue) {
    if (world.size() != gridInfo.width * gridInfo.height)
    { throw std::invalid_argument("World vector does not match grid info"); }

    constexpr std::array<GridInfo::RowCol, 4> neighbourOffsets{
        std::make_pair(-1, 0),
        std::make_pair(1, 0),
        std::make_pair(0, -1),
        std::make_pair(0, 1)};

    std::stack<GridInfo::RowCol> toVisit;
    tf2::Vector3 center(
        (rect[0].getX() + rect[2].getX()) / 2,
        (rect[0].getY() + rect[2].getY()) / 2,
        0);
    auto centerPoint = gridInfo.toRowCol(center);
    // If the center is outside the rectangle, do not flood fill.
    if (centerPoint.first < 0 || centerPoint.first >= static_cast<int>(gridInfo.height))
    { return; }
    if (centerPoint.second < 0 || centerPoint.second >= static_cast<int>(gridInfo.width))
    { return; }
    toVisit.push(centerPoint);
    std::set<GridInfo::RowCol, RowColCompare> visited;

    while (toVisit.size() != 0)
    {
        GridInfo::RowCol curr = toVisit.top();
        toVisit.pop();
        visited.insert(curr);

        world[gridInfo.toCellIdx(curr)] = fillValue;

        for (const auto &neighbourOffset: neighbourOffsets)
        {
            GridInfo::RowCol neighbour = std::make_pair(
                curr.first + neighbourOffset.first,
                curr.second + neighbourOffset.second);

            // Valid indices?
            if (neighbour.first < 0 || neighbour.first >= static_cast<int>(gridInfo.height))
            { continue; }
            if (neighbour.second < 0 || neighbour.second >= static_cast<int>(gridInfo.width))
            { continue; }

            // Within rectangle?
            if (!pointInRect(
                gridInfo.toPoint(neighbour),
                rect))
            { continue; }

            // Already marked?
            if (visited.find(neighbour) != visited.end())
            { continue; }

            toVisit.push(neighbour);
        }
    }
}

double castDistance(const tf2::Transform &start,
                    const double minDistance,
                    const double maxDistance,
                    const GridInfo &gridInfo,
                    const std::vector<int8_t> &world,
                    const int8_t detectionThreshold,
                    const bool unknownIsBlocked)
{
    tf2::Vector3 p0 = start.getOrigin();
    tf2::Vector3 a = tf2::quatRotate(start.getRotation(), tf2::Vector3(1, 0, 0)).normalized();

    tf2::Vector3 minPoint = p0 + minDistance * a;
    tf2::Vector3 maxPoint = p0 + maxDistance * a;

    double maxT = (maxPoint.getX() - minPoint.getX()) / a.getX();
    double t = 0.0;
    double tIncrement = gridInfo.mPerCell / 2;

    while (t <= maxT)
    {
        double x = minPoint.getX() + t * a.getX();
        double y = minPoint.getY() + t * a.getY();
        double percent = t / maxT;

        t += tIncrement;

        GridInfo::RowCol rowCol = gridInfo.toRowCol(tf2::Vector3(x, y, 0.0));
        if (rowCol.first < 0 || rowCol.first >= static_cast<int>(gridInfo.height))
        { continue; }
        if (rowCol.second < 0 || rowCol.second >= static_cast<int>(gridInfo.width))
        { continue; }

        int8_t value = world[gridInfo.toCellIdx(rowCol)];

        if ((unknownIsBlocked && value == -1) || value >= detectionThreshold)
        { return minDistance + (maxDistance - minDistance) * percent; }
    }

    return maxDistance * 2;
}

Rect rectFromCarInfo(
    const std::pair<double, double> &offset,
    const std::pair<double, double> &size)
{
    return {
        tf2::Vector3(
            offset.first - size.first / 2,
            offset.second + size.second / 2,
            0.0),
        tf2::Vector3(
            offset.first - size.first / 2,
            offset.second - size.second / 2,
            0.0),
        tf2::Vector3(
            offset.first + size.first / 2,
            offset.second - size.second / 2,
            0.0),
        tf2::Vector3(
            offset.first + size.first / 2,
            offset.second + size.second / 2,
            0.0)};
}

}
