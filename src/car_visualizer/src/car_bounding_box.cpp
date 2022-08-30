#include "car_visualizer/car_bounding_box.hpp"

#include <array>
#include <stdexcept>

namespace car_bounding_box
{

BoundingBox::BoundingBox(const Point2D &boundOffset, const Point2D &boundSize)
{
    if (boundSize.first <= 0.0 || boundSize.second <= 0.0)
    { throw std::invalid_argument("Boundary size must be a non-zero positive pair."); }

    const std::array<Point2D, 5> boundMult{
        std::make_pair(-1.0, -1.0),
        std::make_pair(1.0, -1.0),
        std::make_pair(1.0, 1.0),
        std::make_pair(-1.0, 1.0),
        std::make_pair(-1.0, -1.0)};

    box.reserve(boundMult.size());
    for (const Point2D &boundMultPoint: boundMult)
    {
        geometry_msgs::msg::Point point; //auto point = geometry_msgs::msg::Point(); -> invoking constructor
        point.x = boundOffset.first + boundMultPoint.first * boundSize.first / 2;
        point.y = boundOffset.second + boundMultPoint.second * boundSize.second / 2;
        point.z = 0.0;

        box.push_back(point);
    }
}

const std::vector<geometry_msgs::msg::Point> &BoundingBox::get() const
{
    return box;
}

}
