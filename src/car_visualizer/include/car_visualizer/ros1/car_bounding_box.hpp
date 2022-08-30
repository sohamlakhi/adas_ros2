#ifndef CAR_BOUNDING_BOX_HPP_
#define CAR_BOUNDING_BOX_HPP_

#include <vector>
#include <geometry_msgs/msg/point.h>
#include <algorithm>

namespace car_bounding_box
{

using Point2D = std::pair<double, double>;

class BoundingBox
{
public:
    BoundingBox(const Point2D &boundOffset, const Point2D &boundSize);

    const std::vector<geometry_msgs::msg::point> &get() const;

private:
    std::vector<geometry_msgs::msg::point> box;
};

}

#endif
