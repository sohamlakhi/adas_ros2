#ifndef ADAS_PARAMS_HPP_
#define ADAS_PARAMS_HPP_

#include <ros/ros.h>
#include <nav_msgs/MapMetaData.h>

#include <string>
#include <vector>
#include <map>
#include <stdexcept>

namespace adas_common
{

std::string getCarPath(unsigned int carId, const std::string &postfix);
std::string getPathById(const std::string &base, unsigned int id, const std::string &path);

bool boolParam(ros::NodeHandle &node, const std::string &name);
bool boolParamWithDefault(ros::NodeHandle &node, const std::string &name, const bool defaultValue);

template <typename T>
T integerParam(ros::NodeHandle &node,
               const std::string &name,
               const T min = std::numeric_limits<T>::min(),
               const T max = std::numeric_limits<T>::max())
{
    static_assert(std::numeric_limits<T>::is_integer, "Given type is not integer.");

    int value;
    bool result = node.getParam(name, value);
    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }
    if (static_cast<T>(value) < min)
    { throw std::runtime_error(name + " exceeds the minimum value."); }
    if (static_cast<T>(value) > max)
    { throw std::runtime_error(name + " exceeds the maximum value."); }

    return static_cast<T>(value);
}

template <typename T>
T integerParamWithDefault(ros::NodeHandle &node,
               const std::string &name,
               const T defaultValue,
               const T min = std::numeric_limits<T>::min(),
               const T max = std::numeric_limits<T>::max())
{
    static_assert(std::numeric_limits<T>::is_integer, "Given type is not integer.");

    int value;
    bool result = node.getParam(name, value);
    if (!result)
    { return defaultValue; }
    if (static_cast<T>(value) < min)
    { throw std::runtime_error(name + " exceeds the minimum value."); }
    if (static_cast<T>(value) > max)
    { throw std::runtime_error(name + " exceeds the maximum value."); }

    return static_cast<T>(value);
}

template <typename T>
std::vector<T> vectorParam(ros::NodeHandle &node, const std::string &name)
{
    static_assert(!std::numeric_limits<T>::is_integer, "Must call vectorIntegerParam for vector of ints.");
    std::vector<T> result;
    if (!node.getParam(name, result))
    { throw std::runtime_error{"Failed to retrieve " + name}; }

    return result;
}

template <typename T>
std::vector<T> vectorParam(ros::NodeHandle &node, const std::string &name, const unsigned int size)
{
    std::vector<T> result = vectorParam<T>(node, name);

    if (result.size() != size)
    { throw std::runtime_error(name + " is a list of unexpected size."); }

    return result;
}

template <typename T>
std::vector<T> vectorIntegerParam(ros::NodeHandle &node, const std::string &name)
{
    static_assert(std::numeric_limits<T>::is_integer, "Must call vectorParam for vector of non-ints.");
    std::vector<int> value;

    bool result = node.getParam(name, value);
    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }

    std::vector<T> valueCasted;
    valueCasted.reserve(value.size());

    for (std::vector<int>::const_iterator it = value.begin(); it != value.end(); ++it)
    {
        if (static_cast<T>(*it) < std::numeric_limits<T>::min())
        { throw std::runtime_error(name + "'s element exceeds the minimum value."); }
        if (static_cast<T>(*it) > std::numeric_limits<T>::max())
        { throw std::runtime_error(name + "'s element exceeds the maximum value."); }
        valueCasted.push_back(static_cast<T>(*it));
    }

    return valueCasted;
}

template <typename T>
std::vector<T> vectorIntegerParam(ros::NodeHandle &node, const std::string &name, unsigned int size)
{
    std::vector<T> result = vectorIntegerParam<T>(node, name);

    if (result.size() != size)
    { throw std::runtime_error(name + " is a list of unexpected size."); }

    return result;
}

std::vector<double> vectorPosDoubleParam(ros::NodeHandle &node, const std::string &name);
std::vector<double> vectorPosDoubleParam(ros::NodeHandle &node, const std::string &name, unsigned int size);

double doubleParam(
    ros::NodeHandle &node,
    const std::string &name,
    double min = -std::numeric_limits<double>::max(),
    double max = std::numeric_limits<double>::max());
double doubleParam(
    const std::string &name,
    double min = -std::numeric_limits<double>::max(),
    double max = std::numeric_limits<double>::max());
double doubleOverridableParam(
    const std::string &baseName,
    const std::string &overrideName,
    double min = -std::numeric_limits<double>::max(),
    double max = std::numeric_limits<double>::max());
std::string nonEmptyStringParam(ros::NodeHandle &node, const std::string &name);
std::string defaultStringParam(
    ros::NodeHandle &node,
    const std::string &name,
    const std::string &defaultValue);

struct CarTagInfo
{
    // ID of the tag
    unsigned int tagId;
    // ID of the car
    unsigned int carId;
    // CCW rotation (by 90 degrees) of the tag
    unsigned int rotation;
    // Size, in m, of the tag
    double size;
    // x distance to center of turn from the tag
    double centerOfTurnOffset;
};

std::vector<CarTagInfo> getCarTagInfo(ros::NodeHandle &node);

nav_msgs::MapMetaData getTreadmillGridInfo(ros::NodeHandle &node);

}

#endif
