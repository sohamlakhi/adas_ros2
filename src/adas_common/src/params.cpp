#include "adas_common/params.hpp"

#include <sstream>

#include <geometry_msgs/Pose.h>

using std::vector;
using std::map;
using std::string;
using std::stringstream;
using XmlRpc::XmlRpcValue;

// TODO: Use of the bare interface (e.g. ros::param::get() instead of node.getParam()) would be more convenient

namespace adas_common
{

string getCarPath(unsigned int carId, const std::string &postfix)
{
    return getPathById("/car/", carId, postfix);
}

std::string getPathById(const std::string &base, unsigned int id, const std::string &path)
{
    std::stringstream nodeStream;

    nodeStream << base << "/" << id << "/" << path;
    return nodeStream.str();
}

bool boolParam(ros::NodeHandle &node, const string &name)
{
    bool value;
    bool result = node.getParam(name, value);
    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }

    return value;
}

bool boolParamWithDefault(ros::NodeHandle &node, const std::string &name, const bool defaultValue)
{
    bool value;
    node.param(name, value, defaultValue);

    return value;
}

std::vector<double> vectorPosDoubleParam(ros::NodeHandle &node, const std::string &name)
{
    std::vector<double> value;

    bool result = node.getParam(name, value);
    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }

    for (const double v: value)
    {
        if (v < 0.0)
        { throw std::runtime_error(name + "'s value has a negative."); }
    }

    return value;
}

std::vector<double> vectorPosDoubleParam(ros::NodeHandle &node, const std::string &name, unsigned int size)
{
    std::vector<double> value = vectorPosDoubleParam(node, name);

    if (value.size() != size)
    { throw std::runtime_error(name + " is a list of unexpected size."); }

    return value;
}

double doubleParam(ros::NodeHandle &node, const std::string &name, double min, double max)
{
    double doubleParsed;
    bool result = node.getParam(name, doubleParsed);

    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }

    if (doubleParsed < min)
    { throw std::runtime_error(name + " exceeds the minimum value."); }
    if (doubleParsed > max)
    { throw std::runtime_error(name + " exceeds the maximum value."); }

    return doubleParsed;
}

double doubleParam(const std::string &name, double min, double max)
{
    double doubleParsed;
    bool result = ros::param::get(name, doubleParsed);

    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }

    if (doubleParsed < min)
    { throw std::runtime_error(name + " exceeds the minimum value."); }
    if (doubleParsed > max)
    { throw std::runtime_error(name + " exceeds the maximum value."); }

    return doubleParsed;
}

double doubleOverridableParam(
    const std::string &baseName,
    const std::string &overrideName,
    double min,
    double max)
{
    double doubleParsed;
    bool overridden = ros::param::get(overrideName, doubleParsed);
    if (overridden)
    { return doubleParsed; }

    return doubleParam(baseName, min, max);
}

string nonEmptyStringParam(ros::NodeHandle &node, const string &name)
{
    string stringParsed;
    bool result = node.getParam(name, stringParsed);

    if (!result)
    { throw std::runtime_error("Failed to retrieve " + name); }
    if (stringParsed.empty())
    { throw std::runtime_error(name + " is an empty string."); }

    return stringParsed;
}

std::string defaultStringParam(
    ros::NodeHandle &node,
    const std::string &name,
    const std::string &defaultValue)
{
    std::string ret;
    node.param<std::string>(name, ret, defaultValue);
    return ret;
}

std::vector<CarTagInfo> getCarTagInfo(ros::NodeHandle &node)
{
    unsigned int numCars = integerParam<unsigned int>(node, "/num_cars");
    std::vector<CarTagInfo> tagInfos;
    tagInfos.reserve(numCars);

    for (unsigned int i = 0; i < numCars; i++)
    {
        tagInfos.push_back(CarTagInfo());

        tagInfos[i].tagId = integerParam<unsigned int>(node, getCarPath(i, "tag_id"));
        tagInfos[i].carId = integerParam<unsigned int>(node, getCarPath(i, "car_id"));
        tagInfos[i].rotation = integerParam<unsigned int>(node, getCarPath(i, "tag_rotation"), 0.0f);
        tagInfos[i].size = doubleParam(node, getCarPath(i, "tag_size"), 0.0f);
        tagInfos[i].centerOfTurnOffset = doubleParam(node, getCarPath(i, "rear_axle_from_tag"));
    }

    return tagInfos;
}

nav_msgs::MapMetaData getTreadmillGridInfo(ros::NodeHandle &node)
{
    nav_msgs::MapMetaData mapInfo;

    mapInfo.resolution = doubleParam(node, "top_down_estimator/grid_m_per_cell");
    mapInfo.width = std::ceil(doubleParam(node, "treadmill/length") / mapInfo.resolution);
    mapInfo.height = std::ceil(vectorPosDoubleParam(node, "treadmill/lanes").back() / mapInfo.resolution);

    geometry_msgs::Pose origin;
    origin.position.x = 0;
    origin.position.y = 0;
    origin.position.z = 0;
    origin.orientation.x = 0;
    origin.orientation.y = 0;
    origin.orientation.z = 0;
    origin.orientation.w = 1;
    mapInfo.origin = origin;

    return mapInfo;
}

}
