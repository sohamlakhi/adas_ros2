#ifndef ID_CALLBACK_HPP
#define ID_CALLBACK_HPP

#include <ros/ros.h>

namespace id_callback
{

// Used by a functor version of NodeHandle::subscribe. Must be copyable.
template <typename T, typename Msg>
struct IdCallback
{
    IdCallback() : id(0), callback(nullptr), obj(nullptr) {}
    IdCallback(
        size_t id,
        void (T::*callback)(unsigned int, const boost::shared_ptr<const Msg> &),
        T *obj)
        : id(id),
          callback(callback),
          obj(obj)
    {}

    void operator()(const boost::shared_ptr<const Msg> &msg) { (obj->*callback)(id, msg); }
    size_t id;
    void (T::*callback)(unsigned int, const boost::shared_ptr<const Msg> &msg);
    T *obj;
};

/*
 * Used by global-level nodes to subscribe to individual car nodes' topic.
 * Wraps around the normal callback, providing the car's ID information.
 */
template <typename T, typename Msg>
std::vector<ros::Subscriber> subscribeCars(
    ros::NodeHandle &node,
    const std::string &topic,
    void (T::*callback)(unsigned int, const boost::shared_ptr<const Msg> &),
    T *obj,
    size_t numCars,
    size_t queueLength = 1)
{
    std::vector<ros::Subscriber> subs;
    subs.reserve(numCars);

    for (size_t i = 0; i < numCars; i++)
    {
        subs.push_back(node.subscribe<Msg>(
            "/car/" + std::to_string(i) + "/" + topic,
            queueLength,
            IdCallback<T, Msg>(i, callback, obj))); // Copies
    }
    return subs;
}

}

#endif
