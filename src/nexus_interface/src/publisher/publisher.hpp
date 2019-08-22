#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <memory>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "nexus_interface/msg/position.hpp"
#include "utils.hpp"

/// ROS2 node class that allow segment data to be published in a ROS2 topic.
class Publisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<nexus_interface::msg::Position>::SharedPtr position_publisher_;

public:
    Publisher(std::string topic_name);

    /// Publishes the given position in the ROS2 topic whose name is indicated in
    /// the constructor.
    void PublishPosition(Position p);
    ~Publisher();
};

#endif