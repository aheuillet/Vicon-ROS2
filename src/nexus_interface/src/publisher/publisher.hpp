#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <memory>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "nexus_interface/msg/position.hpp"
#include "nexus_interface/msg/root_segment.hpp"
#include "utils.hpp"

class Publisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<nexus_interface::msg::Position>::SharedPtr position_publisher_;
    rclcpp::Publisher<nexus_interface::msg::RootSegment>::SharedPtr root_segment_publisher_;
public:
    Publisher(std::string topic_name);
    void PublishPosition(Position p);
    void PublishRootSegment(RootSegment r);
    std::string AdaptSegmentName(std::string s);
    ~Publisher();
};  



#endif