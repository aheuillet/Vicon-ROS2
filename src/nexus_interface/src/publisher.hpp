#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include  <memory>
#include "rclcpp/rclcpp.hpp"
#include "nexus_interface/msg/position.hpp"
#include "utils.hpp"

#define POSITION_NUMBER 2

class Publisher : public rclcpp::Node
{
private:
    rclcpp::Publisher<nexus_interface::msg::Position>::SharedPtr position_publisher_;
public:
    Publisher(std::string topic_name);
    void PublishPosition(Position p);
    ~Publisher();
};  



#endif