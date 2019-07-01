#include "publisher.hpp"

Publisher::Publisher(std::string topic_name): Node("vicon_publisher")
{
    position_publisher_ = this->create_publisher<nexus_interface::msg::Position>(topic_name);
}

void Publisher::PublishPosition(Position p) 
{
    auto msg = std::make_shared<nexus_interface::msg::Position>();
    msg->x = p.translation[0];
    msg->y = p.translation[1];
    msg->z = p.translation[2];
    position_publisher_->publish(*msg);
}

Publisher::~Publisher()
{   
}