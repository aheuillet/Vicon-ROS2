#include "publisher.hpp"

Publisher::Publisher(std::string topic_name): Node("vicon_publisher")
{
    position_publisher_ = this->create_publisher<nexus_interface::msg::Position>(topic_name);
}

void Publisher::PublishPosition(Position p) 
{
    auto msg = std::make_shared<nexus_interface::msg::Position>();
    for (size_t i = 0; i < POSITION_NUMBER; i++)
    {
        msg->position_point[i] = p.position_point[i];
    }
    msg->radius = p.radius;
    position_publisher_->publish(*msg);
}

Publisher::~Publisher()
{   
}