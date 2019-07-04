#include "publisher.hpp"

Publisher::Publisher(std::string topic_name): Node("vicon_publisher")
{
    position_publisher_ = this->create_publisher<nexus_interface::msg::Position>(topic_name, 10);
}

void Publisher::PublishPosition(Position p) 
{
    auto msg = std::make_shared<nexus_interface::msg::Position>();
    msg->x = p.translation[0];
    msg->y = p.translation[1];
    msg->z = p.translation[2];
    msg->subject_name = p.subject_name;
    msg->segment_name = p.segment_name;
    msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}

Publisher::~Publisher()
{   
}

/* int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    Publisher pub("toto");
    sleep(10); 
    return 0;
}
 */