#include "publisher.hpp"

Publisher::Publisher(std::string topic_name): Node("vicon_publisher")
{
    position_publisher_ = this->create_publisher<nexus_interface::msg::Position>(topic_name, 10);
    root_segment_publisher_ = this->create_publisher<nexus_interface::msg::RootSegment>(topic_name, 10);
}

void Publisher::PublishPosition(Position p) 
{
    auto msg = std::make_shared<nexus_interface::msg::Position>();
    msg->x_trans = p.translation[0];
    msg->y_trans = p.translation[1];
    msg->z_trans = p.translation[2];
    msg->x_rot = p.rotation[0];
    msg->y_rot = p.rotation[1];
    msg->z_rot = p.rotation[2];
    msg->w = p.rotation[3];
    msg->subject_name = p.subject_name;
    msg->segment_name = AdaptSegmentName(p.segment_name);
    msg->frame_number = p.frame_number;
    msg->translation_type = p.translation_type;
    position_publisher_->publish(*msg);
}

void Publisher::PublishRootSegment(RootSegment r) 
{
    auto msg = std::make_shared<nexus_interface::msg::RootSegment>();
    msg->name = r.name;
    msg->subject_name = r.subject_name;
    root_segment_publisher_->publish(*msg);
}

std::string Publisher::AdaptSegmentName(std::string s) 
{
    if ((ci_find_substr(s, "l_femur")) || (ci_find_substr(s, "leftupleg")))
    {
        return std::string("LeftUpLeg");
    }
    else if ((ci_find_substr(s, "r_femur")) || (ci_find_substr(s, "rightupleg")))
    {
        return std::string("RightUpLeg");
    }
    else if ((ci_find_substr(s, "l_tibia")) || (ci_find_substr(s, "leftleg")))
    {
        return std::string("LeftLeg");
    }
    else if ((ci_find_substr(s, "r_tibia")) || (ci_find_substr(s, "rightleg")))
    {
        return std::string("RightLeg");
    }
    else if ((ci_find_substr(s, "l_foot")) || (ci_find_substr(s, "leftfoot")))
    {
        return std::string("LeftForeFoot");
    }
    else if ((ci_find_substr(s, "r_foot")) || (ci_find_substr(s, "rightfoot")))
    {
        return std::string("RightForeFoot");
    }
    else if ((ci_find_substr(s, "l_toe")) || (ci_find_substr(s, "lefttoebase")))
    {
        return std::string("LeftToeBaseEnd");
    }
    else if ((ci_find_substr(s, "r_toe")) || (ci_find_substr(s, "righttoebase")))
    {
        return std::string("RightToeBaseEnd");
    }
    else if ((ci_find_substr(s, "lowerback")) || (ci_find_substr(s, "spine")))
    {
        return std::string("Spine");
    }
    else if ((ci_find_substr(s, "l_humerus")) || (ci_find_substr(s, "leftarm")))
    {
        return std::string("LeftArm");
    }
    else if ((ci_find_substr(s, "r_humerus")) || (ci_find_substr(s, "rightarm")))
    {
        return std::string("RightArm");
    }
    else if ((ci_find_substr(s, "l_elbow")) || (ci_find_substr(s, "leftforearm")))
    {
        return std::string("LeftForeArm");
    }
    else if ((ci_find_substr(s, "r_elbow")) || (ci_find_substr(s, "rightforearm")))
    {
        return std::string("RightForeArm");
    }
    else if ((ci_find_substr(s, "l_wrist")) || (ci_find_substr(s, "lefthand")))
    {
        return std::string("LeftHand");
    }
    else if ((ci_find_substr(s, "r_wrist")) || (ci_find_substr(s, "righthand")))
    {
        return std::string("RightHand");
    }
    else if ((ci_find_substr(s, "l_hand")) || (ci_find_substr(s, "lefthandmiddle1")))
    {
        return std::string("LeftHandMiddle1");
    }
    else if ((ci_find_substr(s, "r_hand")) || (ci_find_substr(s, "righthandmiddle1")))
    {
        return std::string("RightHandMiddle1");
    }
    else if (ci_find_substr(s, "head_end")) 
    {
        return std::string("HeadEnd");
    }
    else if ((ci_find_substr(s, "head")) || (ci_find_substr(s, "collar")) || (ci_find_substr(s, "wrist_end")))
    {
        return s;
    }
    return std::string("");
}

Publisher::~Publisher()
{   
}

