#include "ros/ros_node.h"
#include <ros/ros.h>
#include <signal.h>
#include <csignal>

namespace robot {
namespace ros_node {

std::shared_ptr<RosNode> RosNode::instance_ = nullptr;

RosNode::RosNode(const std::string &name)
{
    int argc = 0;
    ::ros::init(argc, NULL, name);
    if (nullptr == ros_handle_) {
        ros_handle_ = std::make_shared<ros::NodeHandle>();
    }
}

RosNode::~RosNode()
{
    shutDown();
}

std::shared_ptr<ros::NodeHandle> RosNode::nodeHandle()
{
    return ros_handle_;
};

bool RosNode::start()
{
    ::ros::start();
}

void RosNode::spin(bool enable_once)
{
    if (enable_once)
        ::ros::spinOnce();
    else
        ::ros::spin();
}

void RosNode::shutDown()
{
    ::ros::shutdown();
}

}
}
