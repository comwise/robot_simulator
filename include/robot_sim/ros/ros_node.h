#ifndef __COMWISE__ROS_NODE__H__
#define __COMWISE__ROS_NODE__H__

#include <memory>
#include <atomic>
#include <thread>
#include <string>

namespace ros {
    class NodeHandle;
}

namespace robot {
  namespace ros_node {

class RosNode
{
public:
    static std::shared_ptr<RosNode> instance(const std::string &ins = "sim_node") {
        if (nullptr == instance_) {
            instance_ = std::shared_ptr<RosNode>(new RosNode(ins));
            while (instance_ == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        return instance_;
    }
    ~RosNode();

    static bool start();
    static void spin(bool enable_once = false);
    static void shutDown();

    std::shared_ptr<ros::NodeHandle> nodeHandle();

private:
    RosNode(const std::string &name = "sim_node");

    static std::shared_ptr<RosNode> instance_;
    std::shared_ptr<ros::NodeHandle> ros_handle_{nullptr};
};

  } // namespace ros_node
} // namespace robot

#define ROS_NODE robot::ros_node::RosNode::instance()

#endif //__COMWISE__ROS_NODE__H__
