#include "node/node.h"
#include "ros/ros_node.h"
#include "srv/robot_manage.h"

int main()
{
    auto mgr = std::make_shared<robot::srv::robot_manage>();
    mgr->add_robot(1);
    mgr->set_robot_pos(1, 0, 0, 0);
    mgr->set_robot_bound(1, -2, -2, 4, 4);
    //mgr->set_robot_vel(1, 0.1, 0);
    ROS_NODE->start();
    robot::node::sim_node node(mgr);
    mgr->start();
    ROS_NODE->spin();
    ROS_NODE->shutDown();
    return 0;
}
