#include "node/node.h"
#include <ros/ros.h>
#include "ros/ros_node.h"
#include "var/var.h"
#include "ros_node/add_robot.h"
#include "ros_node/del_robot.h"
#include "ros_node/set_bound.h"
#include "ros_node/set_pos.h"
#include "ros_node/set_vel.h"
#include "ros_node/start.h"
#include "ros_node/move_cmd.h"
#include "ros_node/move_feedback.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


namespace robot {
namespace node {

using namespace ::ros_node;
sim_node::sim_node(std::shared_ptr<srv::robot_manage> robot_mng)
    : robot_srv_(robot_mng)
{
    init();
}

sim_node::~sim_node()
{
    deinit();
}

bool sim_node::init()
{
    ros_handle_ = ROS_NODE->nodeHandle();
    if(nullptr == ros_handle_) {
        printf("current ros node object is null\n");
        return false;
    }

    using namespace ::ros_node;
    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<add_robot::Request, add_robot::Response>(
            "sim/add_robot", [&](add_robot::Request &req, add_robot::Response &res) {
                if (robot_srv_) {
                    robot_srv_->add_robot(req.id, req.x, req.y, req.angle, req.w, req.h, req.v, req.vw);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true; 
            }, ros::VoidConstPtr())));

    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<set_bound::Request, set_bound::Response>(
            "sim/set_bound", [&](set_bound::Request &req, set_bound::Response &res) {
                if(robot_srv_) {
                    robot_srv_->set_robot_bound(req.id, req.x, req.y, req.w, req.h);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true;
            }, ros::VoidConstPtr())));

    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<set_pos::Request, set_pos::Response>(
            "sim/set_pos", [&](set_pos::Request &req, set_pos::Response &res) {
                if(robot_srv_) {
                    robot_srv_->set_robot_pos(req.id, req.x, req.y, req.angle);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true; 
            }, ros::VoidConstPtr())));

    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<set_vel::Request, set_vel::Response>(
            "sim/set_speed", [&](set_vel::Request &req, set_vel::Response &res) {
                if(robot_srv_) {
                    robot_srv_->set_robot_vel(req.id, req.v, req.vw);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true;
            }, ros::VoidConstPtr())));

    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<del_robot::Request, del_robot::Response>(
            "sim/remove_robot", [&](del_robot::Request &req, del_robot::Response &res) {
                if(robot_srv_) {
                    robot_srv_->remove_robot(req.id);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true; 
            },  ros::VoidConstPtr())));

    ros_srvs_.push_back(std::make_shared<ros::ServiceServer>(
        ros_handle_->advertiseService<start::Request, start::Response>(
            "sim/start_robot", [&](start::Request &req, start::Response &res) {
                if(robot_srv_) {
                    robot_srv_->start(req.id);
                    res.response = true;
                } else {
                    res.response = false;
                }
                return true; 
            }, ros::VoidConstPtr())));

    ros_pubs_[kOdomTopic] = std::make_shared<ros::Publisher>(
        ros_handle_->advertise<nav_msgs::Odometry>(kOdomTopic, 20));

    ros_subs_.emplace_back(std::make_shared<ros::Subscriber>(
        ros_handle_->subscribe<move_cmd>(kCmdVelTopic, 20,
            [&](const move_cmd::ConstPtr& vel) {
                if(robot_srv_) {
                    robot_srv_->set_robot_vel(1, vel->cmd_velocity, vel->cmd_omega);
                }
        })));
    
    ros_pubs_[kCmdFeedbackTopic] = std::make_shared<ros::Publisher>(
        ros_handle_->advertise<move_feedback>(kCmdFeedbackTopic, 20));

    odom_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();

    if (robot_srv_) {
        robot_srv_->set_data_cb([&](const robot_vector_t &list) {
            for(auto &item : list) {
                pub_odom(item.pos, item.vel);
                pub_cmd_feedback(item.vel);
            }
        });
    }

    return true;
}

bool sim_node::deinit()
{
    return true;
}

void sim_node::pub_odom(const pos_t &pos, const vel_t &vel)
{
    auto current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(vel.angle);
    if (odom_broadcaster_) {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pos.x;
        odom_trans.transform.translation.y = pos.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster_->sendTransform(odom_trans);
    }

    if (odom_broadcaster_) {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "odom";

        odom_trans.transform.translation.x = 0.0;
        odom_trans.transform.translation.y = 0.0;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        odom_broadcaster_->sendTransform(odom_trans);
    }


    auto pub = ros_pubs_[kOdomTopic];
    if (pub) {
        nav_msgs::Odometry msg;

        msg.header.stamp = current_time;
        msg.header.frame_id = "base_link";
        msg.child_frame_id = "odom";

        msg.pose.pose.position.x = pos.x;
        msg.pose.pose.position.y = pos.y;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation = odom_quat;

        msg.twist.twist.linear.x = vel.v*std::cos(vel.angle);
        msg.twist.twist.linear.y = vel.v*std::sin(vel.angle);
        msg.twist.twist.angular.z = vel.angle;

        pub->publish(msg);
    }
}

void sim_node::pub_cmd_feedback(const vel_t &vel)
{
    auto pub = ros_pubs_[kCmdFeedbackTopic];
    if (pub) {
        move_feedback data;
        data.velocity = vel.v;
        data.omega = vel.w;
        pub->publish(data);
    }
}

} // namespace node
} // namespace robot
