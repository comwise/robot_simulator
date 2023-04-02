#ifndef __COMWISE_NODE__SIM_NODE__H__
#define __COMWISE_NODE__SIM_NODE__H__

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include <mutex>
#include "srv/robot_manage.h"

namespace std {
    class thread;
}

namespace ros {
    class NodeHandle;
    class ServiceServer;
    class Publisher;
    class Subscriber;
    class Timer;
}

namespace tf {
    class TransformBroadcaster;
}

namespace robot {

namespace srv {
    class robot_manage;
}

namespace node {

class sim_node
{
public:
    sim_node(std::shared_ptr<srv::robot_manage> robot_mng);
    ~sim_node();

protected:


private:
    bool init();
    bool deinit();

    void pub_odom(const pos_t &pos, const vel_t &vel);
    void pub_cmd_feedback(const vel_t &vel);
    void pub_safety();

private:
    // config
    std::string cfg_file_ = "$/robot_sim.json";

    // service
    std::shared_ptr<srv::robot_manage> robot_srv_{nullptr};

    // ros
    std::shared_ptr<ros::NodeHandle> ros_handle_{nullptr};
    std::vector<std::shared_ptr<ros::ServiceServer>> ros_srvs_;
    std::vector<std::shared_ptr<ros::Subscriber>> ros_subs_;
    std::map<std::string, std::shared_ptr<ros::Publisher>> ros_pubs_;
    std::mutex pubs_mtx_;

    std::shared_ptr<tf::TransformBroadcaster> odom_broadcaster_;
};

} // namespace node
} // namespace robot
 
#endif // __COMWISE_NODE__SIM_NODE__H__
