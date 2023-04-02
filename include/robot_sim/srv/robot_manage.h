#ifndef __COMWISE_SRV__MOTION_MANAGE__H__
#define __COMWISE_SRV__MOTION_MANAGE__H__

#include <cstdint>
#include <memory>
#include <atomic>
#include <string>
#include <map>
#include <chrono>
#include <mutex>
#include <functional>
#include "motion/motion_type.h"
#include "var/var_object.h"

namespace std {
    class thread;
}

namespace robot {

namespace motion {
    class motion;
}

namespace srv {

class robot_manage
{
public:
    using pos_t = pos_2d_t;
    using motion_ptr_t = std::shared_ptr<motion::motion>;
    using motion_list_t = std::map<int, motion_ptr_t>;

    struct robot_info : public robot::robot_data {
        motion_ptr_t chassis;
    };
    using robot_list_t = std::map<int, robot_info>;

    using data_cb_t = std::function<void(const robot_vector_t &)>;
public:
    explicit robot_manage();
    virtual ~robot_manage();

    int add_robot(int id, double x, double y, double angle, int w, int  h, double v, double vw, int type = motion::kChassisNormal);
    int add_robot(int id, int type = motion::kChassisNormal);
    int remove_robot(int id, int type = motion::kChassisNormal);

    void set_robot_pos(int id, double x, double y, double angle);
    void set_robot_bound(int id, int x, int y, int w, int h);
    void set_robot_vel(int id, double v, double w);

    void start(int id = -1);
    void stop(int id = -1);

    void set_data_cb(data_cb_t cb) { data_cb_ = cb; }

private:
    void init();
    void deinit();

    void start_robot(int id);
    void stop_robot(int id);

    void bound_monitor();
    bool pos_monitor(double x, double y, double red, double w = 0.0, double h = 0.0);

    void show_speed();

private:
    motion_list_t motion_lst_;
    robot_list_t robot_lst_;
    std::mutex lst_lck_;

    std::shared_ptr<std::thread> bound_thr_{nullptr};
    std::atomic_bool is_loop_{false};

    std::shared_ptr<std::thread> data_thr_{nullptr};
    std::atomic_bool is_data_loop_{false};
    data_cb_t data_cb_;
};

} // namespace srv
} // namespace robot
 
#endif // __COMWISE_SRV__MOTION_MANAGE__H__
