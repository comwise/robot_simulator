#include "motion/motion_diff.h"
#include <thread>
#include <cmath>
#include <iostream>

namespace robot {
namespace motion {

static const double PI = 3.1415926;

motion_diff::motion_diff(const motion_id_t id, motion_type_t type )
    : motion_impl(id, type)
{

}

motion_diff::~motion_diff()
{
    deinit();
}

int motion_diff::init(const motion_param_t &param)
{
    is_loop_ = true;
    thr_ = std::make_shared<std::thread>(&motion_diff::sim_thread, this);
    return 0;
}

int motion_diff::deinit()
{
    is_loop_ = false;
    if(thr_ && thr_->joinable()) {
        thr_->join();
    }
    thr_ = nullptr;
    return 0;
}

int motion_diff::start()
{
    last_sim_time_ = std::chrono::steady_clock::now();
    start_thread_ = true;
    return 0;
}

int motion_diff::stop()
{
    start_thread_ = false;
    return 0;
}

void motion_diff::sim_thread()
{
    while(is_loop_) {
        
        vel_t cmd(vel_);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(period_));

        if (!start_thread_) {
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        double delta_t = std::chrono::duration_cast<
            std::chrono::milliseconds>(now - last_sim_time_).count()/1000.0;
        
        double delta_angle = normal_theta(cmd.w * delta_t);
        double angle = pos_.angle + delta_angle;
        angle = normal_theta(angle);

        double delta_x = cmd.v * delta_t * cos(angle);
        double delta_y = cmd.v * delta_t * sin(angle);

        pos_.x += delta_x;
        pos_.y += delta_y;
        pos_.angle = angle;

        last_sim_time_ = now;
#if 1
        std::cout << "v(" << cmd.v<< ", " << cmd.w*180/PI
            << ") delta(" << delta_x << ", " << delta_y << ", " << delta_angle*180/PI
            << ") pos(" << pos_.x << ", " << pos_.y << ", " << pos_.angle*180/PI 
            << ")" << std::endl;
#endif
    }
}

double motion_diff::normal_theta(double theta)
{
    double angle = theta;
    while(angle < -2*PI) {
        angle += 2*PI;
    }

    while(angle > 2*PI) {
        angle -= 2*PI;
    }

    return angle;
}


} // namespace motion
} // namespace robot
