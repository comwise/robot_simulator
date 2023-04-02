#include "motion/motion_impl.h"
#include <thread>
#include <cmath>
#include <iostream>
#include <sstream>

namespace robot {
namespace motion {

static const double PI = 3.1415926;

#define CHASSIS_DEBUG

motion_impl::motion_impl(const motion_id_t id, motion_type_t type )
    : motion(id, type)
{

}

motion_impl::~motion_impl()
{
    deinit();
}

int motion_impl::init(const motion_param_t &param)
{
    is_loop_ = true;
    thr_ = std::make_shared<std::thread>(&motion_impl::sim_thread, this);
    return 0;
}

int motion_impl::deinit()
{
    is_loop_ = false;
    if(thr_ && thr_->joinable()) {
        thr_->join();
    }
    thr_ = nullptr;
    return 0;
}

void motion_impl::set_period(int period)
{
    period_ = period;
}

int motion_impl::get_period()
{
    return period_;
}

void motion_impl::set_pos(double x, double y, double angle)
{
    pos_.x = x;
    pos_.y = y;
    pos_.angle = angle;
}

void motion_impl::set_pos(const pos_t &pos)
{
    pos_ = pos;
}

void motion_impl::get_pos(pos_t &pos)
{
    pos = pos_;
}

void motion_impl::set_bound(double x, double y, double w, double h)
{
    rc_.x = x;
    rc_.y = y;
    rc_.w = w;
    rc_.h = h;
}

void motion_impl::set_bound(const rect_t &rc)
{
    rc_ = rc;
}

void motion_impl::get_bound(rect_t &rc)
{
    rc = rc_;
}

void motion_impl::set_speed(double v, double w)
{
    vel_.v = v;
    vel_.w = w;
    last_cmd_time_ = std::chrono::steady_clock::now();
}

void motion_impl::set_speed(const vel_t &vel)
{
    vel_ = vel;
    last_cmd_time_ = std::chrono::steady_clock::now();
}

void motion_impl::get_speed(vel_t &vel)
{
    vel = vel_;
}

int motion_impl::start()
{
    last_sim_time_ = std::chrono::steady_clock::now();
    start_thread_ = true;
    return 0;
}

int motion_impl::stop()
{
    start_thread_ = false;
    return 0;
}

void motion_impl::sim_thread()
{
    while(is_loop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(period_));
        
        vel_t cmd(vel_);

        if(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_cmd_time_).count() > 100) {
            cmd = vel_t();
        }

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
        cmd.angle = angle;
        //emit("vel_feedback", cmd);
        //emit("pos_feedback", pos_);

        last_sim_time_ = now;
#ifdef CHASSIS_DEBUG
        std::stringstream ss;
        using namespace std::chrono;
        auto time = duration_cast<milliseconds>(steady_clock().now().time_since_epoch()).count();
        ss << time << " id: " << id_ << " v(" << cmd.v<< ", " << cmd.w*180/PI
            << ") delta(" << delta_x << ", " << delta_y << ", " << delta_angle*180/PI
            << ") pos(" << pos_.x << ", " << pos_.y << ", " << pos_.angle*180/PI 
            << ")" << "\n";
        std::cout << ss.str() << std::endl;
#endif
    }
}

double motion_impl::normal_theta(double theta)
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
