#ifndef __COMWISE__MOTION_ODOM__H__
#define __COMWISE__MOTION_ODOM__H__

#include <cstdint>
#include <memory>
#include "common/singleton.h"
#include "var/var_object.h"

namespace robot {
namespace motion {

class motion_odom : public common::singleton<motion_odom>
{
public:
    motion_odom();

    void OnTimeCaller(double time);

    inline void set_vel(double v, double w, double theta) {
        speed_v_ = v; speed_w_ = w; speed_theta_ = theta; }
    inline double get_v() { return speed_v_; }
    inline double get_w() { return speed_w_; }
    inline double get_theta() { return speed_theta_; }

    inline void set_pos(double x, double y, double t) {
        odo_x_ = x; odo_y_ = y; odo_theta_ = t; }
    inline double get_pos_X() { return odo_x_; }
    inline double get_pos_Y() { return odo_y_; }
    inline double get_pos_theta() { return odo_theta_; }

private:
    void init();

private:
    double speed_v_{0.0};     //合成速度
    double speed_theta_{0.0}; //转角
    double speed_w_{0.0};     //合成角速度

    double odo_x_{0.0};     //x里程
    double odo_y_{0.0};     //y里程
    double odo_theta_{0.0}; //角度里程，弧度
};

} // namespace motion
} // namespace robot

#define ODOM robot::motion::motion_odom::singleton()
 
#endif // __COMWISE__MOTION_ODOM__H__
