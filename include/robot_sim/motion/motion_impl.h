#ifndef __COMWISE__MOTION_IMPL__H__
#define __COMWISE__MOTION_IMPL__H__

#include "motion.h"

namespace std {
    class thread;
}

namespace robot {
namespace motion {

class motion_impl : public motion
{
public:
    explicit motion_impl(const motion_id_t id, motion_type_t type = kChassisNormal);
    virtual ~motion_impl();

    virtual void set_period(int period) override;
    virtual int get_period() override;

    virtual void set_pos(double x, double y, double angle) override;
    virtual void set_pos(const pos_t &pos) override;
    virtual void get_pos(pos_t &pos) override;

    virtual void set_bound(double x, double y, double w, double h) override;
    virtual void set_bound(const rect_t &rc) override;
    virtual void get_bound(rect_t &rc) override;

    virtual void set_speed(double v, double w) override;
    virtual void set_speed(const vel_t &vel) override;
    virtual void get_speed(vel_t &vel) override;

    virtual int start() override;
    virtual int stop() override;

protected:
    //! init and uninit drvier
    virtual int init(const motion_param_t &param);
    virtual int deinit();

private:
    void sim_thread();

    double normal_theta(double theta);

protected:
    int32_t period_{10};
    std::shared_ptr<std::thread> thr_{nullptr};
    bool is_loop_{false};
    bool start_thread_{false};

    std::chrono::steady_clock::time_point last_sim_time_;

    pos_2d_t pos_;
    vel_t vel_;
    rect_t rc_;
    std::chrono::steady_clock::time_point last_cmd_time_;
};

} // namespace motion
} // namespace robot
 
#endif // __COMWISE__MOTION_IMPL__H__
