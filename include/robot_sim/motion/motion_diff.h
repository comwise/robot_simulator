#ifndef __COMWISE__MOTION_DIFF__H__
#define __COMWISE__MOTION_DIFF__H__

#include "motion_impl.h"

namespace robot {
namespace motion {

class motion_diff : public motion_impl 
{
public:
    explicit motion_diff(const motion_id_t id, motion_type_t type = kChassisDiff);
    virtual ~motion_diff();

    virtual int start() override;
    virtual int stop() override;

private:
    int init(const motion_param_t &param);
    int deinit();

    void sim_thread();

    double normal_theta(double theta);

protected:
    int32_t period_{20};
    std::shared_ptr<std::thread> thr_{nullptr};
    bool is_loop_{false};
    bool start_thread_{false};

    std::chrono::steady_clock::time_point last_sim_time_;

    pos_2d_t pos_;
    vel_t vel_;
};

} // namespace motion
} // namespace robot
 
#endif // __COMWISE__MOTION_DIFF__H__
