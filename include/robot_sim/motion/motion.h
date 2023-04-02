#ifndef __COMWISE__MOTION_BASE__H__
#define __COMWISE__MOTION_BASE__H__

#include <cstdint>
#include <memory>
#include <string>
#include "var/var_object.h"
#include "core/module.h"
#include "motion_type.h"

namespace robot {
namespace motion {

class motion : public core::module_core<int32_t, motion_param_t>, public core::event
{
public:
    using pos_t = pos_2d_t;
    using motion_id_t = module_id_t;
    using motion_type_t = module_type_t;

public:
    motion(const motion_id_t &id, motion_type_t type = 0) 
        : core::module_core<int32_t, motion_param_t>(id, type) { }
    virtual ~motion() { }

    virtual void set_period(int period) = 0;
    virtual int get_period() = 0;

    virtual void set_pos(double x, double y, double angle) = 0;
    virtual void set_pos(const pos_t &pos) = 0;
    virtual void get_pos(pos_t &pos) = 0;

    virtual void set_bound(double x, double y, double w, double h) = 0;
    virtual void set_bound(const rect_t &rect) = 0;
    virtual void get_bound(rect_t &rect) = 0;

    virtual void set_speed(double v, double w) = 0;
    virtual void set_speed(const vel_t &vel) = 0;
    virtual void get_speed(vel_t &vel) = 0;
};

} // namespace motion
} // namespace robot
 
#endif // __COMWISE__MOTION_BASE__H__
