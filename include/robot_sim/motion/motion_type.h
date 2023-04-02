#ifndef __COMWISE__MOTION_TYPE__H__
#define __COMWISE__MOTION_TYPE__H__

#include <cstdint>
#include <memory>

namespace robot {
namespace motion {

enum motion_type_t
{
    kMotionNone,
    kMotionSteer = 0,
    kMotionDiff = 1,
    kChassisNormal = 0,
    kChassisDiff = 1,
};

class motion_param_t
{
public:

};

class motion_data_t 
{
public:
};


} // namespace motion
} // namespace robot
 
#endif // __COMWISE__MOTION_TYPE__H__
