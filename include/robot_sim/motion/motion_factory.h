#ifndef __COMWISE__MOTION_FACTORY__H__
#define __COMWISE__MOTION_FACTORY__H__

#include <cstdint>
#include <memory>
#include <string>
#include "motion_type.h"
#include "motion_impl.h"
#include "motion_diff.h"

namespace robot {
namespace motion {

class motion_factory final
{
public:
    using motion_ptr_t = std::shared_ptr<motion>;
    using motion_id_t = motion::motion_id_t;
    using motion_type_t = motion::motion_type_t;

public:
    static motion_ptr_t create_object(motion_id_t id, 
        motion_type_t type = kChassisNormal)
    {
        motion_ptr_t obj {nullptr};
        switch (type)
        {
        case kChassisNormal:
            obj = std::make_shared<motion_impl>(id);
            break;
        case kChassisDiff:
            obj = std::make_shared<motion_diff>(id);
            break;
        default:
            obj = std::make_shared<motion_impl>(id);
            break;
        }
        return std::move(obj);
    }
};

} // namespace motion
} // namespace robot
 
#endif // __COMWISE__MOTION_FACTORY__H__
