#ifndef __COMWISE__ROBOT_BOUND_H__
#define __COMWISE__ROBOT_BOUND_H__

#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <functional>
#include <chrono>
#include "var/var_object.h"

namespace robot {
namespace srv {

class robot_bound
{
public:
    explicit robot_bound();
    virtual ~robot_bound();

    bool judge_bound(rect_t rc_1, rect_t rc_2, double red);
};

} // namespace srv
} // namespace robot
 
#endif // __COMWISE__ROBOT_BOUND_H__
