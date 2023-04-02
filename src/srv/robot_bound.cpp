
#include "srv/robot_bound.h"
#include<math.h>

namespace robot {
namespace srv {

robot_bound::robot_bound()
{

}

robot_bound::~robot_bound()
{

}

bool robot_bound::judge_bound(rect_t rc1, rect_t rc2, double red)
{
    double x_1 = rc1.x + rc1.w/2.0;
    double y_1 = rc1.y + rc1.h/2.0;
    double r_1 = sqrt(rc1.w * rc1.w + rc1.h * rc1.h)/2.0;

    double x_2 = rc2.x + rc2.w/2.0;
    double y_2 = rc2.y + rc2.h/2.0;
    double r_2 = sqrt(rc2.w * rc2.w + rc2.h * rc2.h)/2.0;

#ifdef BOUND_DEBUG
    printf("(x1, y1)=(%f, %f), (x2, y2)=(%f, %f)", x_1, y_1, x_2, y_2);
#endif

    double distance = sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
#ifdef BOUND_DEBUG
    bool ret = distance > (r_1 + r_2 + red);
    printf("(distance >(r_1 + r_2 + red) = (%f, %f ,%f, %f)\n", distance, r_1, r_2, red);
#endif
    return distance > (r_1 + r_2 + red);
}

} // namespace srv
} // namespace robot
