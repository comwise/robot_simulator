#include "srv/robot_manage.h"
#include <thread>
#include<math.h>
static const double PI = 3.1415926;

int main ()
{
    int i = 0;
    
    robot::srv::robot_manage mgr;

    mgr.add_robot(1, 0, 0, 0, 4, 4, 100, 0);
    mgr.add_robot(2, 100, 0, PI, 4, 4, 100, 0);
    
    // mgr.add_robot(1);
    // mgr.set_robot_pos(1, 0, 0, 0);
    // mgr.set_robot_bound(1, -2, -2, 4, 4);
    // mgr.set_robot_vel(1, 100, 0);
    // mgr.set_robot_vel(1, 100, 0);

    // mgr.add_robot(2);
    // mgr.set_robot_pos(2, 100, 0, PI);
    // mgr.set_robot_bound(2, -2, 8, 4, 4);
    // mgr.set_robot_vel(2, 100, 0);
    // mgr.set_robot_vel(2, 100, 0);
    
    mgr.start();

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i++;
        if(i == 20) {
            printf("count ok, i=%d\n", i);
            mgr.stop();
        }
    }
    mgr.stop();

    return 0;
}
