#include "motion/motion_impl.h"
#include <thread>

static const double PI = 3.1415926;

int main()
{
    int i = 0;
    robot::motion::motion_impl impl(1);
    impl.set_pos(0, 0, 0);
    impl.set_period(20);
    impl.set_speed(PI/4, PI/4);
    impl.start();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        i++;
        if(i == 82) {
            printf("count ok, i=%d\n", i);
            impl.stop();
        }
    }
    impl.stop();
    return 0;
}

