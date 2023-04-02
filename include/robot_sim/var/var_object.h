#ifndef __COMWISE__VAR_OBJECT__H__
#define __COMWISE__VAR_OBJECT__H__

#include <cstdint>
#include <memory>
#include <vector>

namespace robot {

class vel_t
{
public:
    double v{0.0};
    double w{0.0};
    union {
        double angle;
        double theta;
    };

    vel_t(double _v = 0.0, double _w = 0.0, double _angle = 0.0) 
        : v(_v), w(_w), angle(_angle) { }
};

class pos_2d_t {
public:
    double x{0.0};
    double y{0.0};
    union {
        double angle{0};
        double theta;
    };
    pos_2d_t(double _x = 0.0, double _y = 0.0, double _angle = 0.0) 
        : x(_x), y(_y), angle(_angle) { }
};
using pos_t = pos_2d_t;

class pos_3d_t {
public:
    double x{0.0};
    double y{0.0};
    double z{0.0};

    pos_3d_t(double _x = 0.0, double _y = 0.0, double _z = 0.0) 
        : x(_x), y(_y), z(_z) { }
};

class rect_t {
public:
    double x{0.0};
    double y{0.0};
    double w{0.0};
    double h{0.0};

    rect_t(double _x = 0.0, double _y = 0.0, double _w = 0.0, double _h = 0.0)
        : x(_x), y(_y), w(_w), h(_h) { }
};

class robot_data {
public:
    int id{0};
    vel_t vel;
    pos_t pos;
    rect_t rect;
};
using robot_vector_t = std::vector<robot_data>;

} // namespace robot

#endif // __COMWISE__VAR_OBJECT__H__
