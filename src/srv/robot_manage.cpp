#include "srv/robot_manage.h"
#include "srv/robot_bound.h"
#include "motion/motion_factory.h"
#include <thread>
#include <iostream>

namespace robot {
namespace srv {

robot_manage::robot_manage()
{
    init();
}

robot_manage::~robot_manage()
{
    deinit();
}

void robot_manage::init()
{
    bound_thr_ = std::make_shared<std::thread>([&]() {
        bool is_first = true;
        is_loop_ = true;
        while (is_loop_) {
            if (is_first) {
                std::this_thread::sleep_for(std::chrono::seconds(5));
                is_first = false;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            bound_monitor();
        }
    });

    data_thr_ = std::make_shared<std::thread>([&]() {
        is_data_loop_ = true;
        while (is_data_loop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
            if(data_cb_) {
                robot_list_t temp;
                {
                    std::lock_guard<std::mutex> lck(lst_lck_);
                    temp = robot_lst_;
                }
                robot_vector_t robot_list;
                for (auto &item : temp) {
                    robot_list.push_back((robot_data)(*(static_cast<robot_data*>(&(item.second)))));
                }
                data_cb_(robot_list);
            }
        }
    });
}

void robot_manage::deinit()
{
    is_data_loop_ = false;
    if(data_thr_ && data_thr_->joinable()) {
        data_thr_->join();
    }
    data_thr_ = nullptr;

    is_loop_ = false;
    if(bound_thr_ && bound_thr_->joinable()) {
        bound_thr_->join();
    }
    bound_thr_ = nullptr;
}

int robot_manage::add_robot(int id, double x, double y, double angle, int w, int  h, double v, double vw, int type)
{
    int ret = 0;
    double red = 0.0;
    if (pos_monitor(x, y, red, w, h))
    {
        add_robot(id, type);
        set_robot_pos(id, x, y, angle);
        set_robot_bound(id, 0, 0, w, h);
        set_robot_vel(id, v, vw); 
    } else {
        std::cout << "id: " << id << " Illegal positon parameter ! Fail to add robot" << std::endl;
        ret = -1;
    }
    return ret;
}

int robot_manage::add_robot(int id, int type)
{
    auto obj = motion::motion_factory::create_object(id, 
        static_cast<motion::motion_type_t>("", type));
    robot_info info;
    info.id = id;
    info.chassis = obj;
#if 0
    if(obj) {
        obj->on("vel_feedback", [=](const common::any &data) {
            auto vel = common::any_cast<vel_t>(data);
            robot_lst_[id].vel = vel;
        });

        obj->on("pos_feedback", [=](const common::any &data) {
            auto pos = common::any_cast<pos_t>(data);
            robot_lst_[id].pos = pos;
        });
    }
#endif
    {
        std::lock_guard<std::mutex> lck(lst_lck_);
        motion_lst_[id] = obj;
        robot_lst_[id] = info;
    }
#ifdef SHOW_DEBUG
    show_speed();
#endif
    return 0;
}

void robot_manage::show_speed()
{
    std::lock_guard<std::mutex> lck(lst_lck_);
    vel_t vel;
    for (auto it:motion_lst_) {
        it.second->get_speed(vel);
        std::cout << "id =" << it.first << " v = " 
            << vel.v << " " << "w =" << vel.w << std::endl;
    }
}

int robot_manage::remove_robot(int id, int type)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        motion_lst_.erase(cit);
    }
    return 0;
}

void robot_manage::set_robot_pos(int id, double x, double y, double angle)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        motion_ptr_t &obj = cit->second;
        if(obj) {
            obj->set_pos(x, y, angle);
        }
    }
}

void robot_manage::set_robot_bound(int id, int x, int y, int w, int h)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        motion_ptr_t &obj = cit->second;
        if(obj) {
            obj->set_bound(x, y, w, h);
            robot_lst_[id].rect = rect_t(x, y, w, h);
        }
    }
}

void robot_manage::set_robot_vel(int id, double v, double w)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        auto &obj = cit->second;
        if(obj) {
            obj->set_speed(v, w);
        }
    }
}

void robot_manage::bound_monitor()
{
    robot_bound monitor;
    robot_list_t temp;
    {
        std::lock_guard<std::mutex> lck(lst_lck_);
        temp = robot_lst_;
    }
    if (temp.empty())
        return;

    for (auto first = temp.begin(); first != temp.end(); first++) {
        auto second = (++first)--;
        for ( ; second != temp.end(); second++) {
            rect_t &rc1 = first->second.rect;
            rc1.x = first->second.pos.x;
            rc1.y = first->second.pos.y;
            rect_t &rc2 = second->second.rect;
            rc2.x = second->second.pos.x;
            rc2.y = second->second.pos.y;
            if (!monitor.judge_bound(rc1, rc2, 0)) {
#ifdef BOUND_DEBUG
                using namespace std::chrono;
                printf("collosion: %ld\n", duration_cast<milliseconds>(
                    steady_clock().now().time_since_epoch()).count());
#endif
                set_robot_vel(first->first, 0, 0);
                set_robot_vel(second->first, 0, 0);
            }
        }
    }
}
bool robot_manage::pos_monitor(double x, double y, double red, double w, double h)
{
    robot_bound monitor;
    robot_list_t temp;
    {
        std::lock_guard<std::mutex> lck(lst_lck_);
        temp = robot_lst_;
    }
    if (temp.empty()) {
        return true;
    }
    rect_t rc1(x, y, w, h);
    auto flag = temp.begin();
    for ( ; flag != temp.end(); flag++) {
        rect_t &rc2 = flag->second.rect;
        rc2.x = flag->second.pos.x;
        rc2.y = flag->second.pos.y;
        if (!monitor.judge_bound(rc1, rc2, red)) {
            return false;
        }
    }
    return true;
}

void robot_manage::start(int id)
{
    if(id < 0) {
        for(auto &cit : motion_lst_) {
            auto &obj = cit.second;
            if(obj) {
                obj->start();
            }
        }
    } else {
        start_robot(id);
    }
}

void robot_manage::stop(int id)
{
    if(id < 0) {
        for(auto &cit : motion_lst_) {
            auto &obj = cit.second;
            if(obj) {
                obj->stop();
            }
        }
    } else {
        stop_robot(id);
    }
}

void robot_manage::start_robot(int id)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        auto &obj = cit->second;
        if(obj) {
            obj->start();
        }
    }
}

void robot_manage::stop_robot(int id)
{
    auto cit = motion_lst_.find(id);
    if(cit != motion_lst_.end()) {
        auto &obj = cit->second;
        if(obj) {
            obj->stop();
        }
    }
}

} // namespace srv
} // namespace robot
