//
// Created by yunfan on 2021/3/19.
// Version: 1.0.0
//


#ifndef SCROPE_TIMER_HPP//SRC_POLY_VISUAL_UTILS_HPP
#define SCROPE_TIMER_HPP

#include <chrono>
#include "fmt/color.h"
#include "cstring"

using namespace std;

class TimeConsuming {
public:
    TimeConsuming();

    TimeConsuming(string msg, int repeat_time) {
        repeat_time_ = repeat_time;
        msg_ = msg;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = false;
    }

    TimeConsuming(string msg, bool enable = false) {
        msg_ = msg;
        repeat_time_ = 1;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = !enable;
    }


    ~TimeConsuming() {
        if (!has_shown && enable_) {
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
            double t_us = (double) dt * 1e6 / repeat_time_;
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            }else
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        }
    }


    void set_enbale(bool enable){
        enable_ = enable;
    }

    void start() {
        tc_start = std::chrono::high_resolution_clock::now();
    }

    double stop() {
        if(!enable_){return -1;}
        tc_end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
        double t_us = (double) dt * 1e6 / repeat_time_;
        if(!has_shown){
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            }else
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        }
        has_shown = true;

        return dt;
    }

private:
    std::chrono::high_resolution_clock::time_point tc_start, tc_end;
    string msg_;
    int repeat_time_;
    bool has_shown = false;
    bool enable_{true};
};

#endif //SRC_POLY_VISUAL_UTILS_HPP
