/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>

namespace rog_map {
    class TimeConsuming {

    public:
        TimeConsuming();

        TimeConsuming(std::string msg, int repeat_time) {
            repeat_time_ = repeat_time;
            msg_ = msg;
            tc_start = std::chrono::high_resolution_clock::now();
            has_shown = false;
            print_ = true;
        }

        TimeConsuming(std::string msg, bool print_log) {
            msg_ = msg;
            repeat_time_ = 1;
            print_ = print_log;
            tc_start = std::chrono::high_resolution_clock::now();
            has_shown = false;
        }

        ~TimeConsuming() {
            if (!has_shown && enable_ && print_) {
                tc_end = std::chrono::high_resolution_clock::now();
                double dt = std::chrono::duration_cast<std::chrono::duration<double >>(tc_end - tc_start).count();
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
                } else
                    printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
            }
        }

        void set_enbale(bool enable) {
            enable_ = enable;
        }

        void start() {
            tc_start = std::chrono::high_resolution_clock::now();
        }

        double stop() {
            if (!enable_) { return 0; }
            has_shown = true;
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double >>(tc_end - tc_start).count();
            if (!print_) {
                return dt;
            }
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
            } else
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
            return dt;
        }

    private:
        std::chrono::high_resolution_clock::time_point tc_start, tc_end;
        std::string msg_;
        int repeat_time_{1};
        bool has_shown = false;
        bool enable_{true};
        bool print_{true};
    };
}

