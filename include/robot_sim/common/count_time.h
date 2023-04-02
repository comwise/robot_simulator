/*********************************************************************
*
*  GNU GENERAL PUBLIC LICENSE
*
*  Copyright (c) 2023, comwise li.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: comwise comwise@hotmail.com
* web:  https://github.com/comwise/robot_simulator
* @file count_time.hpp
* @code usage:
*        #include <count_time.hpp>
*        count_time time;
*        time.begin();
*        coding .....
*        time.end();
*        printf("use time = %f\n",getTime());
*********************************************************************/

#ifndef __COUNT_TIME__H__
#define __COUNT_TIME__H__

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <chrono>

namespace common {

class count_time
{
public:
    count_time() {
        begin();
    }

    void begin() {
        begin_time_ = std::chrono::steady_clock::now();
    }

    void end() {
        end_time_ = std::chrono::steady_clock::now();
    }

    float get_time() const {
        return std::chrono::duration_cast<
            std::chrono::milliseconds>(end_time_ - begin_time_).count();
    }

    void print() {
        printf("@time count = %fms\n", get_time());
    }

private:
    std::chrono::steady_clock::time_point begin_time_;
    std::chrono::steady_clock::time_point end_time_;
};

} // namespace common

#endif //__COUNT_TIME__H__
