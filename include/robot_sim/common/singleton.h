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
*********************************************************************/
#ifndef __COMMON__SINGLETON__H__
#define __COMMON__SINGLETON__H__

#include <atomic>
#include <memory>
#include <mutex>

namespace common {


template <class T>
class singleton
{
  public:
    static T *instance() {
        // with atomic mutex DoubleCheckNull
        T *tmp = instance_.load(std::memory_order::memory_order_acquire);
        if (!tmp) {
            std::lock_guard<std::recursive_mutex> guard(lock_);
            tmp = instance_.load(std::memory_order::memory_order_relaxed);
            if (!tmp) {
                tmp = new T;
                associated_ = 0;
                instance_.store(tmp, std::memory_order::memory_order_release);
            }
        }
        return tmp;
    }

    static void associate(T *ptr) {
        // with atomic mutex DoubleCheckNull
        T *tmp = instance_.load(std::memory_order::memory_order_acquire);
        if (!tmp) {
            std::lock_guard<std::recursive_mutex> guard(lock_);
            tmp = instance_.load(std::memory_order::memory_order_relaxed);
            if (!tmp) {
                tmp = ptr;
                associated_ = 1;
                instance_.store(tmp, std::memory_order::memory_order_release);
            }
        }
    }

    static void release() {
        T *tmp = instance_.load(std::memory_order::memory_order_acquire);
        if (tmp && !associated_) {
            std::lock_guard<std::recursive_mutex> guard(lock_);
            tmp = instance_.load(std::memory_order::memory_order_relaxed);
            if (tmp && !associated_) {
                delete tmp;
                instance_.store(nullptr, std::memory_order::memory_order_release);
            }
        }
    }

  protected:
    singleton() { }
    ~singleton() { }

    singleton(const singleton &) = delete;
    singleton(singleton &&) = delete;
    singleton &operator=(const singleton &) = delete;
    singleton &operator=(singleton &&) = delete;

  private:
    static std::atomic<T *> instance_;
    static std::recursive_mutex lock_;
    static int associated_;
};

template <class T>
std::atomic<T *> singleton<T>::instance_ {nullptr};

template <class T>
std::recursive_mutex singleton<T>::lock_;

template <class T>
int singleton<T>::associated_ {0};

} //namespace common

#endif  // __COMMON__SINGLETON__H__
