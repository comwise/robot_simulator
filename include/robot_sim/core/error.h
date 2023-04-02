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
#ifndef __COMWISE_CORE__ERROR__H__
#define __COMWISE_CORE__ERROR__H__

#include <cstdint>
#include <string>
#include <vector>
#include <tuple>

namespace core {

class error_base
{
public:
    enum level_t {
        kLevelOK = 0,   // all ok
        kLevelTimeout,  // sometime disconnect, can resume it
        kLevelWarn,     // keep report warn periodically or alway, can ignore it
        kLevelError,    // keep report error, need manual operation
        kLevelFatal,    // can't manual, need contact developer
    };
public:
    virtual ~error_base() { }

    virtual void set_code(int code, const std::string &detail = "") { 
        code_ = code; detail_ = detail; }
    virtual int get_code() const { return code_; }

    virtual void set_level(const level_t &level) { detail_ = level; }
    virtual level_t get_level() { return level_; }

    virtual void set_detail(const std::string &detail) { detail_ = detail; }
    virtual std::string get_detail() { return detail_; }

    virtual void set_suggest(const std::string &sd) { suggest_ = sd; }
    virtual std::string get_suggest() { return suggest_; }

    virtual void set_error(int code, const level_t &level, const std::string &detail = "") {
        code_ = code; level_ = level; detail_ = detail; }
    virtual std::tuple<int, level_t, std::string> get_error() {
        return std::make_tuple(code_, level_, detail_); }

    virtual void clear() { code_ = 0; level_ = level_; detail_ = ""; suggest_ = ""; }

protected:
    int code_{0};
    level_t level_{kLevelOK};
    std::string detail_;
    std::string suggest_;
};

template <typename T>
class error_code : public error_base
{
public:
    enum {
      kCodeOK           = 0x0,      // normal
      kCodeSendError    = 0xFFFC,   // send data error
      kCodeInitError    = 0xFFFD,   // init error
      kCodeIllegalParam = 0xFFFE,   // illegal param
      kCodeDisconnected = 0xFFFF    // disconnected alway
    };

public:
    error_code() { }
    virtual ~error_code() { }

    virtual void set_codes(const T &code) { codes_.emplace_back(code); }
    virtual void set_codes(const std::vector<T> &codes) { codes_ = codes; }
    virtual std::vector<T> get_codes() { return codes_; }

    virtual void clear() { codes_.clear(); error_base::clear(); }

private:
    std::vector<T> codes_;
};

} // namespace core

#endif // __COMWISE_CORE__ERROR__H__
