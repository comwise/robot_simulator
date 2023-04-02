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
#ifndef __COMWISE_CORE__MODULE__H__
#define __COMWISE_CORE__MODULE__H__

#include <cstdint>
#include <memory>
#include <string>
#include "error.h"
#include "state.h"
#include "type.h"
#include "event.h"

namespace core {

template <typename Id, typename Param>
class module_core : public module_type<Id>, public state, public error_code<int>
{
public:
    module_core(const Id &id, int type = 0) : module_type<Id>(id, type) { }
    virtual ~module_core() { }

    //>! init/deinit module
    virtual int init(const Param &param) { return -1; }
    virtual int deinit() { return -1; }

    //>! start/stop module
    virtual int start() { return -1; }
    virtual int stop() { return -1; }
};

template <typename Id, typename Param, typename Request, typename Response>
class module : public module_core<Id, Param>
{
public:
    module(const Id &id, int type = 0) : module_core<Id, Param>(id, type) { }
    virtual ~module() { }

    //>! init/deinit module
    virtual int init(const Param &param) { return -1; }
    virtual int deinit() { return -1; }

    //>! start/stop module
    virtual int start() { return -1; }
    virtual int stop() { return -1; }

    //>! read/write module
    virtual int read(const Request &req, Response res) { return -1; }
    virtual int write(const Request &req, Response res) { return -1; }
};

template <typename Id, typename Param, typename Request, typename Response>
class module_event : public module<Id, Param, Request, Response>, public event
{
public:
    module_event(const Id &id, int type = 0) 
        : module<Id, Param, Request, Response>(id, type) { }
    virtual ~module_event() { }
};

} // namespace core
 
#endif // __COMWISE_CORE__MODULE__H__
