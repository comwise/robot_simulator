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
#ifndef __COMWISE_CORE__MODULE_TYPE__H__
#define __COMWISE_CORE__MODULE_TYPE__H__

#include <cstdint>

namespace core {

template <typename Id, typename Type = uint32_t>
class module_type
{
    enum type_const_t {
        TYPE_INIT_MASK = 0x00000000,
        TYPE_HB_MASK = 0xffff0000,
        TYPE_LB_MASK = 0x0000ffff,
        TYPE_MOVE_BIT = 16
    };
public:
    using module_id_t = Id;
    using module_type_t = Type;

public:
    explicit module_type(module_id_t id, module_type_t type) : id_(id) { set_major_type(type); }
    virtual ~module_type() { }

    //! module type
    virtual void set_type(module_type_t type) { set_major_type(type); }
    virtual module_type_t get_type() const { return get_major_type(); }

    //! module id
    virtual void set_id(const module_id_t &id) { id_ = id; }
    virtual module_id_t get_id() const { return id_; }

    //! set/get major type 
    void set_major_type(module_type_t major) { type_ = (type_&TYPE_LB_MASK)|((major<<16)&TYPE_HB_MASK); }
    module_type_t get_major_type() const { return (type_>>16)&TYPE_LB_MASK; }

    //!  set/get minor type
    void set_minor_type(module_type_t minor) { type_ = (type_&TYPE_HB_MASK)|(minor&TYPE_LB_MASK); }
    module_type_t get_minor_type() const { return type_&TYPE_LB_MASK; }

protected:
    module_type_t type_ {TYPE_INIT_MASK};
    module_id_t id_;
};

} // namespace core
 
#endif // __COMWISE_CORE__MODULE_TYPE__H__
