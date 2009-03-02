/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/* \author Ioan Sucan */

#include "collision_space/output.h"
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <cstdlib>

static collision_space::msg::OutputHandlerSTD _defaultOutputHandler;
static collision_space::msg::OutputHandler   *OUTPUT_HANDLER = static_cast<collision_space::msg::OutputHandler*>(&_defaultOutputHandler);
static boost::mutex                _lock; // it is likely the outputhandler does some I/O, so we serialize it

void collision_space::msg::noOutputHandler(void)
{
    _lock.lock();
    OUTPUT_HANDLER = NULL;
    _lock.unlock();
}

void collision_space::msg::useOutputHandler(OutputHandler *oh)
{
    _lock.lock();
    OUTPUT_HANDLER = oh;
    _lock.unlock();
}

collision_space::msg::OutputHandler* collision_space::msg::getOutputHandler(void)
{
    return OUTPUT_HANDLER;
}

collision_space::msg::Interface::Interface(void)
{
}

collision_space::msg::Interface::~Interface(void)
{
}

void collision_space::msg::Interface::message(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
	_lock.lock();
	OUTPUT_HANDLER->message(text);
	_lock.unlock();
    }
}

void collision_space::msg::Interface::message(const char *msg, ...) const
{
    va_list ap;
    va_start(ap, msg);
    message(combine(msg, ap));
    va_end(ap);
}

void collision_space::msg::Interface::inform(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
	_lock.lock();
	OUTPUT_HANDLER->inform(text);
	_lock.unlock();
    }
}

void collision_space::msg::Interface::inform(const char *msg, ...) const
{
    va_list ap; 
    va_start(ap, msg);
    inform(combine(msg, ap));
    va_end(ap);
}

void collision_space::msg::Interface::warn(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
	_lock.lock();
	OUTPUT_HANDLER->warn(text);
	_lock.unlock();
    }
}

void collision_space::msg::Interface::warn(const char *msg, ...) const
{
    va_list ap; 
    va_start(ap, msg);
    warn(combine(msg, ap));
    va_end(ap);
}

void collision_space::msg::Interface::error(const std::string &text) const
{
    if (OUTPUT_HANDLER)
    {
	_lock.lock();
	OUTPUT_HANDLER->error(text);
	_lock.unlock();
    }
}

void collision_space::msg::Interface::error(const char *msg, ...) const
{
    va_list ap;
    va_start(ap, msg);
    error(combine(msg, ap));
    va_end(ap);
}
std::string collision_space::msg::Interface::combine(const char *msg, va_list va) const
{
    va_list ap;
    va_copy(ap, va);
    char buf[1024];
    vsnprintf(buf, sizeof(buf), msg, ap);
    va_end(ap);
    return buf;
}

void collision_space::msg::OutputHandlerSTD::error(const std::string &text)
{
    std::cerr << "Error:   " << text << std::endl;
    std::cerr.flush();
}

void collision_space::msg::OutputHandlerSTD::warn(const std::string &text)
{
    std::cerr << "Warning: " << text << std::endl;
    std::cerr.flush();
}

void collision_space::msg::OutputHandlerSTD::inform(const std::string &text)
{ 
    std::cout << "Info:    " << text << std::endl;
    std::cout.flush();
}

void collision_space::msg::OutputHandlerSTD::message(const std::string &text)
{
    std::cout << text;
    std::cout.flush();
}
