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


/** \Author Ioan Sucan */

#ifndef UTIL_PROFILING_UTILS_PROFILER_
#define UTIL_PROFILING_UTILS_PROFILER_

#include <map>
#include <string>
#include <ros/time.h>
#include <iostream>

namespace profiling_utils
{

    class Profiler
    {
    public:
	
	static Profiler* Instance(void);

	Profiler(void)
	{
	    m_running = false;
	}
	
	virtual ~Profiler(void)
	{
	}
	
	static void Start(void)
	{
	    Instance()->start();
	}
	
	static void Stop(void)
	{
	    Instance()->stop();
	}
	
	void start(void);
	void stop(void);
	
	static void Event(const std::string& name, const unsigned int times = 1)
	{
	    Instance()->event(name, times);
	}
	
	void event(const std::string &name, const unsigned int times = 1);
	
	static void Begin(const std::string &name)
	{
	    Instance()->begin(name);
	}
	
	static void End(const std::string &name)
	{
	    Instance()->end(name);
	}
	
	void begin(const std::string &name);
	void end(const std::string &name);	
	
	static void Status(std::ostream &out = std::cout, bool merge = true)
	{
	    Instance()->status(out, merge);
	}
	
	void status(std::ostream &out = std::cout, bool merge = true);
	
    protected:
	
	struct TimeInfo
	{
	    ros::Duration total;
	    ros::Time     start;
	    
	    void set(void)
	    {
		start = ros::Time::now();
	    }
	    
	    void update(void)
	    {
		total = total + (ros::Time::now() - start);
	    }
	};
	
	struct PerThread
	{
	    std::map<std::string, unsigned int> events;
	    std::map<std::string, TimeInfo>     time;
	};
	
	void printThreadInfo(std::ostream &out, const PerThread &data) const;	
	    
	std::map<unsigned long int, PerThread> m_data;
	TimeInfo                               m_tinfo;
	bool                                   m_running;
	
    };
    
}

#endif
