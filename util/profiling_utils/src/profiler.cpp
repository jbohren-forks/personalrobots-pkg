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

#include <profiling_utils/profiler.h>
#include <pthread.h>
#include <vector>
#include <algorithm>

static profiling_utils::Profiler defaultProfiler;

static pthread_mutex_t MUTEX = PTHREAD_MUTEX_INITIALIZER;       
#define LOCK   { pthread_mutex_lock(&MUTEX);   }
#define UNLOCK { pthread_mutex_unlock(&MUTEX); }

static inline unsigned int long self(void)
{
    return (unsigned long int)pthread_self();
}

profiling_utils::Profiler* profiling_utils::Profiler::Instance(void)
{
    return &defaultProfiler;
}

void profiling_utils::Profiler::start(void)
{  
    LOCK;
    if (!m_running)
    {
	m_tinfo.set();
	m_running = true;
    }	    
    UNLOCK;
}

void profiling_utils::Profiler::stop(void)
{
    LOCK;    
    if (m_running)
    {
	m_tinfo.update();
	m_running = false;
    }
    UNLOCK;    
}

void profiling_utils::Profiler::event(const std::string &name, const unsigned int times)
{
    LOCK;
    m_data[self()].events[name] += times;
    UNLOCK;    
}

void profiling_utils::Profiler::begin(const std::string &name)
{
    LOCK;
    m_data[self()].time[name].set();
    UNLOCK; 
}

void profiling_utils::Profiler::end(const std::string &name)
{
    LOCK;
    m_data[self()].time[name].update();
    UNLOCK; 
}

void profiling_utils::Profiler::status(std::ostream &out, bool merge)
{
    stop();
    
    out << std::endl;
    out << " *** Profiling statistics. Total counted time : " << m_tinfo.total.to_double() << " seconds" << std::endl;
    
    if (merge)
    {
	PerThread combined;
	for (std::map<unsigned long int, PerThread>::const_iterator it = m_data.begin() ; it != m_data.end() ; ++it)
	{
	    for (std::map<std::string, unsigned int>::const_iterator iev = it->second.events.begin() ; iev != it->second.events.end(); ++iev)
		combined.events[iev->first] += iev->second;
	    for (std::map<std::string, TimeInfo>::const_iterator itm = it->second.time.begin() ; itm != it->second.time.end(); ++itm)
		combined.time[itm->first].total = combined.time[itm->first].total + itm->second.total;
	}
	printThreadInfo(out, combined);
    }
    else
	for (std::map<unsigned long int, PerThread>::const_iterator it = m_data.begin() ; it != m_data.end() ; ++it)
	{
	    out << "Thread " << it->first << ":" << std::endl;
	    printThreadInfo(out, it->second);
	}
}

namespace profiling_utils
{
    
    struct dEnv
    {
	std::string  name;	
	unsigned int value;	
    };
    
    struct SortEnvByValue
    {
	bool operator()(const dEnv &a, const dEnv &b) const
	{
	    return a.value > b.value;
	}
    };
    
    struct dTm
    {
	std::string  name;	
	double       value;	
    };
    
    struct SortTmByValue
    {
	bool operator()(const dTm &a, const dTm &b) const
	{
	    return a.value > b.value;
	}
    };
}

void profiling_utils::Profiler::printThreadInfo(std::ostream &out, const PerThread &data) const
{
    double total = m_tinfo.total.to_double();
    
    std::vector<dEnv> events;
    
    for (std::map<std::string, unsigned int>::const_iterator iev = data.events.begin() ; iev != data.events.end() ; ++iev)
    {
	dEnv next = {iev->first, iev->second};
	events.push_back(next);
    }
    
    std::sort(events.begin(), events.end(), SortEnvByValue());
    
    for (unsigned int i = 0 ; i < events.size() ; ++i)
	out << events[i].name << ": " << events[i].value << std::endl;
    
    std::vector<dTm> time;
    
    for (std::map<std::string, TimeInfo>::const_iterator itm = data.time.begin() ; itm != data.time.end() ; ++itm)
    {
	dTm next = {itm->first, itm->second.total.to_double()};
	time.push_back(next);
    }
    
    std::sort(time.begin(), time.end(), SortTmByValue());
    
    for (unsigned int i = 0 ; i < time.size() ; ++i)
	out << time[i].name << ": " << time[i].value << " (" << (100.0 * time[i].value/total) << " %)" << std::endl;
    
    out << std::endl;    
}

