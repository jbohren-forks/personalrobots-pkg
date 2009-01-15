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

/** \file parse.h Some utilities for parsing XML files using expat. */

#ifndef MPBENCH_PARSE_H
#define MPBENCH_PARSE_H

// entire file
#ifdef MPBENCH_HAVE_EXPAT

#include <mpbench/setup.h>
#include <expat.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <stdexcept>
#include <stack>

namespace mpbench {
  
  class StringBuffer
  {
  public:
    StringBuffer();
    
    void Append(const XML_Char * s, int len);
    std::string const & GetString() const;
    bool Empty() const;
    
  private:
    std::string m_string;
    char m_lastchar;
  };
  
  
  class File
  {
  public:
    File(char const * path, int flags) throw (std::runtime_error);
    File(char const * path, int flags, mode_t mode)
      throw (std::runtime_error);
    
    ~File();
    
    int fd;
  };
  
  
  typedef enum {
    SETUP,
    MAP,
    INIT,
    CHANGE,
    ADDLINE,
    RMLINE,
    TASK,
    DESCRIPTION,
    GOAL,
    START,
    NONE
  } tag_t;
  
  tag_t getTag(std::string const & tag_name) throw(std::runtime_error);
  
  typedef std::stack<tag_t> tag_stack_t;
  
  class Setup;
  
  class SetupParser
  {
  public:
    SetupParser();
    virtual ~SetupParser();
    
    void Parse(std::string xml_filename, Setup * setup, std::ostream * progress_os)
      throw(std::runtime_error);
    
    // Everything is public for easy access from C callback functions.
    
    XML_ParserStruct * parser;
    boost::shared_ptr<StringBuffer> buffer;
    tag_stack_t tag_stack;
    std::string filename;
    boost::shared_ptr<File> file;
    int bufsize;
    Setup * setup;
    std::ostream * progress_os;
    task::setup tmp_task;
  };
  
}

#endif // MPBENCH_HAVE_EXPAT

#endif // MPBENCH_PARSE_H
