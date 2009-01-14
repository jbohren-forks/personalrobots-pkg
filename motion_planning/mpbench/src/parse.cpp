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

#include "parse.h"
#include <sfl/util/strutil.hpp>

extern "C" {
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <err.h>
}

using namespace std;

extern "C" {
  
  static void start_element_handler(void * user_data, const XML_Char * name,
				    const XML_Char ** atts)
    throw(std::runtime_error);
  
  static void end_element_handler(void * user_data, const XML_Char * name)
    throw(std::runtime_error);
  
  static void character_data_handler(void * user_data, const XML_Char * s,
				     int len);
  
}

namespace mpbench {
  
  StringBuffer::
  StringBuffer()
    : m_lastchar(' ')
  {
  }
  
  
  void StringBuffer::
  Append(const XML_Char * s, int len)
  {
    for (int ii(0); ii < len; ++ii) {
      if (isspace(s[ii])) {
	if (' ' != m_lastchar) {
	  m_string += ' ';
	  m_lastchar = ' ';
	}
	// else skip
      }
      else {
	m_string += s[ii];
	m_lastchar = s[ii];
      }
    }
  }
  
  
  string const & StringBuffer::
  GetString() const
  {
    return m_string;
  }
  
  
  bool StringBuffer::
  Empty() const
  {
    return m_string.empty();
  }
  
  
  File::
  File(char const * path, int flags) throw (std::runtime_error)
    : fd(open(path, flags))
  {
    if (fd < 0)
      throw runtime_error(string("open(") + path + "): " + strerror(errno));
  }
  
  
  File::
  File(char const * path, int flags, mode_t mode) throw (std::runtime_error)
    : fd(open(path, flags, mode))
  {
    if (fd < 0)
      throw runtime_error(string("open(") + path + "): " + strerror(errno));
  }
  
  
  File::
  ~File()
  {
    if (fd >= 0)
      close(fd);
  }
  
  
  SetupParser::
  SetupParser()
    : parser(0),
      filename("/dev/null"),
      bufsize(128),
      setup(0),
      progress_os(0),
      tmp_task("", task::goalspec(0, 0, 0, 0, 0))
  {
  }
  
  
  SetupParser::
  ~SetupParser()
  {
    if (parser)
      XML_ParserFree(parser);
  }
  
  
  void SetupParser::
  Parse(std::string xml_filename, Setup * setup, std::ostream * progress_os)
    throw(std::runtime_error)
  {
    if (0 == parser)
      parser = XML_ParserCreate(NULL);
    XML_SetElementHandler(parser, start_element_handler, end_element_handler);
    XML_SetCharacterDataHandler(parser, character_data_handler);
    XML_SetUserData(parser, this);
    
    filename = xml_filename;
    this->setup = setup;
    this->progress_os = progress_os;
    file.reset(new File(filename.c_str(), O_RDONLY));
    
    // clear tag_stack
    while ( ! tag_stack.empty())
      tag_stack.pop();
    
    while (true) {
      void * buf(XML_GetBuffer(parser, bufsize));
      if (NULL == buf)
	throw runtime_error("mpbench::SetupParser::Parse(): XML_GetBuffer() failed");
      ssize_t const bytes_read(read(file->fd, buf, bufsize));
      if (bytes_read < 0) {
	ostringstream os;
	os << "mpbench::SetupParser::Parse(): " << filename << ": read(): " << strerror(errno);
	throw runtime_error(os.str());
      }
      if ( ! XML_ParseBuffer(parser, bytes_read, bytes_read == 0)) {
	ostringstream os;
	os << "mpbench::SetupParser::Parse(): " << filename
	   << ": " << XML_GetCurrentLineNumber(parser)
	   << ": parse error: " << XML_ErrorString(XML_GetErrorCode(parser));
	throw runtime_error(os.str());
      }
      if (bytes_read == 0)
	break;
    }
  }
  
  
  tag_t getTag(std::string const & tag_name) throw(std::runtime_error)
  {
    typedef map<string, tag_t> foo_t;
    static foo_t foo;
    if (foo.empty()) {
      foo.insert(make_pair("setup", SETUP));
      foo.insert(make_pair("map", MAP));
      foo.insert(make_pair("init", INIT));
      foo.insert(make_pair("change", CHANGE));
      foo.insert(make_pair("addline", ADDLINE));
      foo.insert(make_pair("rmline", RMLINE));
      foo.insert(make_pair("task", TASK));
      foo.insert(make_pair("description", DESCRIPTION));
      foo.insert(make_pair("goal", GOAL));
      foo.insert(make_pair("start", START));
    }
    foo_t::const_iterator ifoo(foo.find(tag_name));
    if (foo.end() == ifoo)
      throw runtime_error("mpbench::getTag(): unknown tag <" + tag_name + ">");
    return ifoo->second;
  }
  
}


using namespace mpbench;

static void throwme(SetupParser * sp, string const & complaint) throw(runtime_error)
{
  ostringstream os;
  os << sp->filename.c_str() << ":" << XML_GetCurrentLineNumber(sp->parser) << ": " << complaint;
  throw runtime_error(os.str());
}


void start_element_handler(void * user_data,
			   const XML_Char * name,
			   const XML_Char ** atts)
    throw(std::runtime_error)
{
  mpbench::SetupParser * sp(reinterpret_cast<mpbench::SetupParser *>(user_data));

  mpbench::tag_t prevtag;
  if (sp->tag_stack.empty())
    prevtag = NONE;
  else
    prevtag = sp->tag_stack.top();
  
  mpbench::tag_t tag;
  try {
    tag = getTag(name);
  }
  catch (runtime_error const & ee) {
    throwme(sp, ee.what());
  }
  
  switch (tag) {
    
  case ADDLINE:
  case RMLINE:
    if ((INIT != prevtag) && (CHANGE != prevtag))
      throwme(sp, "<" + string(name) + "> only allowed inside <init> or <change>");
    sp->buffer.reset(new mpbench::StringBuffer());
    break;
    
  case DESCRIPTION:
  case GOAL:
  case START:
    if (TASK != prevtag)
      throwme(sp, "<" + string(name) + "> only allowed inside <task>");
    sp->buffer.reset(new mpbench::StringBuffer());
    break;
    
  case TASK:
    sp->tmp_task.description = "none";
    sp->tmp_task.start.clear();
    break;
    
  default:
    break;
  }
  
  sp->tag_stack.push(tag);
}


void end_element_handler(void * user_data,
			 const XML_Char * name)
    throw(std::runtime_error)
{
  mpbench::SetupParser * sp(reinterpret_cast<mpbench::SetupParser *>(user_data));
  
  mpbench::tag_t tag(NONE);
  try {
    tag = getTag(name);
  }
  catch (runtime_error const & ee) {
    throwme(sp, ee.what());
  }
  
  switch (tag) {
    
  case ADDLINE:
    if ( ! sp->buffer)
      throwme(sp, "BUG: no string buffer");
    else {
      istringstream is(sp->buffer->GetString());
      double x0, y0, x1, y1;
      is >> x0 >> y0 >> x1 >> y1;
      if ( ! is)
	throwme(sp, "could not read x0 y0 x1 y1 from \"" + sp->buffer->GetString() + "\"");
      sp->setup->drawLine(x0, y0, x1, y1, sp->progress_os);
    }
    break;
    
  case RMLINE:
    warnx("WIP: ignoring <rmline>");
    break;
    
  case DESCRIPTION:
    if ( ! sp->buffer)
      throwme(sp, "BUG: no string buffer");
    sp->tmp_task.description = sp->buffer->GetString();
    break;
    
  case GOAL:
    if ( ! sp->buffer)
      throwme(sp, "BUG: no string buffer");
    else {
      istringstream is(sp->buffer->GetString());
      is >> sp->tmp_task.goal.px
	 >> sp->tmp_task.goal.py
	 >> sp->tmp_task.goal.pth
	 >> sp->tmp_task.goal.tol_xy
	 >> sp->tmp_task.goal.tol_th;
      if ( ! is)
	throwme(sp, "could not read px py pth tol_xy tol_th from \"" + sp->buffer->GetString() + "\"");
    }
    break;
    
  case START:
    if ( ! sp->buffer)
      throwme(sp, "BUG: no string buffer");
    else {
      istringstream is(sp->buffer->GetString());
      string from_scratch_str;
      double px, py, pth;
      is >> from_scratch_str >> px >> py >> pth;
      if ( ! is)
	throwme(sp, "could not read from_scratch px py pth from \"" + sp->buffer->GetString() + "\"");
      bool from_scratch;
      if ( ! sfl::string_to(from_scratch_str, from_scratch))
	throwme(sp, "could not convert \"" + sp->buffer->GetString() + "\" to boolean");
      sp->tmp_task.start.push_back(task::startspec(from_scratch, px, py, pth));
    }
    break;
    
  case TASK:
    sp->setup->addTask(sp->tmp_task);
    break;
    
  default:
    break;
  }
  
  sp->buffer.reset();
  sp->tag_stack.pop();
}


void character_data_handler(void * user_data,
			    const XML_Char * s,
			    int len)
{
  mpbench::SetupParser * sp(reinterpret_cast<mpbench::SetupParser *>(user_data));
  if (sp->buffer)
    sp->buffer->Append(s, len);
}
