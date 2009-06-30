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

#ifndef PLANNING_MODELS_OUTPUT_INTERFACE_
#define PLANNING_MODELS_OUTPUT_INTERFACE_

#include <string>
#include <cstdarg>

/** \brief Main namespace */
namespace planning_models
{
    
    /** \brief Message namespace. Code in this namespace is used to
	forward information/warnings/errors to the user by using
	output handlers. The library uses an Interface to send such
	messages. The Interface in turn uses an instance of an
	OutputHandler to publish this data. By default, data goes to
	STDOUT and STDERR */
    namespace msg
    {
	
	/** \brief The piece of code that desires interaction with an
	    output handler should use an instance of this
	    class. Multiple instances of this class will use the same
	    OutputHandler */
	class Interface
	{
	public: 
	    
	    Interface(void);	
	    virtual ~Interface(void);
	    
	    void inform(const std::string &text) const;
	    void warn(const std::string &text) const;
	    void error(const std::string &text) const;
	    void message(const std::string &text) const;

	    void inform(const char *msg, ...) const;
	    void warn(const char *msg, ...) const;
	    void error(const char *msg, ...) const;
	    void message(const char *msg, ...) const;

	protected:
	    
	    std::string combine(const char *msg, va_list ap) const;
	    
	};
	
	/** \brief Generic class to handle output from a piece of
	    code. An instance of an output handler is used by the
	    Interface instances */
	class OutputHandler
	{
	public:
	    
	    OutputHandler(void)
	    {
	    }
	    
	    virtual ~OutputHandler(void)
	    {
	    }

	    /** \brief Print an error message: "Error: ...." */
	    virtual void error(const std::string &text) = 0;

	    /** \brief Print an warning message: "Warning: ...." */
	    virtual void warn(const std::string &text) = 0;

	    /** \brief Print some information: "Information: ...." */
	    virtual void inform(const std::string &text) = 0;
	    
	    /** \brief Print a simple message */
	    virtual void message(const std::string &text) = 0;
	};
	
	/** \brief Default output handler: prints to STDOUT and STDERR */
	class OutputHandlerSTD : public OutputHandler
	{
	public:
	    
	    OutputHandlerSTD(void) : OutputHandler()
	    {
	    }
	    
	    /** \brief Print an error message: "Error: ...." */
	    virtual void error(const std::string &text);
	    
	    /** \brief Print an warning message: "Warning: ...." */
	    virtual void warn(const std::string &text);

	    /** \brief Print some information: "Information: ...." */
	    virtual void inform(const std::string &text);
	    
	    /** \brief Print a simple message */
	    virtual void message(const std::string &text);
	    
	    
	};

	/** \brief Disable all output */
	void noOutputHandler(void);

	/** \brief Direct output to this handler */
	void useOutputHandler(OutputHandler *oh);

	/** \brief Get the address of the active handler */
	OutputHandler* getOutputHandler(void);
    }
    
}

#endif
