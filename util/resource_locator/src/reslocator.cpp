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

/** \author Rosen Diankov, Ioan Sucan */

#include <reslocator/reslocator.h>
#include <rosconsole/rosconsole.h>
#include <boost/regex.hpp>


std::string res_locator::resource2path(const std::string &resource)
{
    // check if URL is valid
    boost::cmatch matches;
    boost::regex re("(ros-pkg|ros-param):\\/\\/((\\w+\\.)*(\\w*))\\/([\\w\\d]+\\/{0,1})+");
    
    if (boost::regex_match(resource.c_str(), matches, re) && matches.size() >= 3){
	std::string protocol(matches[1].first, matches[1].second);
	std::string protocol_path(matches[2].first, matches[2].second);
	std::string relpath(matches[2].second,matches[matches.size()-1].second);
	
	if( protocol == std::string("ros-pkg") ) {
	    // find the ROS package
	    FILE* f = popen((std::string("rospack find ") + protocol_path).c_str(),"r");
	    if( f == NULL )
		ROS_ERROR("%s\n", (std::string("failed to launch rospack find ") + protocol_path).c_str());
	    else {
		char basepath[1024];
		fgets(basepath, sizeof(basepath), f);
		pclose(f);
		
		// strip out any new lines or spaces from the end
		int len = strlen(basepath);
		char* p = basepath+len-1;
		while(len-- > 0 && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r'))
		    *p-- = 0;
		return std::string(basepath) + relpath;
	    }
	}
	else if( protocol == std::string("ros-param") )
	{
	    return "";
	}
    }
    else // not a url so copy directly
	return resource;
    
    return "";
}
