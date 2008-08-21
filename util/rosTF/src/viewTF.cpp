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

#include <ros/node.h>
#include <ros/time.h>
#include <rosTF/rosTF.h>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <termios.h>

#include <namelookup/NumberToName.h>

class ViewTF : public ros::node
{
public:

    ViewTF(void) : ros::node("viewTF"),
		   m_tf(*this, true, 1 * 1000000000ULL, 1000000000ULL)
    {
	active = true;
    }
    
    ~ViewTF(void)
    {
    }
    
    void save(const char *filename)
    {
	std::map<int, int> m = m_tf.getFramesMap();
	std::map<int, std::string> names;
	
	for (std::map<int, int>::iterator it = m.begin() ; it != m.end() ; ++it)
	{
	    namelookup::NumberToName::request req;
	    namelookup::NumberToName::response res;
	    if (names.find(it->first) == names.end())
	    {
		req.number = it->first;
		if (ros::service::call("numberToName", req, res))
		    names[it->first] = res.name;
		else
		    names[it->first] = "error";
	    }
	    if (names.find(it->second) == names.end())
	    {
		req.number = it->second;
		if (ros::service::call("numberToName", req, res))
		    names[it->second] = res.name;
		else
		    names[it->second] = "error";
	    }	    
	}
	
	std::ofstream out(filename);
	out << "digraph TF {" << std::endl;
	
	for (std::map<int, int>::iterator it = m.begin() ; it != m.end() ; ++it)
	{
	    if (it->second == 0)
		continue;
	    out << "\"" << it->first << ": " << names[it->first] << "\" -> \"" << it->second << ": " << names[it->second] << "\"" << std::endl;
	}	
	
	out << "};" << std::endl;	
	out.close();
	
	printf("Saved '%s'\n", filename);
    }
    
    bool active;    

private:
    
    rosTFClient m_tf; 
        
};

int main(int argc, char **argv)
{
    // get the console in raw mode
    int kfd = 0;
    struct termios cooked, raw;
    
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO | ISIG);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    ros::init(argc, argv);
    ViewTF viewer;
    
    if (argc >= 2 && strcmp(argv[1], "--dump") == 0)
    {
	std::cout<< "Taking data for 5 seconds . . ." << std::endl;
	sleep(5);
	viewer.save(argc > 2 ? argv[2] : "viewTF.dot");
    }
    else
        while (viewer.ok() && viewer.active)
	{
	    char command;	
	    if (read(kfd, &command, 1) < 0)
	    {
		perror("read():");
		exit(-1);
	    }
	    
	    switch (command)
	    {
	    case 32:
		viewer.save("viewTF.dot");
		break;
	    case 3:
	    case 28:
		viewer.active = false;
		break;	    
	    default:
		break;
	    }	
	}
	
    viewer.shutdown();
    
    tcsetattr(kfd, TCSANOW, &cooked);
    
    return 0;    
}
