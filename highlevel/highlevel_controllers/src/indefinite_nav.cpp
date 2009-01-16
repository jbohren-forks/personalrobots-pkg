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


#include <IndefiniteNav.hh>
#include <math.h>

using namespace std;


namespace ros {
  namespace highlevel_controllers {

    IndefiniteNav::IndefiniteNav() : ros::Node("indefinite_nav_node")
    {
      addGoal (24.6, 30, 0.66);
      addGoal (17.5, 31.3, -1.02);
      addGoal (30.45, 37.6, 1.11);
      addGoal (33.75, 31.4, -1.53);
      addGoal (28.85, 24.5, -0.63);
      addGoal (25.7, 22.5, -1.74);
      addGoal (17.6, 14.2, 1.78);
      addGoal (33.1, 12.1, 1.64);
      first = true;
    }



    void IndefiniteNav::run() 
    {
      list<Planner2DGoal>::iterator i;

      cout << "In IndefiniteNav.run\n";
      advertise<Planner2DGoal>("goal", 10);
      subscribe<Planner2DState>("state", state_msg, &IndefiniteNav::stateCallback, 10);
	
      sleep(4);
      while (true) {

	for (i=msgs.begin(); i!=msgs.end(); i++) {
	  
	  if (first || fabs(state_msg.pos.x - state_msg.goal.x) + fabs(state_msg.pos.y - state_msg.goal.y) < 1) {
	    printf ("Dispatching goal %f, %f, %f", i->goal.x, i->goal.y, i->goal.th);
	    publish ("goal", *i);
	    sleep (2); // Should avoid need for lock
	    first = false;
	  }
	}
      }
    }





    void IndefiniteNav::addGoal (double x, double y, double th) {
	
      Planner2DGoal g;
      g.enable = 1;
      g.goal.x = x;
      g.goal.y = y;
      g.goal.th = th;
      msgs.push_back(g);

    }



    void IndefiniteNav::stateCallback ()
    {
    }



  }
}

















int main (int argc, char** argv)
{
  ros::init(argc,argv);
  ros::highlevel_controllers::IndefiniteNav node;
  node.run();
  ros::fini();
  return(0);
}
  
