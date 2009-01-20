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

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b move_base is...
 *
 * <hr>
 *
 *  @section usage Usage
 *  @verbatim
 *  $ move_base
 *  @endverbatim
 *
 * <hr>
 *
 * @section topic ROS topics
 *
 * Subscribes to (name/type):
 * - @b 
 *
 * Publishes to (name / type):
 * - @b 
 *
 *  <hr>
 *
 * @section parameters ROS parameters
 *
 * - None
 **/

#include <highlevel_controllers/move_base.hh>
#include <navfn.h>

namespace ros {
  namespace highlevel_controllers {

    /**
     * @brief Specialization for the SBPL planner
     */
    class MoveBaseNAVFN: public MoveBase {
    public:
      MoveBaseNAVFN();

    private:

      /**
       * @brief Builds a plan from current state to goal state
       */
      virtual bool makePlan();

      NavFn planner_;
    };
    
    
    MoveBaseNAVFN::MoveBaseNAVFN()
      : MoveBase(), planner_(getCostMap().getWidth(), getCostMap().getHeight())
    {
      initialize();
    }

    bool MoveBaseNAVFN::makePlan(){
      ROS_DEBUG("Planning for new goal...\n");
   
      try {
	// Update costs
	lock();
	planner_.setCostMap(getCostMap().getMap(), true);
	unlock();
	
	// Lock the state message and obtain current position and goal
	int pos[2];
	int goal[2];
	stateMsg.lock();
	unsigned int mx, my;
	getCostMap().WC_MC(stateMsg.pos.x, stateMsg.pos.y, mx, my);
	pos[0] = mx;
	pos[1] = my;

	getCostMap().WC_MC(stateMsg.goal.x, stateMsg.goal.y, mx, my);
	goal[0] = mx;
	goal[1] = my;
	stateMsg.unlock();

	// Invoke the planner
	planner_.setStart(pos);
	planner_.setGoal(goal);
	bool success = planner_.calcNavFnAstar();

	// If good, extract plan and update
	if(success){
	  // Extract the plan in world co-ordinates
	  float *x = planner_.getPathX();
	  float *y = planner_.getPathY();
	  int len = planner_.getPathLen();
	  std::list<std_msgs::Pose2DFloat32> newPlan;
	  for(int i=0; i < len; i++){
	    double wx, wy;
	    unsigned int mx = (unsigned int) x[i];
	    unsigned int my = (unsigned int) y[i];
	    getCostMap().MC_WC(mx, my, wx, wy);
	    std_msgs::Pose2DFloat32 step;
	    step.x = wx;
	    step.y = wy;
	    newPlan.push_back(step);
	  }

	  updatePlan(newPlan);
	}

	return success;
      }
      catch (std::runtime_error const & ee) {
	ROS_ERROR("runtime_error in makePlan(): %s\n", ee.what());
      }

      return false;
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ros::highlevel_controllers::MoveBaseNAVFN node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }

  ros::fini();

  return(0);
}
