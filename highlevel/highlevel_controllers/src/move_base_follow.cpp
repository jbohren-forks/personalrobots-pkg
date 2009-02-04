/*********************************************************************
*
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <highlevel_controllers/move_base.hh>

using namespace costmap_2d;

namespace ros {
  namespace highlevel_controllers {
    /**
     * @brief Specialization for the SBPL planner
     */
    class MoveBaseFollow: public MoveBase {
      public:
        MoveBaseFollow();

      private:

        /**
         * @brief Builds a plan from current state to goal state
         */
        virtual bool makePlan();

    };


    MoveBaseFollow::MoveBaseFollow()
      : MoveBase()
    {
      initialize();
    }

    bool MoveBaseFollow::makePlan(){
      // Lock the state message and obtain current goal
      stateMsg.lock();
      double goalX = stateMsg.goal.x, goalY = stateMsg.goal.y;
      stateMsg.unlock();

      // Simply pass on the goal as the plan for the controller
      std::list<std_msgs::Pose2DFloat32> newPlan;

      //This is a hack for now to add the goal.
      std_msgs::Pose2DFloat32 goalstep;
      goalstep.x = goalX;
      goalstep.y = goalY;
      newPlan.push_back(goalstep);

      updatePlan(newPlan);

      return true;

    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ros::highlevel_controllers::MoveBaseFollow node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }



  return(0);
}
