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

#include <boost/scoped_array.hpp>
#include <highlevel_controllers/move_base.hh>
#include <robot_actions/Pose2D.h>
#include <navfn.h>

using namespace costmap_2d;
using robot_msgs::Polyline2D;
using deprecated_msgs::Point2DFloat32;
using std::vector;

namespace ros {
  namespace highlevel_controllers {

  typedef unsigned char uchar;
  typedef unsigned int uint;



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

      void enforceConstraints (const Polyline2D& boundary, uchar* cost_map);
      void enforceConstraint (const Point2DFloat32& p1, const Point2DFloat32& p2, uchar* cost_map);

      NavFn planner_;
    
      vector<Point2DFloat32> visualized_points_;
    };
    
    
    MoveBaseNAVFN::MoveBaseNAVFN()
      : MoveBase(), planner_(getCostMap().getWidth(), getCostMap().getHeight())
    {
      initialize();
      Node::instance()->advertise<Polyline2D>("planning_boundary", 1);
    }



  void MoveBaseNAVFN::enforceConstraints (const Polyline2D& boundary, uchar* cost_map)
  {
    vector<Point2DFloat32> points;
    boundary.get_points_vec(points);

    visualized_points_.clear();
    if (points.size()>1) {
      ROS_DEBUG_NAMED ("constraints", "I am about to enforce a polyline constraint with length %u starting with points %f, %f and %f, %f",
                       points.size(), points[0].x, points[0].y, points[1].x, points[1].y);

      // \todo Need to take the header of the polyline into account
      for (uint i=0; i<points.size()-1; ++i) {
        enforceConstraint(points[i], points[i+1], cost_map);
      }
    }

    Polyline2D visualized_boundary;
    visualized_boundary.header.frame_id=global_frame_;
    visualized_boundary.set_points_vec(visualized_points_);
    Node::instance()->publish("planning_boundary", visualized_boundary);
  }

  
  // This is a hack that will occasionally miss cells.  Bresenham's algorithm is the right way.
  void MoveBaseNAVFN::enforceConstraint (const Point2DFloat32& p1, const Point2DFloat32& p2, uchar* cost_map)
  {
    const double dx=p2.x-p1.x;
    const double dy=p2.y-p1.y;
    const double l=sqrt(dx*dx+dy*dy);
    const uint num_steps = 5*ceil(l/getCostMap().getResolution());
    ROS_DEBUG_NAMED ("constraints", "Enforcing costmap constraint for line between %f, %f and %f, %f", p1.x, p1.y, p2.x, p2.y);

    for (uint i=0; i<num_steps; ++i) {
      double x=p1.x+dx*i/num_steps;
      double y=p1.y+dy*i/num_steps;
      uint ind=getCostMap().WC_IND(x, y);
      ROS_INFO ("Marking point %f, %f with index %u as obstacle", x, y, ind);
      cost_map[ind] = CostMap2D::LETHAL_OBSTACLE;

      Point2DFloat32 visualized_point;
      visualized_point.x=x;
      visualized_point.y=y;
      visualized_points_.push_back(visualized_point);
    }
  }
    
    
                           


    bool MoveBaseNAVFN::makePlan(){
      ROS_DEBUG("Planning for new goal...\n");
   
      try {
	// Update costs
	lock();

        // Need to put condition in here to check when goal constraints need to be enforced
        if (true) {

          ROS_DEBUG_NAMED ("constraints", "About to create duplicate costmap");
          // Since costmap gives us an immutable array, we have to make a copy before we make our changes
          uint height = getCostMap().getHeight();
          uint width = getCostMap().getWidth();
          const uchar* original_cost_map = getCostMap().getMap();
          uchar* cost_map = new uchar[height*width];
          ROS_DEBUG_NAMED ("constraints", "Allocated duplicate costmap");

          boost::scoped_array<uchar> cost_map_ensure_delete(cost_map); // Exception-safe-ly make sure it's deleted at the end
          memcpy(cost_map, original_cost_map, height*width*sizeof(uchar));
          ROS_DEBUG_NAMED ("constraints", "Initialized duplicate costmap");
          enforceConstraints(goalMsg.boundary, cost_map);
          planner_.setCostMap(cost_map, true);
        }

	unlock();
	
	// Lock the state message and obtain current position and goal
	int pos[2];
	int goal[2];
	stateMsg.lock();
	unsigned int mx, my;
	getCostMap().WC_MC(stateMsg.feedback.x, stateMsg.feedback.y, mx, my);
	pos[0] = mx;
	pos[1] = my;

	double goalX = stateMsg.goal.x, goalY = stateMsg.goal.y;
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
	  std::list<deprecated_msgs::Pose2DFloat32> newPlan;
	  for(int i=0; i < len; i++){
	    double wx, wy;
	    unsigned int mx = (unsigned int) x[i];
	    unsigned int my = (unsigned int) y[i];
	    getCostMap().MC_WC(mx, my, wx, wy);
	    deprecated_msgs::Pose2DFloat32 step;
	    step.x = wx;
	    step.y = wy;
	    newPlan.push_back(step);
	  }
	  
	  //This is a hack for now to add the goal.
	  deprecated_msgs::Pose2DFloat32 goalstep;
	  goalstep.x = goalX;
	  goalstep.y = goalY;
	  newPlan.push_back(goalstep);
	  
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
  ros::Node rosnode("move_base_navfn");

  ros::highlevel_controllers::MoveBaseNAVFN node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }

  

  return(0);
}
