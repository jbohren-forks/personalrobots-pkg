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
#include <mpglue/sbpl_planner.h>
#include <mpglue/sbpl_environment.h>
#include <mpglue/plan.h>
#include <sfl/util/strutil.hpp>

#include <sbpl/headers.h>
#include <err.h>


namespace {
  
  /** Set an angle to the range -pi < angle <= +pi */
  double mod2pi(double x)
  {
    x = fmod(x, 2 * M_PI);
    if (x > M_PI)
      x -= 2 * M_PI;
    else if (x <= - M_PI)
      x += 2 * M_PI;
    return x;
  }
  
}


/**
 * @todo Resolve issue with blocking ino locking
 */

namespace ros {
  namespace highlevel_controllers {

    /**
     * @brief Specialization for the SBPL planner
     */
    class MoveBaseSBPL: public MoveBase {
    public:
      MoveBaseSBPL();

      virtual ~MoveBaseSBPL();

    protected:

      /**
       * @brief Builds a plan from current state to goal state
       */
      virtual bool makePlan();
      
      /** do MoveBase::updateGoalMsg() and increment goalCount_ */
      virtual void updateGoalMsg();
      
    private:
      bool isMapDataOK();

      boost::shared_ptr<mpglue::SBPLEnvironment> env_;
      boost::shared_ptr<mpglue::SBPLPlannerWrap> pWrap_;
      double plannerTimeLimit_; /* The amount of time given to the planner to find a plan */
      std::string planStatsFile_;
      size_t goalCount_;
      size_t prevGoalCount_;
    };
    
    
    MoveBaseSBPL::MoveBaseSBPL()
      : MoveBase(),
	goalCount_(0),
	prevGoalCount_(0xdeadbeef)
    {
      try {
	// We're throwing int exit values if something goes wrong, and
	// clean up any new instances in the catch clause. The sentry
	// gets destructed when we go out of scope, so unlock() gets
	// called no matter what.
	sentry<MoveBaseSBPL> guard(this);
	local_param("planStatsFile", planStatsFile_, string("/tmp/move_base_sbpl.log"));
	local_param("plannerTimeLimit", plannerTimeLimit_, 10.0);
	/*
	if (0 > plannerTimeLimit_) {
	  int blah;
	  local_param("plannerTimeLimit", blah, -1); // parameters are picky about dots
	  if (0 > blah) {
	    ROS_ERROR("invalid or no %s/plannerTimeLimit specified: %g",
		      getName().c_str(), plannerTimeLimit_);
	    throw int(7);
	  }
	  plannerTimeLimit_ = blah;
	}
	*/
	string environmentType;
	local_param("environmentType", environmentType, string("2D"));
	
	boost::shared_ptr<mpglue::Costmap> mcm(mpglue::createCostmap(&getCostMap()));
	boost::shared_ptr<mpglue::IndexTransform> mit(mpglue::createIndexTransform(&getCostMap()));
	
	if ("2D" == environmentType) {
	  static int const obst_cost_thresh(CostMap2D::INSCRIBED_INFLATED_OBSTACLE);
	  env_.reset(mpglue::create2DEnvironment(mcm, mit, obst_cost_thresh));
	}
	else if ("3DKIN" == environmentType) {
	  string const prefix("env3d/");
	  string obst_cost_thresh_str;
	  local_param(prefix + "obst_cost_thresh", obst_cost_thresh_str, string("lethal"));
	  int obst_cost_thresh(0);
	  if ("lethal" == obst_cost_thresh_str)
	    obst_cost_thresh = costmap_2d::CostMap2D::LETHAL_OBSTACLE;
	  else if ("inscribed" == obst_cost_thresh_str)
	    obst_cost_thresh = costmap_2d::CostMap2D::INSCRIBED_INFLATED_OBSTACLE;
	  else {
	    ROS_ERROR("invalid env3d/obst_cost_thresh \"%s\"\n"
		      "  valid options: lethal, inscribed, or circumscribed",
		      obst_cost_thresh_str.c_str());
	    throw int(6);
	  }
	  //// ignored by SBPL (at least in r9900).
	  // double goaltol_x, goaltol_y, goaltol_theta;
	  // local_param(prefix + "goaltol_x", goaltol_x, 0.3);
	  // local_param(prefix + "goaltol_y", goaltol_y, 0.3);
	  // local_param(prefix + "goaltol_theta", goaltol_theta, 30.0);
	  double nominalvel_mpersecs, timetoturn45degsinplace_secs;
	  local_param(prefix + "nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
	  local_param(prefix + "timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
	  // Could also sanity check the other parameters...
	  env_.reset(mpglue::create3DKINEnvironment(mcm, mit, obst_cost_thresh,
						    getFootprint(), nominalvel_mpersecs,
						    timetoturn45degsinplace_secs));
	}
	else {
	  ROS_ERROR("in MoveBaseSBPL ctor: invalid environmentType \"%s\", use 2D or 3DKIN",
		    environmentType.c_str());
	  throw int(2);
	}
	
	static bool const forward_search(false); // could make this configurable...
	string plannerType;
	local_param("plannerType", plannerType, string("ARAPlanner"));
	boost::shared_ptr<SBPLPlanner> sbplPlanner;
	if ("ARAPlanner" == plannerType)
	  sbplPlanner.reset(new ARAPlanner(env_->getDSI(), forward_search));
	else if ("ADPlanner" == plannerType)
	  sbplPlanner.reset(new ADPlanner(env_->getDSI(), forward_search));
	else {
	  ROS_ERROR("in MoveBaseSBPL ctor: invalid plannerType \"%s\","
		    " use ARAPlanner or ADPlanner",
		    plannerType.c_str());
	  throw int(5);
	}
	pWrap_.reset(new mpglue::SBPLPlannerWrap(env_, sbplPlanner));
      }
      catch (int ii) {
	exit(ii);
      }
      
      //Now initialize
      initialize();
    }
    
    MoveBaseSBPL::~MoveBaseSBPL(){
    }

    bool MoveBaseSBPL::isMapDataOK() {
      const CostMapAccessor& cm = getCostMap();
      
      for(unsigned int i = 0; i<cm.getWidth(); i++){
	for(unsigned int j = 0; j < cm.getHeight(); j++){
	  if(env_->IsObstacle(i, j) && cm.getCost(i, j) < CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
	    ROS_DEBUG("Extra obstacle at <%d, %d>", i, j);
	    throw "Extra obstacle in sbpl";
	  }
	  if(!env_->IsObstacle(i, j) && cm.getCost(i, j) >= CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
	    ROS_DEBUG("Missing obstacle at <%d, %d>", i, j);
	    throw "Missing obstacle in sbpl";
	  }
	}
      }

      return true;
    }

    bool MoveBaseSBPL::makePlan(){
      ROS_DEBUG("Planning for new goal...\n");
      
      try {
	// Update costs
	lock();
	const CostMapAccessor& cm = getCostMap();
	unsigned int x = cm.getWidth();
	while(x > 0){
	  x--;
	  unsigned int y = cm.getHeight();
	  while(y > 0){
	    y--;
	    // Note that ompl::EnvironmentWrapper::UpdateCost() will
	    // check if the cost has actually changed, and do nothing
	    // if it hasn't.  It internally maintains a list of the
	    // cells that have actually changed, and this list is what
	    // gets "flushed" to the planner (a couple of lines
	    // further down).
	    env_->UpdateCost(x, y, (unsigned char) cm.getCost(x, y));
	  }
	}
	unlock();
	
	// Tell the planner about the changed costs. Again, the called
	// code checks whether anything has really changed before
	// embarking on expensive computations.
	pWrap_->flushCostChanges(true);
	
	// Copy out start and goal states to minimize locking requirement. Lock was not previously required because the
	// planner and controller were running on the same thread and the only contention was for cost map updates on call
	// backs. Note that cost map queries here are const methods that merely do co-ordinate transformations, so we do not need
	// to lock protect those.
	stateMsg.lock();
	std_msgs::Pose2DFloat32 const start(stateMsg.pos);
	std_msgs::Pose2DFloat32 const goal(stateMsg.goal);
	stateMsg.unlock();
	
	// Assume the robot is constantly moving, so always set start.
	// Maybe a bit inefficient, but not as bad as "changing" the
	// goal when it hasn't actually changed.
	pWrap_->setStart(start.x, start.y, start.th);
	
	// Usually, when interweaving planning and control, the goal
	// will not have changed.  So avoid wasting computations,
	// based on the idea that we will only get a goal message if
	// the goal has actually changed.  If users send us twice to
	// the same location, well maybe they have a good reason.
	if (prevGoalCount_ != goalCount_)
	  pWrap_->setGoal(goal.x, goal.y, goal.th);
	
	// BTW if desired, we could call pWrap_->forcePlanningFromScratch(true)...
	
	// Invoke the planner, updating the statistics in the process.
	// The returned plan might be empty, but it will not contain a
	// null pointer.  On planner errors, the createPlan() method
	// throws a std::exception.
	boost::shared_ptr<mpglue::waypoint_plan_t> plan(pWrap_->createPlan());
	
	// Log to file: failure, replan, or plan
	string const prefix("[" + sfl::to_string(goalCount_) + "] ");
	string title("\n");
	if (prevGoalCount_ == goalCount_)
	  title += "replan ";
	else
	  title += "PLAN ";
	if ( plan->empty())
	  title += "FAILURE";
	else
	  title += "success";
	pWrap_->getStats().logFile(planStatsFile_.c_str(), title, prefix);
	
	prevGoalCount_ = goalCount_;
	
	if (plan->empty()) {
	  ROS_ERROR("No plan found\n");
	  return false;
	}
	updatePlan(*plan);
	return true;
      }
      catch (std::runtime_error const & ee) {
	ROS_ERROR("runtime_error in makePlan(): %s\n", ee.what());
      }
      
      return false;
    }
    
    
    void MoveBaseSBPL::updateGoalMsg()
    {
      MoveBase::updateGoalMsg();
      ++goalCount_;
    }
    
  }
}


int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ros::highlevel_controllers::MoveBaseSBPL node;

  try {
    node.run();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }

  ros::fini();

  return(0);
}
