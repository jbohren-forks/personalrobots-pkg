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

#include <MoveBase.hh>
#include <sbpl_util.hh>

//sbpl headers file
#include <headers.h>
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
       * @brief Called during update of cost map. Will just buffer and handle in batch.
       * @see applyMapUpdates
       */
      virtual void handleMapUpdates(const std::vector<unsigned int>& updates);

      /**
       * @brief Builds a plan from current state to goal state
       */
      virtual bool makePlan();
      
    private:

      bool isMapDataOK();

      MDPConfig mdpCfg_;
      EnvironmentWrapper * env_;
      SBPLPlannerManager * pMgr_;
      SBPLPlannerStatistics pStat_;
      double plannerTimeLimit_; /* The amount of time given to the planner to find a plan */
    };
    
    
    MoveBaseSBPL::MoveBaseSBPL()
      : MoveBase(),
	env_(0),
	pMgr_(0)
    {
      try {
	// We're throwing int exit values if something goes wrong, and
	// clean up any new instances in the catch clause. The sentry
	// gets destructed when we go out of scope, so unlock() gets
	// called no matter what.
	sentry guard(this);
	
	param(get_name() + "/plannerTimeLimit", plannerTimeLimit_, 1.0);
	string environmentType;
	param(get_name() + "/environmentType", environmentType, string("2D"));
	
	if ("2D" == environmentType) {
	  // Initial Configuration is set with the threshold for
	  // obstacles set to the inscribed obstacle threshold. These,
	  // lethal obstacles, and cells with no information will thus
	  // be regarded as obstacles
	  env_ = new EnvironmentWrapper2D(getCostMap(), 0, 0, 0, 0,
					  CostMap2D::INSCRIBED_INFLATED_OBSTACLE);
	}
	else if ("3DKIN" == environmentType) {
	  string const prefix(get_name() + "/env3d/");
	  double goaltol_x, goaltol_y, goaltol_theta, cellsize_m,
	    nominalvel_mpersecs, timetoturn45degsinplace_secs;
	  param(prefix + "goaltol_x", goaltol_x, 0.3);
	  param(prefix + "goaltol_y", goaltol_y, 0.3);
	  param(prefix + "goaltol_theta", goaltol_theta, 30.0);
	  param(prefix + "cellsize_m", cellsize_m, -1.0);
	  param(prefix + "nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
	  param(prefix + "timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);
	  if (0 > cellsize_m) {
	    // Would be better to read the size from the map!
	    ROS_ERROR("invalid env3d_cellsize_m %g, must correspond to the costmap", cellsize_m);
	    throw int(1);
	  }
	  // Could also sanity check the other parameters...
	  env_ = new EnvironmentWrapper3DKIN(getCostMap(), 0, 0, 0, 0, 0, 0,
					     goaltol_x, goaltol_y, goaltol_theta,
					     getFootprint(),
					     cellsize_m, nominalvel_mpersecs,
					     timetoturn45degsinplace_secs);
	}
	else {
	  ROS_ERROR("in MoveBaseSBPL ctor: invalid environmentType \"%s\", use 2D or 3DKIN",
		    environmentType);
	  throw int(2);
	}
	
	// This (weird?) order of inits is historical, could maybe be
	// cleaned up...
	bool const success(env_->InitializeMDPCfg(&mdpCfg_));
// 	try {
// 	  // This one throws a string if something goes awry... and
// 	  // always returns true. So no use cecking its
// 	  // retval. Another candidate for cleanup.
// 	  isMapDataOK();
// 	}
// 	catch (char const * ee) {
// 	  ROS_ERROR("in MoveBaseSBPL ctor: isMapDataOK() said %s", ee);
// 	  throw int(3);
// 	}
	if ( ! success) {
	  ROS_ERROR("in MoveBaseSBPL ctor: env_->InitializeMDPCfg() failed");
	  throw int(4);
	}
	
	string plannerType;
	param(get_name() + "/plannerType", plannerType, string("ARAPlanner"));
	pMgr_ = new SBPLPlannerManager(env_->getDSI(), false, &mdpCfg_);
	if ( ! pMgr_->select(plannerType, false)) {
	  ROS_ERROR("in MoveBaseSBPL ctor: pMgr_->select(%s) failed", plannerType);
	  throw int(5);
	}
	pStat_.pushBack(plannerType);
      }
      catch (int ii) {
	delete env_;
	delete pMgr_;
	exit(ii);
      }
      
      //Now initialize
      initialize();
    }
    
    MoveBaseSBPL::~MoveBaseSBPL(){
      delete env_;
      delete pMgr_;
    }

    /**
     * @brief This is called during a cost map update. Will insert new updates, possibly overwriting prior values
     */
    void MoveBaseSBPL::handleMapUpdates(const std::vector<unsigned int>& updates){
      
      const CostMap2D& cm = getCostMap();

      for(std::vector<unsigned int>::const_iterator it = updates.begin(); it != updates.end(); ++it){
	unsigned int x, y; // Cell coordinates
	cm.IND_MC(*it, x, y);
	env_->UpdateCost(x, y, cm.getCost(x, y));
      }
    }

    bool MoveBaseSBPL::isMapDataOK() {
      const CostMap2D& cm = getCostMap();
      
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
	SBPLPlannerStatistics::entry & se(pStat_.top());
	const CostMap2D& cm = getCostMap();
	
	// Copy out start and goal states to minimize locking requirement. Lock was not previously required because the
	// planner and controller were running on the same thread and the only contention was for cost map updates on call
	// backs. Note that cost map queries here are const methods that merely do co-ordinate transformations, so we do not need
	// to lock protect those.
	stateMsg.lock();
	se.start = stateMsg.pos;
	se.goal = stateMsg.goal;
	stateMsg.unlock();

	// Set start state based on global pose, updating statistics in the process.
	cm.WC_MC(se.start.x, se.start.y, se.startIx, se.startIy);
 	env_->SetStart(se.start);
 	se.startState = env_->GetStateFromPose(se.start);
	if (0 > se.startState) {
	  ROS_ERROR("invalid start state ID %d from pose (%+8.3f, %+8.3f): outside of map?\n",
		    se.startState, se.start.x, se.start.y);
	  return false;
	}
	int status(pMgr_->set_start(se.startState));
	if (1 != status) {
	  ROS_ERROR("failed to set start state ID %d from (%ud, %ud): pMgr_->set_start() returned %d\n",
		    se.startState, se.startIx, se.startIy, status);
	  return false;
	}
	
	// Set goal state, updating statistics in the process.
	cm.WC_MC(se.goal.x, se.goal.y, se.goalIx, se.goalIy);
 	env_->SetGoal(se.goal);
 	se.goalState = env_->GetStateFromPose(se.goal);
	if (0 > se.goalState) {
	  ROS_ERROR("invalid goal state ID %d from pose (%+8.3f, %+8.3f): outside of map?\n",
		    se.goalState, se.goal.x, se.goal.y);
	  return false;
	}
	status = pMgr_->set_goal(se.goalState);
	if (1 != status) {
	  ROS_ERROR("failed to set goal state ID %d from (%ud, %ud): pMgr_->set_goal() returned %d\n",
		    se.goalState, se.goalIx, se.goalIy, status);
	  return false;
	}
	
	// Invoke the planner, updating the statistics in the process.
	std::vector<int> solutionStateIDs;
	se.allocated_time_sec = plannerTimeLimit_;
	se.status = pMgr_->replan(se.allocated_time_sec, &se.actual_time_sec, &solutionStateIDs);
	
	// Extract the solution, if available, and update statistics (as usual).
	se.plan_length_m = 0;
	se.plan_angle_change_rad = 0;
	if (1 == se.status) {
	  std::list<std_msgs::Pose2DFloat32> plan;
	  double prevx, prevy, prevth;
	  prevth = 42.17;	// to detect when it has been initialized (see 42 below)
	  for(std::vector<int>::const_iterator it = solutionStateIDs.begin(); it != solutionStateIDs.end(); ++it){
	    std_msgs::Pose2DFloat32 const waypoint(env_->GetPoseFromState(*it));
	    
	    // update stats:
	    // - first round, nothing to do
	    // - second round, update path length only
	    // - third round, update path length and angular change
	    if (plan.empty()) {
	      prevx = waypoint.x;
	      prevy = waypoint.y;
	    }
	    else {
	      double const dx(waypoint.x - prevx);
	      double const dy(waypoint.y - prevy);
	      se.plan_length_m += sqrt(pow(dx, 2) + pow(dy, 2));
	      double const th(atan2(dy, dx));
	      if (42 > prevth) // see 42.17 above
		se.plan_angle_change_rad += fabs(mod2pi(th - prevth));
	      prevx = waypoint.x;
	      prevy = waypoint.y;
	      prevth = th;
#warning 'add the cumulation of delta(waypoint.th) now that we can have 3D plans'
	    }
	    
	    plan.push_back(waypoint);
	  }
	  // probably we should add the delta from the last theta to
	  // the goal theta onto se.plan_angle_change_rad here, but
	  // that depends on whether our planner handles theta for us,
	  // and needs special handling if we have no plan...
	  se.logInfo("move_base_sbpl: ");
	  se.logFile("/tmp/move_base_sbpl.log");
	  pStat_.pushBack(pMgr_->getName());
	  
	  updatePlan(plan);
	  return true;
	}
      }
      catch (std::runtime_error const & ee) {
	ROS_ERROR("runtime_error in makePlan(): %s\n", ee.what());
	return false;
      }

      ROS_ERROR("No plan found\n");
      return false;
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
