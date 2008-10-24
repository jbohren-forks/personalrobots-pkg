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
      EnvironmentNAV2D envNav2D_;
      SBPLPlannerManager * pMgr_;
      SBPLPlannerStatistics pStat_;
      double plannerTimeLimit_; /* The amount of time given to the planner to find a plan */
    };

    MoveBaseSBPL::MoveBaseSBPL()
      : MoveBase(){
      param(get_name() + "/plannerTimeLimit", plannerTimeLimit_, 1.0);

      lock();
      const CostMap2D& cm = getCostMap();
      unsigned char* initialMapData = new unsigned char[cm.getWidth() * cm.getHeight()];
      // Fill with cost map data
      memcpy(initialMapData, cm.getMap(), cm.getWidth() * cm.getHeight());

      // Initial Configuration is set with the threshold for obstacles set to the inscribed obstacle threshold. These, lethal obstacles, and cells with
      // no information will thus be regarded as obstacles
      envNav2D_.InitializeEnv(cm.getWidth(), cm.getHeight(), initialMapData, 0, 0, 0, 0, CostMap2D::INSCRIBED_INFLATED_OBSTACLE);

      // Cleanup
      delete[] initialMapData;
  
      bool success = envNav2D_.InitializeMDPCfg(&mdpCfg_);

      isMapDataOK();

      unlock();

      if(!success){
	ROS_INFO("ERROR: InitializeMDPCfg failed\n");
	exit(1);
      }
      
      pMgr_ = new SBPLPlannerManager(&envNav2D_, false, &mdpCfg_);
      if ( ! pMgr_->select("ARAPlanner", false)) {
	delete pMgr_;
	errx(EXIT_FAILURE, "ERROR in MoveBaseSBPL ctor: pMgr_->select(\"ARAPlanner\") failed");
      }
      pStat_.pushBack("ARAPlanner");

      //Now initialize
      initialize();
    }
    
    MoveBaseSBPL::~MoveBaseSBPL(){
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
	envNav2D_.UpdateCost(x, y, cm.getCost(x, y));
      }
    }

    bool MoveBaseSBPL::isMapDataOK() {
      const CostMap2D& cm = getCostMap();
      
      for(unsigned int i = 0; i<cm.getWidth(); i++){
	for(unsigned int j = 0; j < cm.getHeight(); j++){
	  if(envNav2D_.IsObstacle(i, j) && cm.getCost(i, j) < CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
	    ROS_DEBUG("Extra obstacle at <%d, %d>", i, j);
	    throw "Extra obstacle in sbpl";
	  }
	  if(!envNav2D_.IsObstacle(i, j) && cm.getCost(i, j) >= CostMap2D::INSCRIBED_INFLATED_OBSTACLE){
	    ROS_DEBUG("Missing obstacle at <%d, %d>", i, j);
	    throw "Missing obstacle in sbpl";
	  }
	}
      }

      return true;
    }

    bool MoveBaseSBPL::makePlan(){
      ROS_DEBUG("Planning for new goal...\n");
      
#warning "don't we have to lock() and unlock() here?"

      try {
	SBPLPlannerStatistics::entry & se(pStat_.top());
	const CostMap2D& cm = getCostMap();
	
	// Set start state based on global pose, updating statistics in the process.
	se.start = stateMsg.pos;
	cm.WC_MC(stateMsg.pos.x, stateMsg.pos.y, se.startIx, se.startIy);
	envNav2D_.SetStart(se.startIx, se.startIy);
	se.startState = envNav2D_.GetStateFromCoord(se.startIx, se.startIy);
	int status(pMgr_->set_start(se.startState));
	if (1 != status) {
	  ROS_ERROR("failed to set start state ID %d from (%ud, %ud): pMgr_->set_start() returned %d\n",
		    se.startState, se.startIx, se.startIy, status);
	  return false;
	}
	
	// Set goal state, updating statistics in the process.
	se.goal = stateMsg.goal;
	cm.WC_MC(stateMsg.goal.x, stateMsg.goal.y, se.goalIx, se.goalIy);
	envNav2D_.SetGoal(se.goalIx, se.goalIy);
	se.goalState = envNav2D_.GetStateFromCoord(se.goalIx, se.goalIy);
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
	    int state = *it;
	    int mx, my;
	    envNav2D_.GetCoordFromState(state, mx, my);
	    
	    double wx, wy;
	    cm.MC_WC(mx, my, wx, wy);
	    std_msgs::Pose2DFloat32 waypoint;
	    waypoint.x = wx;
	    waypoint.y = wy;
	    
	    // update stats:
	    // - first round, nothing to do
	    // - second round, update path length only
	    // - third round, update path length and angular change
	    if (plan.empty()) {
	      prevx = wx;
	      prevy = wy;
	    }
	    else {
	      double const dx(wx - prevx);
	      double const dy(wy - prevy);
	      se.plan_length_m += sqrt(pow(dx, 2) + pow(dy, 2));
	      double const th(atan2(dy, dx));
	      if (42 > prevth) // see 42.17 above
		se.plan_angle_change_rad += fabs(mod2pi(th - prevth));
	      prevx = wx;
	      prevy = wy;
	      prevth = th;
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
