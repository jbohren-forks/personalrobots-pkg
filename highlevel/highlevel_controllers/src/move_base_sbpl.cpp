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

//sbpl headers file
#include <headers.h>

/**
 * @todo Resolve issue with blocking ino locking
 */

/**
 * @brief Specialization for the SBPL planner
 */
class MoveBaseSBPL: public MoveBase {
public:
  MoveBaseSBPL(double windowLength, unsigned char lethalObstacleThreshold, unsigned char noInformation, double maxZ, double inflationRadius);

  virtual ~MoveBaseSBPL();

protected:

  /**
   * @brief Called during update of cost map. Will just buffer and handle in batch.
   * @see applyMapUpdates
   */
  virtual void handleMapUpdates(const std::vector<unsigned int>& insertions, std::vector<unsigned int>& deletions);

  /**
   * @brief Builds a plan from current state to goal state
   */
  virtual bool makePlan();

private:

  static const double PLANNER_TIMEOUT = 10.0; /**< Temporary constant */

  void applyMapUpdates();

  MDPConfig mdpCfg_;
  EnvironmentNAV2D envNav2D_;
  ARAPlanner* araPlanner_;

  std::vector< std::vector<unsigned int> > insertionsBuffer_; /*!< Buffers insertions generated on the cost map */
  std::vector< std::vector<unsigned int> > deletionsBuffer_; /*!< Buffers insertions generated on the cost map */
};

MoveBaseSBPL::MoveBaseSBPL(double windowLength, unsigned char lethalObstacleThreshold, unsigned char noInformation, double maxZ, double inflationRadius)
  : MoveBase(windowLength, lethalObstacleThreshold, noInformation, maxZ, inflationRadius){

  lock();
  const CostMap2D& cm = getCostMap();
  char* initialMapData = new char[cm.getWidth() * cm.getHeight()];
  // Set all to 0 (unoccupied) by default.
  memset(initialMapData, 0, cm.getWidth() * cm.getHeight());

  // Now get all the obstacle cells, static and dyanmic, and assign values accordingly
  std::vector<unsigned int> occupiedCells;
  cm.getOccupiedCellDataIndexList(occupiedCells);

  for(std::vector<unsigned int>::const_iterator it = occupiedCells.begin(); it != occupiedCells.end(); ++it){
    unsigned int id = *it;
    initialMapData[id] = 1;
  }

  // Initial Configuration is set with
  envNav2D_.SetConfiguration(cm.getWidth(), cm.getHeight(), initialMapData, 0, 0, 0, 0);
  envNav2D_.InitGeneral();

  unlock();

  // Cleanup
  delete initialMapData;
  
  //Initialize MDP Info
  if(!envNav2D_.InitializeMDPCfg(&mdpCfg_)) 
  {
    printf("ERROR: InitializeMDPCfg failed\n");
    exit(1);
  }

  // Finally, allocate the planner
  araPlanner_ = new ARAPlanner(&envNav2D_);
}

MoveBaseSBPL::~MoveBaseSBPL(){
  if(araPlanner_ != NULL)
    delete araPlanner_;
}

/**
 * @brief This is called during a cost map update.
 */
void MoveBaseSBPL::handleMapUpdates(const std::vector<unsigned int>& insertions, std::vector<unsigned int>& deletions){
  // Just buffer insertions and deletions. Environment can be updated in a batch prior to planning since planner is
  // not thread safe
  lock();
  insertionsBuffer_.push_back(insertions);
  deletionsBuffer_.push_back(deletions);
  unlock();
}

/**
 * Apply insert.deletion operations pairwise to replicate transaction order and preserve correct data. This will do a batch update
 * to the environment and should be called prior to planning.
 */
void MoveBaseSBPL::applyMapUpdates(){
  //lock();
  const CostMap2D& cm = getCostMap();
  for(unsigned int i = 0; i < insertionsBuffer_.size(); i++){
    const std::vector<unsigned int> insertions = insertionsBuffer_[i];
    for(std::vector<unsigned int>::const_iterator it = insertions.begin(); it != insertions.end(); ++it){
      unsigned int id = *it;
      unsigned int x, y; // Cell coordinates
      cm.convertFromMapIndexToXY(id, x, y);
      envNav2D_.UpdateCost(x, y, 1);
    }

    const std::vector<unsigned int> deletions = deletionsBuffer_[i];
    for(std::vector<unsigned int>::const_iterator it = deletions.begin(); it != deletions.end(); ++it){
      unsigned int id = *it;
      unsigned int x, y; // Cell coordinates
      cm.convertFromMapIndexToXY(id, x, y);
      envNav2D_.UpdateCost(x, y, 0);
    }
  }
  insertionsBuffer_.clear();
  deletionsBuffer_.clear();
  //unlock();
}

bool MoveBaseSBPL::makePlan(){
  std::cout << "Planning for new goal...\n";
  // Reset the current plan
  plan_.clear();

  // Apply map updates that were buffered from the cost map
  applyMapUpdates();

  size_t x, y;
  const CostMap2D& cm = getCostMap();

  // Set start state based on global pose.
  cm.convertFromWorldCoordToIndexes(stateMsg.pos.x, stateMsg.pos.y, x, y);
  envNav2D_.SetStart(x, y);
  araPlanner_->set_start(envNav2D_.GetStateFromCoord(x, y));

  // Set goal state
  goalMsg.lock();
  cm.convertFromWorldCoordToIndexes(goalMsg.goal.x, goalMsg.goal.y, x, y);
  goalMsg.unlock();
  envNav2D_.SetGoal(x, y);
  araPlanner_->set_goal(envNav2D_.GetStateFromCoord(x, y));

  // Invoke the planner
  std::vector<int> solutionStateIDs;

  // Extract the solution, if available
  if(araPlanner_->replan(PLANNER_TIMEOUT, &solutionStateIDs)){
    for(std::vector<int>::const_iterator it = solutionStateIDs.begin(); it != solutionStateIDs.end(); ++it){
      int state = *it;
      int mx, my;
      envNav2D_.GetCoordFromState(state, mx, my);
      plan_.push_back(std::pair<unsigned int, unsigned int>(mx, my));
    }
    std::cout << "Plan found\n";
    publishPlan();
    return true;
  }
  else{
    std::cout << "No plan found\n";
    return false;
  }
}


int main(int argc, char** argv)
{
  /*
  if(argc != 2){
    std::cout << "Usage: ./";
    return -1;
  }
  */
  ros::init(argc,argv);

  // Extract parameters
  //const std::string param = argv[1];

  MoveBaseSBPL node(10, 1, 255, 2, 0.5);
  node.run();
  ros::fini();


  return(0);
}
