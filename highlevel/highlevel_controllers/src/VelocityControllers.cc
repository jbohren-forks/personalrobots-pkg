#include <VelocityControllers.hh>
#include <set>

namespace ros {
  namespace highlevel_controllers {
    CostMapAccessor::CostMapAccessor(const CostMap2D& costMap, double maxSize, double poseX, double poseY)
      : ObstacleMapAccessor(computeWX(costMap, maxSize, poseX),
			    computeWY(costMap, maxSize, poseY), 
			    static_cast<unsigned int>(maxSize / costMap.getResolution()), 
			    static_cast<unsigned int>(maxSize / costMap.getResolution()),
			    costMap.getResolution()), costMap_(costMap),
	maxSize_(maxSize){

      // The origin locates this grid. Convert from world coordinates to cell co-ordinates
      // to get the cell coordinates of the origin
      costMap_.WC_MC(origin_x_, origin_y_, mx_0_, my_0_); 

      printf("Creating Local %d X %d Map\n", getWidth(), getHeight());
    }

    bool CostMapAccessor::isObstacle(unsigned int mx, unsigned int my) const {
      unsigned int costMapIndex = costMap_.MC_IND(mx_0_ + mx, my_0_ + my);
      return costMap_.isObstacle(costMapIndex);
    }

    bool CostMapAccessor::isInflatedObstacle(unsigned int mx, unsigned int my) const {
      unsigned int costMapIndex = costMap_.MC_IND(mx_0_ + mx, my_0_ + my);
      return costMap_.isInflatedObstacle(costMapIndex);
    }

    void CostMapAccessor::updateForRobotPosition(double wx, double wy){
      origin_x_ = computeWX(costMap_, maxSize_, wx);
      origin_y_ = computeWX(costMap_, maxSize_, wy);
      costMap_.WC_MC(origin_x_, origin_y_, mx_0_, my_0_); 

      //printf("Moving map to locate at <%f, %f> and size of %f meters for position <%f, %f>\n",
      //origin_x_, origin_y_, maxSize_, wx, wy);
    }

    double CostMapAccessor::computeWX(const CostMap2D& costMap, double maxSize, double wx){
      return std::min(std::max(0.0,  wx - maxSize/2), costMap.getWidth() * costMap.getResolution() - maxSize);
    }

    double CostMapAccessor::computeWY(const CostMap2D& costMap, double maxSize, double wy){
      return std::min(std::max(0.0, wy - maxSize/2 ), costMap.getHeight() * costMap.getResolution() - maxSize);
    }

    TrajectoryRolloutController::TrajectoryRolloutController(rosTFClient* tf, const CostMapAccessor& ma,
							     double sim_time, int sim_steps, int samples_per_dim,
							     double robot_front_radius, double robot_side_radius, double max_occ_dist, 
							     double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
							     double acc_lim_x, double acc_lim_y, double acc_lim_th)
      : tf_(tf), ma_(ma), map_(ma.getWidth(), ma.getHeight()),
	tc_(map_, sim_time, sim_steps, samples_per_dim,robot_front_radius, robot_side_radius, max_occ_dist, 
	    pdist_scale, gdist_scale, dfast_scale, occdist_scale, 
	    acc_lim_x, acc_lim_y, acc_lim_th, tf_, ma_){
      map_.scale = ma_.getResolution();
      printf("Map Scale is: %f\n", map_.scale);
    }

    bool TrajectoryRolloutController::computeVelocityCommands(const std::list<std_msgs::Pose2DFloat32>& globalPlan,
							      const libTF::TFPose2D& pose,
							      const std_msgs::BaseVel& currentVel,
							      std_msgs::BaseVel& cmdVel,
							      std::list<std_msgs::Pose2DFloat32>& localPlan){
      localPlan.clear();

      libTF::TFPose2D drive_cmds;

      libTF::TFPose2D robot_vel;
      robot_vel.x = currentVel.vx;
      robot_vel.y = currentVel.vy;
      robot_vel.yaw = currentVel.vw;
      robot_vel.frame = "base";
      robot_vel.time = 0;

      //do we need to resize our map?
      double origin_x, origin_y;
      ma_.getOriginInWorldCoordinates(origin_x, origin_y);
      map_.sizeCheck(ma_.getWidth(), ma_.getHeight(), origin_x, origin_y);

      // Temporary Transformation till api below changes
      std::vector<std_msgs::Point2DFloat32> copiedGlobalPlan;
      for(std::list<std_msgs::Pose2DFloat32>::const_iterator it = globalPlan.begin(); it != globalPlan.end(); ++it){
	std_msgs::Point2DFloat32 p;
	p.x = it->x;
	p.y = it->y;
	copiedGlobalPlan.push_back(p);
      }

      tc_.updatePlan(copiedGlobalPlan);
  
      //compute what trajectory to drive along
      int path_index = tc_.findBestPath(pose, robot_vel, drive_cmds);

      //pass along drive commands
      cmdVel.vx = drive_cmds.x;
      cmdVel.vy = drive_cmds.y;
      cmdVel.vw = drive_cmds.yaw;

      //if we cannot move... tell someone
      if(path_index < 0)
	return false;

      // Fill out the local plan
      for(int i = 0; i < tc_.num_steps_; ++i){
	std_msgs::Pose2DFloat32 p;
	p.x = tc_.trajectory_pts_(0, path_index * tc_.num_steps_ + i); 
	p.y = tc_.trajectory_pts_(1, path_index * tc_.num_steps_ + i);
	p.th = tc_.trajectory_theta_(0, path_index * tc_.num_steps_ + i);
	localPlan.push_back(p);
      }

      return true;
    }

    std::vector<std_msgs::Point2DFloat32> TrajectoryRolloutController::drawFootprint(double x, double y, double th){
      return tc_.drawFootprint(x, y, th);
    }

    /*
    LocalSearchVelocityController::LocalSearchVelocityController(unsigned int lookAhead, double resolution, double sMax, double dsMax, double dThetaMax)
      : lookAhead_(lookAhead), 
	mapDeltaX_(lookAhead * resolution * 2), 
	mapDeltaY_(mapDeltaX_), 
	sMax_(sMax),
	dsMax_(dsMax),
	dThetaMax_(dThetaMax){}

    LocalSearchVelocityController::~LocalSearchVelocityController(){}


    bool LocalSearchVelocityController::computeVelocityCommands(const CostMapAccessor& ma, 
								const std::list<std_msgs::Pose2DFloat32>& globalPlan, 
								const libTF::TFPose2D& pose, 
								const std_msgs::BaseVel& currentVel, 
								std_msgs::BaseVel& cmdVel,
								std::list<std_msgs::Pose2DFloat32>& localPlan){

      return true;
      // Initialize the local plan with the current plan over the look ahead window
      std::vector<std_msgs::Pose2DFloat32> planSegment;
      std::vector<Selection> bestSolution;
      std::list<std_msgs::Pose2DFloat32>::const_iterator it = globalPlan.begin();
      double x = pose.x;
      double y = pose.y;
      while (it != globalPlan.end() && planSegment.size() < lookAhead_){
	const std_msgs::Pose2DFloat32& w = *it;
	Selection s;
	s.enabled = true;
	s.speed = maxSpeed();
	s.theta = calcTheta(x, y, w.x, w.y);
	planSegment.push_back(w);
	bestSolution.push_back(s);
	x = w.x;
	y = w.y;
	++it;
      }



      // Initialize utility to worst case
      double utility = MINUS_INFINITY();
      unsigned int plateauCount = 0;
      for(unsigned int i = 0; i < maxIterations(); i++){
	if(updateSolution(bestSolution, utility, planSegment, ma, pose, currentVel))
	  plateauCount = 0;
	else
	  plateauCount++;

	if(plateauCount > PLATEAU_MAX())
	  break;
      }

      // Output results
      bool cmdInitialized(false);
      for(unsigned int i=0; i<bestSolution.size(); i++){
	const Selection& s = bestSolution[i];
	if(s.enabled){
	  // Compute velocity commands
	  if(!cmdInitialized){
	    cmdInitialized = true;
	    cmdVel.vx = s.speed * cos(s.theta);
	    cmdVel.vy = s.speed * sin(s.theta);
	    if(s.speed == 0){
	      cmdVel.vx = 0;
	      cmdVel.vy = 0;
	      cmdVel.vw = s.theta - pose.yaw;
	    }
	    else {
	      cmdVel.vx = -s.speed * cos(s.theta);
	      cmdVel.vy = -s.speed * sin(s.theta);
	      cmdVel.vw = 0;
	    }
	  }
	}

	localPlan.push_back(planSegment[i]);
      }

      return utility > 0;
      
    }

    double LocalSearchVelocityController::maxSpeed() const{
      return sMax_;
    }

    unsigned int LocalSearchVelocityController::maxIterations() const {
      return 1000;
    }

    bool LocalSearchVelocityController::updateSolution(std::vector<Selection>& solution, double& utility, 
						       const std::vector<std_msgs::Pose2DFloat32>& planSegment,
						       const CostMapAccessor& ma, const libTF::TFPose2D& pos, const std_msgs::BaseVel& currentVel){

      // Could drop a waypoint

      //
      utility = 1.0;
      return false; 
    }

    double LocalSearchVelocityController::evaluate(std::vector<Selection>& solution, 
						   const std::vector<std_msgs::Pose2DFloat32>& planSegment,
						   const CostMapAccessor& ma, const libTF::TFPose2D& pos, const std_msgs::BaseVel& currentVel,
						   std::multimap<double, Flaw>& flaws){
      return 0;
    }


    double LocalSearchVelocityController::calcTheta(double x1, double y1, double x2, double y2) const{
      double num = y2-y1;
      double denom = x2 - x1;
      return atan(num / denom);
    }
    */
  }
}
