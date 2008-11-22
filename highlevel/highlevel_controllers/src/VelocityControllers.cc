#include <VelocityControllers.hh>
#include <set>

namespace ros {
  namespace highlevel_controllers {

    TrajectoryRolloutController::TrajectoryRolloutController(tf::TransformListener* tf, const CostMapAccessor& ma,
							     double sim_time, int sim_steps, int samples_per_dim,
							     double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
							     double acc_lim_x, double acc_lim_y, double acc_lim_th, std::vector<std_msgs::Point2DFloat32> footprint_spec)
      : tf_(tf), ma_(ma), map_(ma.getWidth(), ma.getHeight()),
	tc_(map_, sim_time, sim_steps, samples_per_dim,
	    pdist_scale, gdist_scale, dfast_scale, occdist_scale, 
	    acc_lim_x, acc_lim_y, acc_lim_th, tf_, ma_, footprint_spec){
      map_.scale = ma_.getResolution();
      printf("Map Scale is: %f\n", map_.scale);
    }

    bool TrajectoryRolloutController::computeVelocityCommands(const std::list<std_msgs::Pose2DFloat32>& globalPlan,
							      const tf::Stamped<tf::Pose>& pose,
							      const std_msgs::BaseVel& currentVel,
							      std_msgs::BaseVel& cmdVel,
							      std::list<std_msgs::Pose2DFloat32>& localPlan){
      localPlan.clear();

      tf::Stamped<tf::Pose> drive_cmds;
      drive_cmds.frame_id_ = "base_link";

      tf::Stamped<tf::Pose> robot_vel;
      btQuaternion qt(currentVel.vw, 0, 0);
      robot_vel.setData(btTransform(qt, btVector3(currentVel.vx, currentVel.vy, 0)));
      robot_vel.frame_id_ = "base_link";
      robot_vel.stamp_ = ros::Time((uint64_t)0ULL);

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
      Trajectory path = tc_.findBestPath(pose, robot_vel, drive_cmds);

      //pass along drive commands
      cmdVel.vx = drive_cmds.getOrigin().getX();
      cmdVel.vy = drive_cmds.getOrigin().getY();
      double uselessPitch, uselessRoll, yaw;
      drive_cmds.getBasis().getEulerZYX(yaw, uselessPitch, uselessRoll);
 
      cmdVel.vw = yaw;

      //if we cannot move... tell someone
      if(path.cost_ < 0)
        return false;

      // Fill out the local plan
      for(int i = 0; i < tc_.num_steps_; ++i){
        double p_x, p_y, p_th;
        path.getPoint(i, p_x, p_y, p_th);

        std_msgs::Pose2DFloat32 p;
        p.x = p_x; 
        p.y = p_y;
        p.th = p_th;
        localPlan.push_back(p);
      }

      return true;
    }

    std::vector<std_msgs::Point2DFloat32> TrajectoryRolloutController::drawFootprint(double x, double y, double th){
      return tc_.drawFootprint(x, y, th);
    }

    void TrajectoryRolloutController::getLocalGoal(double& x, double& y){
      x = tc_.goal_x_;
      y = tc_.goal_y_;
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
