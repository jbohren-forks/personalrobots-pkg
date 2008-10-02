#include <VelocityControllers.hh>
#include <set>

namespace ros {
  namespace highlevel_controllers {
    CostMapAccessor::CostMapAccessor(const CostMap2D& costMap, double deltaX, double deltaY, double poseX, double poseY)
      : ObstacleMapAccessor(std::min(std::max(0.0,  poseX - deltaX/2), costMap.getWidth() * costMap.getResolution() - deltaX),
			    std::min(std::max(0.0, poseY - deltaY/2 ), costMap.getHeight() * costMap.getResolution() - deltaY), 
			    static_cast<unsigned int>(deltaX / costMap.getResolution()), 
			    static_cast<unsigned int>(deltaY / costMap.getResolution()),
			    costMap.getResolution()), costMap_(costMap){

      // The origin locates this grid. Convert from world coordinates to cell co-ordinates
      // to get the cell coordinates of the origin
      costMap_.WC_MC(origin_x_, origin_y_, mx_0_, my_0_);    
    }

    bool CostMapAccessor::isObstacle(unsigned int mx, unsigned int my) const {
      unsigned int costMapIndex = costMap_.MC_IND(mx_0_ + mx, my_0_ + my);
      return costMap_.isObstacle(costMapIndex);
    }

    bool CostMapAccessor::isInflatedObstacle(unsigned int mx, unsigned int my) const {
      unsigned int costMapIndex = costMap_.MC_IND(mx_0_ + mx, my_0_ + my);
      return costMap_.isInflatedObstacle(costMapIndex);
    }

    TrajectoryRolloutController::TrajectoryRolloutController(double mapDeltaX, double mapDeltaY,
							     double sim_time, int sim_steps, int samples_per_dim,
							     double robot_front_radius, double robot_side_radius, double max_occ_dist, 
							     double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
							     double acc_lim_x, double acc_lim_y, double acc_lim_th)
      : mapDeltaX_(mapDeltaX),
	mapDeltaY_(mapDeltaY),
	sim_time_(sim_time),
	sim_steps_(sim_steps),
	samples_per_dim_(samples_per_dim),	  
	robot_front_radius_(robot_front_radius), 
	robot_side_radius_(robot_side_radius), 
	max_occ_dist_(max_occ_dist), 	  
	pdist_scale_(pdist_scale), 
	gdist_scale_(gdist_scale), 
	dfast_scale_(dfast_scale), 
	occdist_scale_(occdist_scale), 	  
	acc_lim_x_(acc_lim_x),
	acc_lim_y_(acc_lim_y),
	acc_lim_th_(acc_lim_th),
	helmsman_(NULL){}

    TrajectoryRolloutController::~TrajectoryRolloutController(){
      if(helmsman_ != NULL)
	delete helmsman_;
    }

    void TrajectoryRolloutController::initialize(rosTFClient& tf){
      // Should just be called once
      if(helmsman_ !=NULL)
	return;

      helmsman_ = new Helmsman(tf, sim_time_, sim_steps_, samples_per_dim_,robot_front_radius_, robot_side_radius_, max_occ_dist_, 
			       pdist_scale_, gdist_scale_, dfast_scale_, occdist_scale_, 
			       acc_lim_x_, acc_lim_y_, acc_lim_th_);
    }

    bool TrajectoryRolloutController::computeVelocityCommands(const CostMapAccessor& ma, 
							      const std::list<std_msgs::Pose2DFloat32>& globalPlan,
							      const libTF::TFPose2D& pose,
							      const std_msgs::BaseVel& currentVel,
							      std_msgs::BaseVel& cmdVel,
							      std::list<std_msgs::Pose2DFloat32>& localPlan){
      if(helmsman_ == NULL){
	cmdVel.vx = 0;
	cmdVel.vy = 0;
	cmdVel.vw = 0;
	return false;
      }

      double vx, vy, vw;
      bool result = helmsman_->computeVelocityCommands(ma, globalPlan, currentVel.vx, currentVel.vy, currentVel.vw,  vx, vy, vw, localPlan);
      printf("Selected velocity vector: (%f, %f, %f)\n", vx, vy, vw);
      cmdVel.vx = vx;
      cmdVel.vy = vy;
      cmdVel.vw = vw;
      return result;
    }

    std::vector<std_msgs::Point2DFloat32> TrajectoryRolloutController::drawFootprint(double x, double y, double th){
      if(helmsman_)
        return helmsman_->drawFootprint(x, y, th);
      else{
        std::vector<std_msgs::Point2DFloat32> empty;
        return empty;
      }
    }


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
      /*
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
      */
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
  }
}
