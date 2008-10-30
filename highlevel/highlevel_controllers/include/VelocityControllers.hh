#ifndef  HIGHLEVEL_CONTROLLERS_VELOCITY_CONTROLERS
#define  HIGHLEVEL_CONTROLLERS_VELOCITY_CONTROLERS

#include <costmap_2d/costmap_2d.h>

#include <std_msgs/BaseVel.h>
#include <std_msgs/Pose2DFloat32.h>
#include <std_msgs/Point2DFloat32.h>
#include <trajectory_rollout/map_grid.h>
#include <trajectory_rollout/trajectory.h>
#include <trajectory_rollout/trajectory_controller.h>

// For transform support
#include <rosTF/rosTF.h>

using namespace costmap_2d;

namespace ros {
  namespace highlevel_controllers {

    /**
     * @brief Encapsualtion point to allow different algorithms to be used to compute velocity commands in following a path
     */
    class VelocityController {
    public:

      virtual ~VelocityController(){}

      /**
       * @brief Compute velocities for x, y and theta based on an obstacle map, and a current path
       */
      virtual bool computeVelocityCommands(const std::list<std_msgs::Pose2DFloat32>& globalPlan,
					   const libTF::TFPose2D& pose,
					   const std_msgs::BaseVel& currentVel, 
					   std_msgs::BaseVel& cmdVel,
					   std::list<std_msgs::Pose2DFloat32>& localPlan) = 0;

      virtual std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th) = 0;
      virtual void getLocalGoal(double& x, double& y) = 0;
    };

    /** 
     * @brief For now stick an implementation in here
     */
    class TrajectoryRolloutController: public VelocityController {
    public:
      TrajectoryRolloutController(rosTFClient* tf, const CostMapAccessor& ma,
				  double sim_time, int sim_steps, int samples_per_dim,
				  double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
				  double acc_lim_x, double acc_lim_y, double acc_lim_th, std::vector<std_msgs::Point2DFloat32> footprint_spec);

      virtual ~TrajectoryRolloutController(){}

      virtual bool computeVelocityCommands(const std::list<std_msgs::Pose2DFloat32>& globalPlan, 
					   const libTF::TFPose2D& pose, 
					   const std_msgs::BaseVel& currentVel, 
					   std_msgs::BaseVel& cmdVel,
					   std::list<std_msgs::Pose2DFloat32>& localPlan);

      virtual std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th);
      virtual void getLocalGoal(double& x, double& y);

    private:

      //transform client
      rosTFClient* tf_;

      // Cost map accessor
      const costmap_2d::ObstacleMapAccessor& ma_;

      //a map
      MapGrid map_;

      //trajectory controller
      TrajectoryController tc_;
    };

    /*
    class LocalSearchVelocityController : public VelocityController {
    public:

      LocalSearchVelocityController(unsigned int lookAhead, double resolution, double sMax, double dsMax, double dThetaMax);

      virtual ~LocalSearchVelocityController();

      virtual bool computeVelocityCommands(const CostMapAccessor& ma, 
					   const std::list<std_msgs::Pose2DFloat32>& globalPlan, 
					   const libTF::TFPose2D& pose, 
					   const std_msgs::BaseVel& currentVel, 
					   std_msgs::BaseVel& cmdVel,
					   std::list<std_msgs::Pose2DFloat32>& localPlan);

      virtual std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th) { std::vector<std_msgs::Point2DFloat32> empty; return empty;}
      virtual void getLocalGoal(double& x, double& y){x = 0.0; y = 0.0;}

      double getMapDeltaX() const {return mapDeltaX_;}

      double getMapDeltaY() const {return mapDeltaY_;}

    private:

      struct Selection {
	bool enabled;
	double speed;
	double theta;
      };

      struct Flaw{
	unsigned int index;
      };

      double maxSpeed() const;

      unsigned int maxIterations() const;

      bool updateSolution(std::vector<Selection>& solution, double& utility,
			  const std::vector<std_msgs::Pose2DFloat32>& planSegment,
			  const CostMapAccessor& ma, const libTF::TFPose2D& pos, const std_msgs::BaseVel& currentVel);

      double evaluate(std::vector<Selection>& solution, 
		      const std::vector<std_msgs::Pose2DFloat32>& planSegment,
		      const CostMapAccessor& ma, const libTF::TFPose2D& pos, const std_msgs::BaseVel& currentVel,
		      std::multimap<double, Flaw>& flaws);

      double calcTheta(double x1, double y1, double x2, double y2) const;

      static double MINUS_INFINITY() { return -99999999;}

      static double PLATEAU_MAX() { return 50;}

      const unsigned int lookAhead_;
      const double mapDeltaX_;
      const double mapDeltaY_;
      const double sMax_;
      const double dsMax_;
      const double dThetaMax_;
    };
    */
  }
}
#endif
