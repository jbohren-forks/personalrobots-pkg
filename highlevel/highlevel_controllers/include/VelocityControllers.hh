#ifndef  HIGHLEVEL_CONTROLLERS_VELOCITY_CONTROLERS
#define  HIGHLEVEL_CONTROLLERS_VELOCITY_CONTROLERS

#include <costmap_2d/costmap_2d.h>

#include <std_msgs/BaseVel.h>
#include <std_msgs/Pose2DFloat32.h>

// For Controller components
#include <trajectory_rollout/obstacle_map_accessor.h>
#include <trajectory_rollout/helmsman.h>

namespace ros {
  namespace highlevel_controllers {

    /**
     * Wrapper class that provides a local window onto a global cost map
     */
    class CostMapAccessor: public ObstacleMapAccessor {
    public:
      /**
       * @brief Constructor
       * @param costMap The underlying global cost map
       * @param deltaX The width in meters
       * @param deltaY The height in meters
       * @param the current x position in global coords
       * @param the current y position in global coords
       */
      CostMapAccessor(const CostMap2D& costMap, double deltaX, double deltaY, double pose_x, double pose_y);

      unsigned int getWidth() const;

      unsigned int getHeight() const;

      double getResolution() const;

      bool contains(double x, double y) const;

      bool isObstacle(unsigned int mx, unsigned int my) const;

      bool isInflatedObstacle(unsigned int mx, unsigned int my) const;

      void getOriginInWorldCoordinates(double& wx, double& wy) const;

    private:

      const CostMap2D& costMap_;
      const unsigned int width_;
      const unsigned int height_;
      double wx_0_;
      double wy_0_;
      unsigned int mx_0_;
      unsigned int my_0_;
    };

    /**
     * @brief Encapsualtion point to allow different algorithms to be used to compute velocity commands in following a path
     */
    class VelocityController {
    public:

      virtual ~VelocityController(){}

      /**
       * @brief Initialize with transform client
       */
      virtual void initialize(rosTFClient& tf){}

      /**
       * @brief Compute velocities for x, y and theta based on an obstacle map, and a current path
       */
      virtual bool computeVelocityCommands(const CostMapAccessor& ma, 
					   const std::list<std_msgs::Pose2DFloat32>& globalPlan,
					   const libTF::TFPose2D& pose,
					   const std_msgs::BaseVel& currentVel, 
					   std_msgs::BaseVel& cmdVel,
					   std::list<std_msgs::Pose2DFloat32>& localPlan) = 0;

      virtual std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th) = 0;


      virtual double getMapDeltaX() const = 0;

      virtual double getMapDeltaY() const = 0;
    };

    /** 
     * @brief For now stick an implementation in here
     */
    class TrajectoryRolloutController: public VelocityController {
    public:
      TrajectoryRolloutController(double mapDeltaX, double mapDeltaY,
				  double sim_time, int sim_steps, int samples_per_dim,
				  double robot_front_radius, double robot_side_radius, double max_occ_dist, 
				  double pdist_scale, double gdist_scale, double dfast_scale, double occdist_scale, 
				  double acc_lim_x, double acc_lim_y, double acc_lim_th);

      virtual ~TrajectoryRolloutController();

      virtual void initialize(rosTFClient& tf);

      virtual bool computeVelocityCommands(const CostMapAccessor& ma, 
					   const std::list<std_msgs::Pose2DFloat32>& globalPlan, 
					   const libTF::TFPose2D& pose, 
					   const std_msgs::BaseVel& currentVel, 
					   std_msgs::BaseVel& cmdVel,
					   std::list<std_msgs::Pose2DFloat32>& localPlan);

      virtual std::vector<std_msgs::Point2DFloat32> drawFootprint(double x, double y, double th);

      double getMapDeltaX() const {return mapDeltaX_;}

      double getMapDeltaY() const {return mapDeltaY_;}

    private:
      const double mapDeltaX_;
      const double mapDeltaY_;
      const double sim_time_;
      const int sim_steps_;
      const int samples_per_dim_;	  
      const double robot_front_radius_; 
      const double robot_side_radius_; 
      const double max_occ_dist_; 	  
      const double pdist_scale_; 
      const double gdist_scale_; 
      const double dfast_scale_; 
      const double occdist_scale_; 	  
      const double acc_lim_x_; 
      const double acc_lim_y_; 
      const double acc_lim_th_;
      Helmsman* helmsman_; // Could put this directly into the controller and skip the helmsman
    };

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
  }
}
#endif
