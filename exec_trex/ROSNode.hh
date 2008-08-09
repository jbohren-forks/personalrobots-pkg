#ifndef H_ROSNode
#define H_ROSNode

#include "ExecDefs.hh"
#include "Observer.hh"

// roscpp
#include <ros/node.h>

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/LaserScan.h>
#include <pthread.h>

// For arms
#include <std_msgs/PR2Arm.h>

// For GUI debug
#include <std_msgs/Polyline2D.h>

// For transform support
#include <rosTF/rosTF.h>

//for KDL
#include <robot_kinematics/robot_kinematics.h>

typedef struct
{
  double* pts;
  size_t pts_num;
  ros::Time ts;
} laser_pts_t;



namespace TREX{
  class ROSNode;
  typedef EUROPA::Id<ROSNode> ROSNodeId;


  class ROSNode : public ros::node
  {
  public:
    /**
     * Creates and instance of the singleton and adds a reference.
     */
    static ROSNodeId request();
    /**
     * Releases a reference to the singleton.
     */
    static void release();
    

    ROSNode();

    ~ROSNode();

    /**
     * Called by Top Level Adapter to publish execution status. Needs to translate observation
     * into suitable message structure.
     */
    void my_publish(const Observation& obs); 
    
    /**
     * Called by Bottom Level Adapter to dispatch goals. Needs to translate token into suitable message structure
     */
    void dispatchWaypoint(const TokenId& goal);

    /**
     * Called by Bottom Level Adapter to dispatch velocities. Needs to translate token into suitable message structure
     */
    void dispatchVel(const TokenId& cmd_vel);

    /**
     * Called by Bottom Level Adapter to dispatch arm commands. Needs to translate token into suitable message structure
     */
    void dispatchArm(const TokenId& cmd_vel, TICK currentTick);    

    void dispatchEndEffector(const TokenId& cmd_end, TICK currentTick);

    /**
     * Get all observations.
     */
    void get_obs(std::vector<Observation*>& obsBuffer, TICK currentTick);

    /**
     * Get all arm observations.
     */
    void get_arm_obs(std::vector<Observation*>& obsBuffer, TICK currentTick);

    /**
     * Get all arm observations.
     */
    void get_end_effector_obs(std::vector<Observation*>& buff, TICK currentTick);

    /**
     * test if the node is initialized with inbound messages it requires
     */
    bool isInitialized() const;

    /**
     * @brief Accessor for armSerialChain for us in Robot Kinematics
     * @todo These should be const accessors. But underlying kinematics api is not written that way.
     */
    robot_kinematics::SerialChain* getArmSerialChain() const {return _armSerialChain;}

    rosTFClient tf;
       
  private:
    
    /**
     * Get Observation for vehicle state if available
     */
    Observation* get_vs_obs();
    
    /**
     * Get Observation for vehicle velocities if available
     */
    //Observation* get_vc_obs();
    
    /**
     * Get Observation for planner state if available
     */
    //Observation* get_wpc_obs();
    
    void get_laser_obs();

    void get_left_arm_obs(std::vector<Observation*>& obsBuffer, TICK currentTick);
    void get_right_arm_obs(std::vector<Observation*>& obsBuffer, TICK currentTick);
    
    Observation* get_right_end_effector_obs();
    Observation* get_left_end_effector_obs();

    void ConvertArmToEndEffectorFrame(const std_msgs::PR2Arm arm,
				      const unsigned int target_frame,
				      KDL::Frame& f);

    /**
     * Adds a reference
     */
    void addRef();
    /**
     * Deletes a reference.
     */
    bool decRef();

    static ROSNodeId s_id;
    ROSNodeId m_id;
    unsigned int m_refCount;

    bool m_initialized;
    PlannerState m_state;

    // Map update paramters (for adding obstacles)
    double laser_maxrange;
    ros::Duration laser_buffer_time;
    std::list<std_msgs::LaserScan> buffered_laser_scans;
    std::list<laser_pts_t> laser_scans;
    double* laser_hitpts;
    size_t laser_hitpts_len, laser_hitpts_size;
    std::vector<std_msgs::LaserScan> laserScans;
    void laserReceived();
    std_msgs::LaserScan laserMsg;
    
    // Vehcile state updates
    void rcs_cb();
    //void localizedOdomReceived();
    
    void leftArmPosReceived();
    void rightArmPosReceived();
    std_msgs::PR2Arm leftArmPosMsg;
    std_msgs::PR2Arm rightArmPosMsg;
    std_msgs::PR2Arm _lastLeftArmGoal;
    std_msgs::PR2Arm _lastRightArmGoal;
    bool _leftArmActive;
    bool _rightArmActive;
    bool _generateFirstObservation;
    bool _rightArmInit;
    bool _leftArmInit;
    TICK _lastActiveLeftArmDispatch;
    TICK _lastActiveRightArmDispatch;

    std_msgs::Planner2DState m_rcs_state;
    std_msgs::Polyline2D polylineMsg;
    std_msgs::Polyline2D pointcloudMsg;
    
    void lock();
    void unlock();
    
    // Message support for vehicle state updates
    pthread_mutex_t m_lock; /*!< Protect access to buffers */


    robot_kinematics::RobotKinematics _pr2Kinematics;
    robot_kinematics::SerialChain* _armSerialChain;
  };
}


#endif
