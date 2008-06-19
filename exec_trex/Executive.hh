#ifndef H_Executive
#define H_Executive

#include "Agent.hh"
#include "LogManager.hh"
#include "Debug.hh"
#include "ExecDefs.hh"
#include "Observer.hh"

#include <std_msgs/RobotBase2DOdom.h>
#include <std_msgs/LaserScan.h>
#include <pthread.h>

// For GUI debug
#include <std_msgs/Polyline2D.h>

// For transform support
#include <rosTF/rosTF.h>

typedef struct
{
  double* pts;
  size_t pts_num;
  ros::Time ts;
} laser_pts_t;

namespace TREX{

  class Executive : public ros::node
  {
  public:

    static ExecutiveId instance();

    Executive();

    ~Executive();

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
     * Get all observations.
     */
    void get_obs(std::vector<Observation*>& obsBuffer);

    /**
     * test if executive is initialized with inbound messages it requires
     */
    bool isInitialized() const;

    /**
     * Called by RCS adapter to require the executive to wait for an initialization message
     */
    void requireROS();

  private:

    /**
     * Get Observation for vehicle state if available
     */
    Observation* get_vs_obs();

    /**
     * Get Observation for vehicle velocities if available
     */
    Observation* get_vc_obs();

    /**
     * Get Observation for planner state if available
     */
    Observation* get_wpc_obs();

    void get_laser_obs();

    static ExecutiveId s_id;
    ExecutiveId m_id;
    bool m_initialized;
    PlannerState m_state;

    rosTFClient tf;
    
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

    std_msgs::Planner2DState m_rcs_state;
    std_msgs::Polyline2D polylineMsg;
    std_msgs::Polyline2D pointcloudMsg;

    //MsgRobotBase2DOdom m_localizedOdomMsg;

    void lock();
    void unlock();

    // Message support for vehicle state updates
    pthread_mutex_t m_lock; /*!< Protect access to buffers */
  };

}

#endif
