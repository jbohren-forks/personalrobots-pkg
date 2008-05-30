#ifndef H_Executive
#define H_Executive

#include "Agent.hh"
#include "LogManager.hh"
#include "Debug.hh"
#include "ExecDefs.hh"
#include "Observer.hh"
#include <pthread.h>

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
    void publish(const Observation& obs); 

    /**
     * Called by Bottom Level Adapter to dispatch goals. Needs to translate token into suitable message structure
     */
    void dispatch(const TokenId& goal);

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
     * Get Observation for planner state if available
     */
    Observation* get_wpc_obs();

    static ExecutiveId s_id;
    ExecutiveId m_id;
    bool m_initialized;
    PlannerState m_state;

    // Vehcile state updates
    void rcs_cb();
    MsgPlanner2DState m_rcs_obs;

    void lock();
    void unlock();

    // Message support for vehicle state updates
    pthread_mutex_t m_lock; /*!< Protect access to buffers */
  };

}

#endif
