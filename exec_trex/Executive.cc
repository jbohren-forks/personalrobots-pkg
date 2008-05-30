///////////////////////////////////////////////////////////////////////////////
// The trex demo node is a simple encapsulation of a Teleo-Reactive Executive 
//
// Copyright (C) 2008, Willow Garage
//////////////////////////////////////////////////////////////////////////////

#include "Nddl.hh"
#include "Executive.hh"
#include "RCSAdapter.hh"
#include "Token.hh"
#include <signal.h>

TiXmlElement* root = NULL;
Clock* agentClock = NULL;
std::ofstream dbgFile("Debug.log");

namespace TREX {

  /**
   * Executive class implementation.
   */

  /**
   * @brief Singleton accessor
   */
  ExecutiveId Executive::instance(){
    return s_id;
  }

  /**
   * @brief Sets up publish and subscribe topics for this node to integrate it
   * at the top to publish Execution Status updates and at the bottom to dispatch 
   * commands to the RCS and receive updates
   */
  Executive::Executive() : ros::node("trex"), m_id(this), m_initialized(1), m_state(UNDEFINED)
  {
    s_id = m_id;
    pthread_mutex_init(&m_lock, NULL);
    m_rcs_obs.pos.x = 0.0;
    m_rcs_obs.pos.y = 0.0;
    m_rcs_obs.pos.th = 0.0;
    m_rcs_obs.done = 1;
    m_rcs_obs.valid = 1;
  }

  Executive::~Executive(){
    m_id.remove();
    s_id = ExecutiveId::noId();
  }

  /**
   * @brief Nothing to do here. The msg object will already have been updated and that is what we use
   * directly in synchronization
   */
  void Executive::rcs_cb(){
    m_initialized = true;
    std::cout << "Received Update:" << toString(m_rcs_obs) << std::endl;
  }

  /**
   * @brief Called when an update is received in the Monitor during synchronization
   */
  void Executive::publish(const Observation& obs)
  {
    debugMsg("Executive:publish", obs.toString());
    MsgToken msg;
    msg.predicate = obs.toString();
    ros::node::publish("navigator", msg);
  }

  /**
   * @brief Dispatch a token
   */
  void Executive::dispatch(const TokenId& goal){
    static const LabelStr ACTIVE("WaypointController.Active");
    static const LabelStr X2("x");
    static const LabelStr Y2("y");

    if(goal->getPredicateName() == ACTIVE){
      MsgPlanner2DGoal msg;
      const IntervalDomain& x = goal->getVariable(X2)->lastDomain();
      const IntervalDomain& y = goal->getVariable(Y2)->lastDomain();

      checkError(x.isSingleton(), x.toString());
      checkError(y.isSingleton(), y.toString());

      // Fill outbound msg. Don't care about orientation
      msg.goal.x = x.getSingletonValue();
      msg.goal.y = y.getSingletonValue();
      msg.enable = true;
      std::cout << "Dispatching Goal: " <<  goal->toString() << " AS " << toString(msg) << std::endl;
      ros::node::publish("goal", msg);
    }
  }

  void Executive::get_obs(std::vector<Observation*>& buff){
    Observation* vs = get_vs_obs();
    if(vs != NULL)
      buff.push_back(vs);

    Observation* wpc = get_wpc_obs();
    if(wpc != NULL)
      buff.push_back(wpc);
  }

  Observation* Executive::get_vs_obs(){
    lock();
    ObservationByValue* obs = NULL;
    obs = new ObservationByValue("vs", "VehicleState.Holds");
    obs->push_back("x", new IntervalDomain(m_rcs_obs.pos.x));
    obs->push_back("y", new IntervalDomain(m_rcs_obs.pos.y));
    unlock();
    return obs;
  }

  Observation* Executive::get_wpc_obs(){
    lock();
    ObservationByValue* obs = NULL;
    // Determine current obseved state
    PlannerState observedState = INACTIVE;

    if(m_rcs_obs.done == 0)
      observedState = ACTIVE;

    // If there is a state change we have work to do
    if(m_state != observedState){

      m_state = observedState;
      if(m_state == INACTIVE){
	obs = new ObservationByValue("wpc", "WaypointController.Inactive");
      }
      else {
	obs = new ObservationByValue("wpc", "WaypointController.Active");
	obs->push_back("x", new IntervalDomain(m_rcs_obs.goal.x));
	obs->push_back("y", new IntervalDomain(m_rcs_obs.goal.y));
      }

      std::cout << "Received Observation:" << obs->toString() << std::endl;
    }

    unlock();
    return obs;
  }

  bool Executive::isInitialized() const {
    return m_initialized;
  }

  void Executive::lock(){
    pthread_mutex_lock(&m_lock);
  }

  void Executive::unlock(){
    pthread_mutex_unlock(&m_lock);
  }

  void Executive::requireROS(){
    advertise<MsgToken>("navigator");
    advertise<MsgPlanner2DGoal>("goal");
    subscribe("state", m_rcs_obs, &Executive::rcs_cb);
    m_initialized = false;
  }
}

/**
 * @brief Handle cleanup on process termination signals.
 */
void signalHandler(int signalNo){
  std::cout << "Handling signal..." << std::endl;
  exit(0);
}

/**
 * @brief Handle cleanup on exit
 */
void cleanup(){
  std::cout << "Cleaning up..." << std::endl;

  // Terminate the agent
  Agent::terminate();

  Clock::sleep(3);

  Agent::cleanupLog();
}

ExecutiveId Executive::s_id;

int main(int argc, char **argv)
{
  signal(SIGINT,  &signalHandler);
  signal(SIGTERM, &signalHandler);
  signal(SIGQUIT, &signalHandler);
  signal(SIGKILL, &signalHandler);
  atexit(&cleanup);

  if (argc != 2) {
    std::cerr << "Invalid argument list: Usage: ros_exec configFile" << std::endl;
    return -1;
  }

  ros::init(argc, argv);

  LogManager::instance();

  initROSExecutive();

  Executive t;

  NDDL::loadSchema();

  char * configFile = argv[1];
  root = LogManager::initXml( configFile );
  DebugMessage::setStream(dbgFile);

  // Allocate a real time clock with 1 second per tick
  agentClock = new RealTimeClock(1.0);

  // Allocate the agent
  debugMsg("Executive", "Initializing the agent");
  Agent::initialize(*root, *agentClock);

  // Wait till we get a message before starting the agent
  while(!t.isInitialized() && t.ok()){
    std::cout << "Waiting.." << std::endl;
    sleep(1);
  }

  std::cout << "Starting TREX" << std::endl;
  try{
    debugMsg("ALWAYS", "Executing the agent");
    Agent::instance()->run();
  }
  catch(void*){
    debugMsg("Executive", "Caught unexpected exception.");
  }

  t.shutdown();
  return 0;
}

