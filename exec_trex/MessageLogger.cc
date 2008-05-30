#include "ExecDefs.hh"
#include <iostream>
#include <fstream>

namespace TREX {

  class MessageLogger : public ros::node
  {
  public:

    MessageLogger() : ros::node("msgLogger"), m_log("msgs.log"){
      subscribe("goal", m_goal, &MessageLogger::goal_cb);
      subscribe("state", m_state, &MessageLogger::state_cb);
    }

    // Goal request will start a timer
    void goal_cb(){
      m_log << "Received Goal: " << toString(m_goal) << std::endl;
    }

    // Goal request will start a timer
    void state_cb(){
      m_log << "Received State: " << toString(m_state) << std::endl;
    }

  private:

    MsgPlanner2DState m_state;
    MsgPlanner2DGoal m_goal;
    std::ofstream m_log;
  };
}

/**
 * @brief Handle cleanup on process termination signals.
 */
void signalHandler(int signalNo){
  exit(0);
}

int main(int argc, char **argv)
{

  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);
  signal(SIGQUIT, signalHandler);
  signal(SIGKILL, signalHandler);

  ros::init(argc, argv);
  MessageLogger logger;

  logger.spin();

  logger.shutdown();
  return 0;
}
