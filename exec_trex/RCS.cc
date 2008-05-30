#include "ExecDefs.hh"
#include <iostream>

namespace TREX {

  static bool terminated = false;

  class RCS : public ros::node
  {
  public:

    RCS() : ros::node("rcs"), m_state(INACTIVE), m_targetX(0), m_targetY(0), m_timer(0){
      subscribe("rcs_goal", m_rcs_goal, &RCS::rcs_goal_cb);
      advertise<MsgPlanner2DState>("state");

      // Defaults
      m_rcs.pos.x = 0.0;
      m_rcs.pos.y = 0.0;
      m_rcs.pos.th = 0.0;
      m_rcs.done = '1';
      m_rcs.valid = '1';
    }

    // Goal request will start a timer
    void rcs_goal_cb(){
      m_targetX = m_rcs_goal.goal.x;
      m_targetY = m_rcs_goal.goal.y;
      m_timer = 10;
      m_state = ACTIVE;
      std::cout << "Received Goal: " << toString(m_rcs_goal) << std::endl;
    }

    // Post state update
    void speak(){

      if(m_state == ACTIVE){

	// If countdown is at zero, we are at the goal
	if(m_timer == 0){
	  m_rcs.pos.x = m_targetX;
	  m_rcs.pos.y = m_targetY;
	  m_rcs.done = '1';
	  m_state = INACTIVE;
	}
	else {
	  m_rcs.goal.x = m_targetX;
	  m_rcs.goal.y = m_targetY;
	  m_rcs.done = '0';
	  m_timer--;
	}
      }

      std::cout << "Sending: " << toString(m_rcs) << std::endl;
      publish("state", m_rcs);
    }

  private:

    MsgPlanner2DState m_rcs;
    MsgPlanner2DGoal m_rcs_goal;
    PlannerState m_state;
    double m_targetX, m_targetY;
    unsigned int m_timer;
  };
}


/**
 * @brief Handle cleanup on process termination signals.
 */
void signalHandler(int signalNo){
  terminated = true;
  exit(0);
}

int main(int argc, char **argv)
{

  signal(SIGINT,  signalHandler);
  signal(SIGTERM, signalHandler);
  signal(SIGQUIT, signalHandler);
  signal(SIGKILL, signalHandler);

  ros::init(argc, argv);
  RCS rcs;
  
  unsigned int count(0);

  while(rcs.ok() && !terminated){
    rcs.speak();
    sleep(1);
    std::cout << "Sending update:" << count++ << std::endl;
  }

  rcs.shutdown();
  return 0;
}
