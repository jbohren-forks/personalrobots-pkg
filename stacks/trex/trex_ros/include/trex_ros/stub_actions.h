#ifndef TREX_ROS_STUB_ACTIONS_HH
#define TREX_ROS_STUB_ACTIONS_HH

#include <boost/thread.hpp>
#include <cstdlib>
#include <ros/ros.h>
#include <robot_actions/action.h>

namespace trex_ros {

  template <class Goal, class Feedback>
  class StubAction: public robot_actions::Action<Goal, Feedback> {
  public:

    StubAction(const std::string& name): robot_actions::Action<Goal, Feedback>(name), default_status_(robot_actions::SUCCESS) {
    }
    StubAction(const std::string& name, const Feedback& default_feedback, robot_actions::ResultStatus default_status = robot_actions::SUCCESS) : 
      robot_actions::Action<Goal, Feedback>(name), default_feedback_(default_feedback), default_status_(default_status) {
    }

  protected:

    virtual robot_actions::ResultStatus execute(const Goal& goal, Feedback& feedback){
      ROS_DEBUG("Executing %s\n", robot_actions::Action<Goal, Feedback>::getName().c_str());
      feedback = getFeedback();
      return default_status_;
    }

    virtual Feedback getFeedback(){
      return default_feedback_;
    }

  private:
    Feedback default_feedback_;
    robot_actions::ResultStatus default_status_;
  };


  template <class T> class SimpleStubAction: public robot_actions::Action<T,T> {
  public:
    SimpleStubAction(const std::string& name): robot_actions::Action<T, T>(name) {
      _duration.fromSec(1.0);
    }
    SimpleStubAction(const std::string& name, double secs): robot_actions::Action<T, T>(name) {
      _duration.fromSec(secs);
    }

    virtual robot_actions::ResultStatus execute(const T& goal, T& feedback){
      feedback = goal;
      ROS_DEBUG("Executing %s\n", robot_actions::Action<T, T>::getName().c_str());
      _duration.sleep();

      if(robot_actions::Action<T, T>::isPreemptRequested())
	return robot_actions::PREEMPTED;

      return robot_actions::SUCCESS;
    }

  private:

    ros::Duration _duration;
  };

  /**
   * @brief This stub handles publishing state messages at a given rate
   */
  template<class State> class StatePublisher{
  public:
    StatePublisher(const State& state, const std::string& update_topic, double update_rate)
      : _terminated(false), _state(state), _update_topic(update_topic), _update_rate(update_rate), _update_thread(NULL) {

      ROS_ASSERT(_update_rate > 0);

      // Register publisher
      _pub = _node_handle.advertise<State>(_update_topic, 1);

      // Start the update
      _update_thread = new boost::thread(boost::bind(&StatePublisher<State>::updateLoop, this));
    }

    ~StatePublisher(){
      _terminated = true;
      _update_thread->join();
      delete _update_thread;
    }

  protected:
    virtual void update(State& s){}

  private:

    void updateLoop(){
      ros::Duration sleep_time(1/_update_rate);
      while (!_terminated){
	update(_state);
	_pub.publish(_state);
	sleep_time.sleep();
      }
    }

    bool _terminated;
    State _state;
    const std::string _update_topic;
    const double _update_rate;
    boost::thread* _update_thread;
    ros::NodeHandle _node_handle;
    ros::Publisher _pub;
  };
}

#endif // ifndef TREX_ROS_STUB_ACTIONS_HH
