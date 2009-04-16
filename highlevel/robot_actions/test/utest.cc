#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <robot_actions/action_client.h>
#include <robot_actions/message_adapter.h>
#include <robot_actions/ActionStatus.h>
#include <std_msgs/Float32.h>
#include <robot_actions/RechargeState.h>
#include <gtest/gtest.h>

using namespace robot_actions;
using namespace std_msgs;

/**
 * This is a really simple example of an action implementation which waits for a time
 * specified by the goal message. It aborts if the goal time == 10 seconds.
 */
class MyAction: public robot_actions::Action<Float32, Float32> {
public:

  MyAction(): robot_actions::Action<Float32, Float32>("my_action") {}

  const static int FAIL_IF = 30;

private:

  // This method will run for the number of ms given by the goal.
  virtual ResultStatus execute(const Float32& goal, Float32& feedback) {

    // Variable for accumulated running time
    double count = 0.0;
    ros::Duration d; d.fromSec(0.001);

    while (!isPreemptRequested() && count <= goal.data) {
      count += 1.0;
      feedback.data = count;
      update(feedback);
      d.sleep();
    }

    if (isPreemptRequested()) {
      return PREEMPTED;
    } else if(goal.data >= FAIL_IF){
      return ABORTED;
    }
    else {
      return SUCCESS;
    }
  }
};

/**
 * This is a really trivial example of a callback interface implementation.
 */
class MySimpleContainer {
public:

  void notify(const ActionStatus& status, const Float32&, const Float32& feedback){
    _status = status;
    _value = feedback.data;
  }

  MySimpleContainer(MyAction& action){
    _value = 0;
    action.connect(boost::bind(&MySimpleContainer::notify, this, _1, _2, _3));
    _status.value = robot_actions::ActionStatus::UNDEFINED;
  }

  ActionStatus _status;
  float _value;
};


// Hack to fordce linkage - gtest macro issue
const int8_t robot_actions::ActionStatus::UNDEFINED;
const int8_t robot_actions::ActionStatus::SUCCESS;
const int8_t robot_actions::ActionStatus::PREEMPTED;
const int8_t robot_actions::ActionStatus::ABORTED;
const int8_t robot_actions::ActionStatus::ACTIVE;


/**
 * Test - just exercises compilation and demonstrates use
 */
TEST(robot_actions, action_with_simple_container){
  MyAction a;
  MySimpleContainer c(a);
  Float32 g;
  g.data = 10.0;
  robot_actions::ActionStatus foo;

  // Activate
  a.activate(g);
  ASSERT_EQ(c._status.value, foo.ACTIVE);

  // Sleep for longer than the goal. Should be finished
  ros::Duration d; 
  d.fromSec(1.0);
  d.sleep();
  ASSERT_EQ(c._status.value == foo.SUCCESS, true);
  ASSERT_EQ(c._value > g.data, true);

  // Now activate again, and preempt before goal is reached
  a.activate(g);
  d.fromSec(0.002);
  d.sleep();
  a.preempt();
  d.sleep();
  ASSERT_EQ(c._status.value == foo.PREEMPTED, true);
  ASSERT_EQ(c._value < g.data, true);

  // Message adapter connects an action to a ros message context
  robot_actions::MessageAdapter<Float32, RechargeState, Float32> adapter(a);
  robot_actions::AbstractAdapter& abstract_adapter(adapter); 
  abstract_adapter.initialize();
  abstract_adapter.terminate();
}

/**
 * Test - Run give a number of goals.
 */
TEST(robot_actions, many_goals) {
  MyAction a;
  MySimpleContainer c(a);
  Float32 g;
  robot_actions::ActionStatus foo;
  ros::Duration d;
  for (int i = 1; i <= MyAction::FAIL_IF; i++) {
    g.data = i;
    a.activate(g);
    g.data = 0; // Test if goal copied properly.
    while(c._status.value == foo.ACTIVE) {
      d.fromSec(0.0001);
      d.sleep();
      if (i == 6) {
	a.preempt();
      }
    }

    d.fromSec(0.002);
    if (i == 6) {
      ASSERT_EQ(foo.PREEMPTED == c._status.value, true);
      ASSERT_EQ(i >= c._value, true);
    } else if (i >= MyAction::FAIL_IF){
      ASSERT_EQ(foo.ABORTED == c._status.value, true);
    } else {
      ASSERT_EQ(foo.SUCCESS == c._status.value, true);
      ASSERT_EQ(i < c._value, true);
    }
  }

}

/**
 * Test - Here is an example using an action runner. 
 */
TEST(robot_actions, action_with_action_runner){

  // Now connect actions
  MyAction a;
  Float32 g;
  robot_actions::ActionStatus foo;

  // First allocate it with an update rate of 10 Hz
  robot_actions::ActionRunner runner(10.0);
  runner.connect<Float32, RechargeState, Float32>(a);

  // Now run it.
  runner.run();

  ros::Duration duration(5);
  duration.sleep();
}

/**
 * Test - Here is an example with the action client 
 */
TEST(robot_actions, action_client){

  // Now connect actions
  MyAction action;

  // Now run it.
  robot_actions::ActionRunner runner(10.0);
  runner.connect<Float32, RechargeState, Float32>(action);
  runner.run();

  // Use a client to test the action
  robot_actions::ActionClient<Float32, RechargeState, Float32> client("my_action");
  ros::Duration duration(5);
  duration.sleep();

  Float32 g, f;
  g.data = 5;
  robot_actions::ResultStatus result = client.execute(g, f, ros::Duration(1));
  ASSERT_EQ(result, robot_actions::SUCCESS);

  result = client.execute(g, f, ros::Duration().fromSec(0.0001));
  ASSERT_EQ(result, robot_actions::PREEMPTED);
}

int main(int argc, char** argv){  
  ros::init(argc, argv);
  
  ros::Node node("robot_actions_test");

  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  node.shutdown();

  return result;
}
