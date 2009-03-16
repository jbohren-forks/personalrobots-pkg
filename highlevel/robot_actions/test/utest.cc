#include <robot_actions/action.h>
#include <robot_actions/action_container.h>
#include <robot_actions/action_runner.h>
#include <robot_actions/ActionStatus.h>
#include <robot_actions/RechargeGoal.h>
#include <robot_actions/RechargeState.h>
#include <gtest/gtest.h>

using namespace robot_actions;


/**
 * This is a really simple example of an action implementation which does no real work
 * but invokes the callback methods on the container as appropriate. The option is there for
 * an implementation to do more work before declaring the appropriate state transition
 */
class MyAction: public robot_actions::Action<RechargeGoal, RechargeGoal> {
public:

  virtual void activate(const RechargeGoal& goal){
    getContainer().notifyActivated(f);
  }

  virtual void preempt(){
    getContainer().notifyPreempted(f);
  }

  virtual void done(){
    getContainer().notifyCompleted(f);
  }

  virtual void updateStatus(RechargeGoal& feedback){feedback = f;}

  MyAction(): robot_actions::Action<RechargeGoal, RechargeGoal>("my_action") {}

private:
  robot_actions::ActionStatus _status;

  RechargeGoal f;
};

/**
 * This is a really trivial example of a callback interface implementation.
 */
class MySimpleContainer : public robot_actions::ActionContainer<RechargeGoal> {
public:
  virtual void notifyActivated(const RechargeGoal& feedback_msg){
    _status.value = robot_actions::ActionStatus::ACTIVE;
  }

  virtual void notifyCompleted(const RechargeGoal& feedback_msg){
    _status.value = robot_actions::ActionStatus::SUCCESS;
  }

  virtual void notifyAborted(const RechargeGoal& feedback_msg){
    _status.value = robot_actions::ActionStatus::ABORTED;
  }

  virtual void notifyPreempted(const RechargeGoal& feedback_msg){
    _status.value = robot_actions::ActionStatus::PREEMPTED;
  }

  MySimpleContainer(MyAction& action) {
    _status.value = robot_actions::ActionStatus::UNDEFINED;
    action.connect(this);
  }

  robot_actions::ActionStatus _status;
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
TEST(robot_actions, basic_compilation){
  // To get around a macro induced linker issue
  MyAction a;
  MySimpleContainer c(a);
  RechargeGoal g;
  a.activate(g);
  robot_actions::ActionStatus foo;

  ASSERT_EQ(c._status.value, foo.ACTIVE);
  a.preempt();
  ASSERT_EQ(c._status.value, foo.PREEMPTED);

  // Switch the conection
  robot_actions::ActionRunner<robot_actions::RechargeGoal, robot_actions::RechargeState, robot_actions::RechargeGoal> bar(a);

  bar.initialize();
}


int main(int argc, char** argv){  
  ros::init(argc, argv);
  
  ros::Node node("robot_actions_test");

  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  node.shutdown();

  return result;
}
