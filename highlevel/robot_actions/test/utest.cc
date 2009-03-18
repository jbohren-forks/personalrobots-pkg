#include <robot_actions/action.h>
#include <robot_actions/action_runner.h>
#include <robot_actions/message_adapter.h>
#include <robot_actions/ActionStatus.h>
#include <robot_actions/RechargeGoal.h>
#include <robot_actions/RechargeState.h>
#include <gtest/gtest.h>

using namespace robot_actions;


/**
 * This is a really simple example of an action implementation which does no real work
 * but invokes the callback methods on the container as appropriate. The option is there for
 * an implementation to do more work before declaring the appropriate state transition.
 */
class MyAction: public robot_actions::Action<RechargeGoal, RechargeGoal> {
public:

  MyAction(): robot_actions::Action<RechargeGoal, RechargeGoal>("my_action") {}

private:

  /** Super class methods **/
  virtual void handleActivate(const RechargeGoal& goal){
    notifyActivated(f);
  }

  virtual void handlePreempt(){
    notifyPreempted(f);
  }

  virtual void updateStatus(RechargeGoal& feedback){
    feedback = f;
  }

  RechargeGoal f;
};

/**
 * This is a really trivial example of a callback interface implementation.
 */
class MySimpleContainer {
public:

  void notify(const ActionStatus& status, const RechargeGoal&, const RechargeGoal&){
    _status = status;
  }

  MySimpleContainer(MyAction& action){
    action.connect(boost::bind(&MySimpleContainer::notify, this, _1, _2, _3));
    _status.value = robot_actions::ActionStatus::UNDEFINED;
  }

  ActionStatus _status;
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

  robot_actions::ActionRunner<RechargeGoal, RechargeState, RechargeGoal> runner(a);
  runner.initialize();
  runner.terminate();

  robot_actions::MessageAdapter<RechargeGoal, RechargeState, RechargeGoal> adapter(a);
  adapter.initialize();
  adapter.update();
  adapter.terminate();
}


int main(int argc, char** argv){  
  ros::init(argc, argv);
  
  ros::Node node("robot_actions_test");

  testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  node.shutdown();

  return result;
}
