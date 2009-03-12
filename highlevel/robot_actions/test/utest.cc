#include <robot_actions/action.h>
#include <gtest/gtest.h>

class MyGoal {
public:
  int a, b;
};

class MyFeedback {
public:
  int c, d;
};

class MyAction: public robot_actions::Action<MyGoal, MyFeedback> {
public:

  virtual void activate(const MyGoal& goal){
    _status.value = robot_actions::ActionStatus::ACTIVE;
  }

  virtual void preempt(){
    _status.value = robot_actions::ActionStatus::PREEMPTED;
  }

  virtual robot_actions::ActionStatus checkStatus(MyFeedback& feedback){
    return _status;
  }

  MyAction(){
    _status.value = robot_actions::ActionStatus::UNDEFINED;
  }

private:
  robot_actions::ActionStatus _status;
};

// This hack is required to get this to link. I have a ticket open on this
const int8_t robot_actions::ActionStatus::UNDEFINED;
const int8_t robot_actions::ActionStatus::SUCCESS;
const int8_t robot_actions::ActionStatus::PREEMPTED;
const int8_t robot_actions::ActionStatus::ABORTED;
const int8_t robot_actions::ActionStatus::ACTIVE;

/**
 * Test - just exercises compilation and demonstrates use
 */
TEST(robot_actions, compilation){
  MyAction a;
  MyGoal g;
  MyFeedback f;
  a.activate(g);
  ASSERT_EQ(a.checkStatus(f).value, robot_actions::ActionStatus::ACTIVE);
  a.preempt();
  ASSERT_EQ(a.checkStatus(f).value, robot_actions::ActionStatus::PREEMPTED);
}

int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
