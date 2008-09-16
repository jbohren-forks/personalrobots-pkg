#include "ROSControllerAdapter.hh"
#include "IntervalDomain.hh"
#include "Token.hh"
#include <pr2_msgs/MoveArmState.h>
#include <pr2_msgs/MoveArmGoal.h>

namespace TREX {

  class ArmControllerAdapter: public ROSControllerAdapter<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal> {
  public:

    ArmControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSControllerAdapter<pr2_msgs::MoveArmState, pr2_msgs::MoveArmGoal>(agentName, configData){
    }

    virtual ~ArmControllerAdapter(){}

  protected:

    void fillActiveObservationParameters(ObservationByValue* obs){
      obs->push_back("shoulder_pan", new IntervalDomain(stateMsg.goal.shoulder_pan));
      obs->push_back("shoulder_pitch", new IntervalDomain(stateMsg.goal.shoulder_pitch));
      obs->push_back("upperarm_roll", new IntervalDomain(stateMsg.goal.upperarm_roll));
      obs->push_back("elbow_flex", new IntervalDomain(stateMsg.goal.elbow_flex));
      obs->push_back("forearm_roll", new IntervalDomain(stateMsg.goal.forearm_roll));
      obs->push_back("wrist_flex", new IntervalDomain(stateMsg.goal.wrist_flex));
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
      obs->push_back("shoulder_pan", new IntervalDomain(stateMsg.configuration.shoulder_pan));
      obs->push_back("shoulder_pitch", new IntervalDomain(stateMsg.configuration.shoulder_pitch));
      obs->push_back("upperarm_roll", new IntervalDomain(stateMsg.configuration.upperarm_roll));
      obs->push_back("elbow_flex", new IntervalDomain(stateMsg.configuration.elbow_flex));
      obs->push_back("forearm_roll", new IntervalDomain(stateMsg.configuration.forearm_roll));
      obs->push_back("wrist_flex", new IntervalDomain(stateMsg.configuration.wrist_flex));    
    }

    void fillRequestParameters(pr2_msgs::MoveArmGoal& goalMsg, const TokenId& goalToken){
      const IntervalDomain& shoulder_pan = goalToken->getVariable("shoulder_pan")->lastDomain();
      const IntervalDomain& shoulder_pitch = goalToken->getVariable("shoulder_pitch")->lastDomain();
      const IntervalDomain& upperarm_roll = goalToken->getVariable("upperarm_roll")->lastDomain();
      const IntervalDomain& elbow_flex = goalToken->getVariable("elbow_flex")->lastDomain();
      const IntervalDomain& forearm_roll = goalToken->getVariable("forearm_roll")->lastDomain();
      const IntervalDomain& wrist_flex = goalToken->getVariable("wrist_flex")->lastDomain();

      assertTrue(shoulder_pan.isSingleton() && 
		 shoulder_pitch.isSingleton() && 
		 upperarm_roll.isSingleton() && 
		 elbow_flex.isSingleton() && 
		 forearm_roll.isSingleton() && 
		 wrist_flex.isSingleton() , "Values for dispatch are not bound");

      goalMsg.configuration.shoulder_pan = shoulder_pan.getSingletonValue();
      goalMsg.configuration.shoulder_pitch = shoulder_pitch.getSingletonValue();
      goalMsg.configuration.upperarm_roll = upperarm_roll.getSingletonValue();
      goalMsg.configuration.elbow_flex = elbow_flex.getSingletonValue();
      goalMsg.configuration.forearm_roll = forearm_roll.getSingletonValue();
      goalMsg.configuration.wrist_flex = wrist_flex.getSingletonValue();
    }

  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<ArmControllerAdapter> l_ArmControllerAdapter_Factory("ArmControllerAdapter");
}
