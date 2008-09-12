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
      obs->push_back("turretAngle", new IntervalDomain(stateMsg.goal.turretAngle));
      obs->push_back("shoulderLiftAngle", new IntervalDomain(stateMsg.goal.shoulderLiftAngle));
      obs->push_back("upperarmRollAngle", new IntervalDomain(stateMsg.goal.upperarmRollAngle));
      obs->push_back("elbowAngle", new IntervalDomain(stateMsg.goal.elbowAngle));
      obs->push_back("forearmRollAngle", new IntervalDomain(stateMsg.goal.forearmRollAngle));
      obs->push_back("wristPitchAngle", new IntervalDomain(stateMsg.goal.wristPitchAngle));
      obs->push_back("gripperForceCmd", new IntervalDomain(stateMsg.goal.gripperForceCmd));
      obs->push_back("gripperGapCmd", new IntervalDomain(stateMsg.goal.gripperGapCmd));
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){
      obs->push_back("turretAngle", new IntervalDomain(stateMsg.configuration.turretAngle));
      obs->push_back("shoulderLiftAngle", new IntervalDomain(stateMsg.configuration.shoulderLiftAngle));
      obs->push_back("upperarmRollAngle", new IntervalDomain(stateMsg.configuration.upperarmRollAngle));
      obs->push_back("elbowAngle", new IntervalDomain(stateMsg.configuration.elbowAngle));
      obs->push_back("forearmRollAngle", new IntervalDomain(stateMsg.configuration.forearmRollAngle));
      obs->push_back("wristPitchAngle", new IntervalDomain(stateMsg.configuration.wristPitchAngle));
      obs->push_back("gripperForceCmd", new IntervalDomain(stateMsg.configuration.gripperForceCmd));
      obs->push_back("gripperGapCmd", new IntervalDomain(stateMsg.configuration.gripperGapCmd));
    }

    void fillRequestParameters(pr2_msgs::MoveArmGoal& goalMsg, const TokenId& goalToken){
      const IntervalDomain& turretAngle = goalToken->getVariable("turretAngle")->lastDomain();
      const IntervalDomain& shoulderLiftAngle = goalToken->getVariable("shoulderLiftAngle")->lastDomain();
      const IntervalDomain& upperarmRollAngle = goalToken->getVariable("upperarmRollAngle")->lastDomain();
      const IntervalDomain& elbowAngle = goalToken->getVariable("elbowAngle")->lastDomain();
      const IntervalDomain& forearmRollAngle = goalToken->getVariable("forearmRollAngle")->lastDomain();
      const IntervalDomain& wristPitchAngle = goalToken->getVariable("wristPitchAngle")->lastDomain();
      const IntervalDomain& gripperForceCmd = goalToken->getVariable("gripperForceCmd")->lastDomain();
      const IntervalDomain& gripperGapCmd = goalToken->getVariable("gripperGapCmd")->lastDomain();

      assertTrue(turretAngle.isSingleton() && 
		 shoulderLiftAngle.isSingleton() && 
		 upperarmRollAngle.isSingleton() && 
		 elbowAngle.isSingleton() && 
		 forearmRollAngle.isSingleton() && 
		 wristPitchAngle.isSingleton() && 
		 gripperForceCmd.isSingleton() && 
		 gripperGapCmd.isSingleton(), "Values for dispatch are not bound");

      goalMsg.configuration.turretAngle = turretAngle.getSingletonValue();
      goalMsg.configuration.shoulderLiftAngle = shoulderLiftAngle.getSingletonValue();
      goalMsg.configuration.upperarmRollAngle = upperarmRollAngle.getSingletonValue();
      goalMsg.configuration.elbowAngle = elbowAngle.getSingletonValue();
      goalMsg.configuration.forearmRollAngle = forearmRollAngle.getSingletonValue();
      goalMsg.configuration.wristPitchAngle = wristPitchAngle.getSingletonValue();
      goalMsg.configuration.gripperForceCmd = gripperForceCmd.getSingletonValue();
      goalMsg.configuration.gripperGapCmd = gripperGapCmd.getSingletonValue();
    }

  };

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<ArmControllerAdapter> l_ArmControllerAdapter_Factory("ArmControllerAdapter");
}
