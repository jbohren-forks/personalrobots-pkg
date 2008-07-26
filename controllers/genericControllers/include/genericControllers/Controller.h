#pragma once
/***************************************************/
/*! \namespace controller
    \brief The controller namespace
    
    \class controller::Controller
    \brief A base level controller class.

*/
/***************************************************/

namespace controller
{
  enum controllerErrorCode
  {
    CONTROLLER_ALL_OK,
    CONTROLLER_JOINT_LIMIT,
    CONTROLLER_TORQUE_LIMIT,
    CONTROLLER_MODE_ERROR, //e.g. Position command given while in CONTROLLER_VELOCITY mode
    CONTROLLER_JOINT_ERROR,
    CONTROLLER_ACTUATOR_DISABLED, 
    CONTROLLER_ACTUATOR_ENABLED,
    CONTROLLER_COMPUTATION_ERROR,
    CONTROLLER_CMD_SET
  };
  
  enum controllerControlMode
  {
    CONTROLLER_MODE_SET,
    CONTROLLER_ENABLED,
    CONTROLLER_DISABLED,
    CONTROLLER_TORQUE,
    CONTROLLER_POSITION,
    CONTROLLER_VELOCITY,
    CONTROLLER_AUTOMATIC,
    ETHERDRIVE_SPEED
  };
  
  class Controller
  {
    public:
    Controller();
    virtual ~Controller();       
    virtual void update(void);
    virtual void init(void);

    private:
  };

  typedef struct{
      double pGain;
      double iGain;
      double dGain;
      double windupMin;
      double windupMax;
  }pidControlParam;
}
