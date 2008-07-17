#pragma once
/***************************************************/
/*! \namespace CONTROLLER
    \brief The CONTROLLER namespace
    
    \class CONTROLLER::Controller
    \brief A base level controller class.

*/
/***************************************************/

namespace CONTROLLER
{

  enum CONTROLLER_ERROR_CODE
  {
    CONTROLLER_ALL_OK,
    CONTROLLER_JOINT_LIMIT,
    CONTROLLER_TORQUE_LIMIT,
    CONTROLLER_MODE_ERROR, //e.g. Position command given while in CONTROLLER_VELOCITY mode
    CONTROLLER_JOINT_ERROR,
    CONTROLLER_ACTUATOR_DISABLED, 
    CONTROLLER_ACTUATOR_ENABLED,
    CONTROLLER_CMD_SET
  };
  
  enum CONTROLLER_CONTROL_MODE
  {
    CONTROLLER_MODE_SET,
    CONTROLLER_ENABLED,
    CONTROLLER_DISABLED,
    CONTROLLER_TORQUE,
    CONTROLLER_POSITION,
    CONTROLLER_VELOCITY,
    CONTROLLER_AUTOMATIC
  };

  
  class Controller
  {
    public:
      Controller();
      virtual ~Controller();
       
     virtual void Update(void);
     virtual void Init(void);

    private:


  };
}
