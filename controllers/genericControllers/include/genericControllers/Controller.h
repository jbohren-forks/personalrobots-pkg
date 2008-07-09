#pragma once
/***************************************************/
/*! \brief A base level controller class.

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
    CONTROLLER_JOINT_ERROR 
  };
  enum CONTROLLER_CONTROL_MODE
  {
    CONTROLLER_ON,
    CONTROLLER_OFF,
    CONTROLLER_TORQUE,
    CONTROLLER_POSITION,
    CONTROLLER_VELOCITY
  };
  class Controller
  {
    public:
      Controller();
      ~Controller();
       
     virtual void Update(void);
     static double ModNPi2Pi(double angle); 
    private:


  };
}
