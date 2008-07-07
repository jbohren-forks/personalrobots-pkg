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
    CONTROLLER_TORQUE_LIMIT
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

    private:
  };
}
