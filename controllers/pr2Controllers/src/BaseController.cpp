#include <pr2Controllers/BaseController.h>
#include <pthread.h>

// roscpp
#include <ros/node.h>

using namespace CONTROLLER;

using namespace PR2;

static pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

const double PGain = 2.5; /**< Proportional gain for speed control */

const double IGain = 0; /**< Integral gain for speed control */

const double DGain = 0; /**< Derivative gain for speed control */

const double IMax  = 0; /**< Max integral error term */

const double IMin  = 0; /**< Min integral error term */

const double maxPositiveTorque = 0.0528; /**< (in Nm) max current = 0.75 A. Torque constant = 70.4 mNm/A.Max Torque = 70.4*0.75 = 52.8 mNm */

const double maxNegativeTorque = -0.0528; /**< max negative torque */

const double maxEffort = 0.0528; /**< maximum effort */
      
const double PGain_Pos = 0.5; /**< Proportional gain for position control */

const double IGain_Pos = 0; /**< Integral gain for position control */

const double DGain_Pos = 0.04; /**< Derivative gain for position control */

void ComputePointVelocity(double vx, double vy, double vw, double x_offset, double y_offset, double &pvx, double &pvy)
{
   pvx = vx - y_offset*vw;
   pvy = vy + x_offset*vw;
};

BaseController::BaseController()
{
}

BaseController::BaseController(char *ns)
{
   this->ns = strdup(ns);
}
  
BaseController::~BaseController( )
{
   delete (this->baseJointControllers);
}

void BaseController::Init()
{
   xDotCmd = 0;
   yDotCmd = 0;
   yawDotCmd = 0;

   xDotNew = 0;
   yDotNew = 0;
   yawDotNew = 0;

   InitJointControllers();
}

void BaseController::InitJointControllers() 
{
   this->baseJointControllers = new JointController[BASE_NUM_JOINTS];
   for(int ii = 0; ii < NUM_CASTERS; ii++) 
   {
      baseJointControllers[ii].Init(PGain, IGain, DGain, IMax, IMin, CONTROLLER_POSITION, ns.getTime(), maxPositiveTorque, maxNegativeTorque, maxEffort);
   }
   for(int ii = NUM_CASTERS; ii < BASE_NUM_JOINTS; ii++) 
   {
      baseJointControllers[ii].Init(PGain, IGain, DGain, IMax, IMin, CONTROLLER_VELOCITY, ns.getTime(), maxPositiveTorque, maxNegativeTorque, maxEffort);
   }
}  

void BaseController::Update( )
{
 
   point drivePointVelocityL, drivePointVelocityR;
   double wheelSpeed[NUM_WHEELS];
   point steerPointVelocity[NUM_CASTERS];
   double steerAngle[NUM_CASTERS];
   point newDriveCenterL, newDriveCenterR;

   if (pthread_mutex_trylock(&dataMutex) == 0)
   {
      xDotCmd = xDotNew;
      yDotCmd = yDotNew;
      yawDotCmd = yawDotNew;
   }
 
   for(int ii=0; ii < NUM_CASTERS; ii++)
   {
      ComputePointVelocity(xDotCmd,yDotCmd,yawDotCmd,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
      steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);
      // hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
      // printf("ii: %d, off: (%f, %f), vel: (%f, %f), angle: %f\n",ii,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y,steerAngle[ii]);
      baseJointControllers[ii].SetPosCmd(steerAngle[ii]);     
   }
   for(int ii = 0; ii < NUM_CASTERS; ii++)
   {

      newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2  ],steerAngle[ii]);
      newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1],steerAngle[ii]);

      newDriveCenterL.x += BASE_CASTER_OFFSET[ii].x;
      newDriveCenterL.y += BASE_CASTER_OFFSET[ii].y;
      newDriveCenterR.x += BASE_CASTER_OFFSET[ii].x;
      newDriveCenterR.y += BASE_CASTER_OFFSET[ii].y;

      ComputePointVelocity(xDotCmd,yDotCmd,yawDotCmd,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
      ComputePointVelocity(xDotCmd,yDotCmd,yawDotCmd,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);

      wheelSpeed[ii*2  ] = -GetMagnitude(drivePointVelocityL.x,drivePointVelocityL.y)/WHEEL_RADIUS;
      wheelSpeed[ii*2+1] = -GetMagnitude(drivePointVelocityR.x,drivePointVelocityR.y)/WHEEL_RADIUS;

      baseJointControllers[ii*2+NUM_CASTERS].SetVelCmd(wheelSpeed[ii*2]);
      baseJointControllers[ii*2+NUM_CASTERS+1].SetVelCmd(wheelSpeed[ii*2+1]);
      // send command
      //hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_L+3*ii),wheelSpeed[ii*2  ]);
      //hw.SetJointSpeed((PR2_JOINT_ID) (CASTER_FL_DRIVE_R+3*ii),wheelSpeed[ii*2+1]);
   }
   for(int ii = NUM_CASTERS; ii < BASE_NUM_JOINTS; ii++) 
      baseJointControllers[ii].Update();
}

PR2::PR2_ERROR_CODE BaseController::setCourse(double v , double yaw)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setVelocity(double xDotNew, double yDotNew, double yawDotNew)
{
   pthread_mutex_lock(&dataMutex);
   this->xDotNew = xDotNew;
   this->yDotNew = yDotNew;
   this->yawDotNew = yawDotNew;
   pthread_mutex_unlock(&dataMutex);
   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setTarget(double x,double y, double yaw, double xDot, double yDot, double yawDot)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setTraj(int num_pts, double x[],double y[], double yaw[], double xDot[], double yDot[], double yawDot[])
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setHeading(double yaw)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setForce(double fx, double fy)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setWrench(double yaw)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setParam(string label,double value)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setParam(string label,string value)
{

   return PR2::PR2_ALL_OK;
}
