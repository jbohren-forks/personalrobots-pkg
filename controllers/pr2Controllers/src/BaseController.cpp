#include <pr2Controllers/BaseController.h>
#include <pthread.h>

// roscpp
//#include <ros/node.h>

#include <sys/time.h>

#include <robot_model/joint.h>

using namespace CONTROLLER;

using namespace PR2;

static pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

const double BaseController::PGain = 2.5; /**< Proportional gain for speed control */

const double BaseController::IGain = 0; /**< Integral gain for speed control */

const double BaseController::DGain = 0; /**< Derivative gain for speed control */

const double BaseController::IMax  = 0; /**< Max integral error term */

const double BaseController::IMin  = 0; /**< Min integral error term */

const double BaseController::maxPositiveTorque = 0.0528; /**< (in Nm) max current = 0.75 A. Torque constant = 70.4 mNm/A.Max Torque = 70.4*0.75 = 52.8 mNm */

const double BaseController::maxNegativeTorque = -0.0528; /**< max negative torque */

const double BaseController::maxEffort = 0.0528; /**< maximum effort */
      
const double BaseController::PGain_Pos = 0.5; /**< Proportional gain for position control */

const double BaseController::IGain_Pos = 0; /**< Integral gain for position control */

const double BaseController::DGain_Pos = 0.04; /**< Derivative gain for position control */

void ComputePointVelocity(double vx, double vy, double vw, double x_offset, double y_offset, double &pvx, double &pvy)
{
   pvx = vx - y_offset*vw;
   pvy = vy + x_offset*vw;
};

BaseController::BaseController()
{
}

BaseController::BaseController(Robot *r)
{
   this->robot = r;
}

BaseController::BaseController(char *ns)
{
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
      baseJointControllers[ii].Init(PGain_Pos, IGain_Pos, DGain_Pos, IMax, IMin, CONTROLLER_POSITION, GetTime(), maxPositiveTorque, maxNegativeTorque, maxEffort, &(robot->joint[ii]));
  
   for(int ii = NUM_CASTERS; ii < BASE_NUM_JOINTS; ii++)
      baseJointControllers[ii].Init(PGain, IGain, DGain, IMax, IMin, CONTROLLER_VELOCITY, GetTime(), maxPositiveTorque, maxNegativeTorque, maxEffort,&(robot->joint[ii]));
}  

double BaseController::GetTime()
{
   struct timeval t;
   gettimeofday( &t, 0);
   return (double) (t.tv_usec *1e-6 + t.tv_sec);
}


void BaseController::Update( )
{
   point drivePointVelocityL, drivePointVelocityR;
   double wheelSpeed[NUM_WHEELS];
   point steerPointVelocity[NUM_CASTERS];
   double steerAngle[NUM_CASTERS];
   point newDriveCenterL, newDriveCenterR;

   if (pthread_mutex_trylock(&dataMutex) == 0){
      xDotCmd = xDotNew;
      yDotCmd = yDotNew;
      yawDotCmd = yawDotNew;
   }
 
   for(int ii=0; ii < NUM_CASTERS; ii++){
      ComputePointVelocity(xDotCmd,yDotCmd,yawDotCmd,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
      steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);
      // hw.SetJointServoCmd((PR2_JOINT_ID) (CASTER_FL_STEER+3*ii),steerAngle[ii],0);
      // printf("ii: %d, off: (%f, %f), vel: (%f, %f), angle: %f\n",ii,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y,steerAngle[ii]);
      baseJointControllers[ii].SetPosCmd(steerAngle[ii]);     
   }

   for(int ii = 0; ii < NUM_CASTERS; ii++){
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

PR2::PR2_ERROR_CODE BaseController::setParam(std::string label,double value)
{

   return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setParam(std::string label,std::string value)
{

   return PR2::PR2_ALL_OK;
}
