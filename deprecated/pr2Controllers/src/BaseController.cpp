#include <pr2Controllers/BaseController.h>
#include <pthread.h>
#include <urdf/URDF.h>

// roscpp
#include <ros/node.h>

#include <sys/time.h>

#include <mechanism_model/joint.h>

#include <angles/angles.h>
#include <control_toolbox/filters.h>

#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>
#include <newmat10/newmatap.h>

   #include <iostream>
   #include <iomanip>
#include "rosTF/rosTF.h"
//   #include "newmatio.h"

#define BASE_NUM_JOINTS 12

using namespace controller;
using namespace PR2;
using namespace NEWMAT;
using namespace angles;

static pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

void BaseController::computePointVelocity(double vx, double vy, double vw, double x_offset, double y_offset, double &pvx, double &pvy)
{
  pvx = vx - y_offset*vw;
  pvy = vy + x_offset*vw;
};

BaseController::BaseController()
{
  this->robot = NULL;
  this->name = "baseController";
}

BaseController::BaseController(Robot *r, std::string name)
{
  this->robot = r;
  this->name = name;
}

BaseController::BaseController(Robot *r)
{
  this->robot = r;
  this->name = "baseController";
}


BaseController::BaseController(char *ns)
{
  this->robot = NULL;
  this->name  = "baseController";
}
  
BaseController::~BaseController( )
{
  delete (this->baseJointControllers);
}

controllerErrorCode BaseController::loadXML(std::string filename)
{
  robot_desc::URDF model;
  int exists = 0;

  if(!model.loadFile(filename.c_str()))
    return CONTROLLER_MODE_ERROR;

  const robot_desc::URDF::Map &data = model.getMap();

  std::vector<std::string> types;
  std::vector<std::string> names;
  std::vector<std::string>::iterator iter;

  data.getMapTagFlags(types);

  for(iter = types.begin(); iter != types.end(); iter++){
    if(*iter == "controller"){
      exists = 1;
      break;
    }
  }

  if(!exists)
    return CONTROLLER_MODE_ERROR;

  exists = 0;

  for(iter = names.begin(); iter != names.end(); iter++){
    if(*iter == this->name){
      exists = 1;
      break;
    }
  }
  paramMap = data.getMapTagValues("controller",this->name);   

  loadParam("pGain",pGain);

  printf("BC:: %f\n",pGain);
  loadParam("dGain",dGain);
  loadParam("iGain",iGain);

  loadParam("pGainPos",pGainPos);
  loadParam("dGainPos",dGainPos);
  loadParam("iGainPos",iGainPos);

  loadParam("casterMode",casterMode);
  loadParam("wheelMode",wheelMode);


  loadParam("maxXDot",maxXDot);
  loadParam("maxYDot",maxYDot);
  loadParam("maxYawDot",maxYawDot);

  return CONTROLLER_ALL_OK;
}

void BaseController::init()
{
  xDotCmd = 0;
  yDotCmd = 0;
  yawDotCmd = 0;

  xDotNew = 0;
  yDotNew = 0;
  yawDotNew = 0;

  pGain = 0.1; /**< Proportional gain for speed control */
  iGain = 0.1; /**< Integral gain for speed control */
  dGain = 0; /**< Derivative gain for speed control */
  iMax  = 10; /**< Max integral error term */
  iMin  = -10; /**< Min integral error term */
  //  maxPositiveTorque = 0.75; /**< (in Nm) max current = 0.75 A. Torque constant = 70.4 mNm/A.Max Torque = 70.4*0.75 = 52.8 mNm */
  //maxNegativeTorque = -0.75; /**< max negative torque */
  maxEffort = 0.75; /**< maximum effort */
  minEffort = -0.75;
  pGainPos = 0.1; /**< Proportional gain for position control */
  iGainPos = 0.1; /**< Integral gain for position control */
  dGainPos = 0; /**< Derivative gain for position control */

  lastTime = getTime(); // should be in separate Odometry class?
  base_x = 0;
  base_y = 0;
  base_w = 0;
  base_vx = 0;
  base_vy = 0;
  base_vw = 0;

  initJointControllers();

  //Subscribe to joystick message
  //   (ros::g_node)->subscribe("base_command", baseCommandMessage, &controller::BaseController::receiveBaseCommandMessage, this);
     (ros::g_node)->subscribe("cmd_vel", baseCommandMessage, &controller::BaseController::receiveBaseCommandMessage, this);
   (ros::g_node)->advertise<std_msgs::RobotBase2DOdom>("odom");

   tf = new rosTFServer(*ros::g_node);
   
 }




Matrix commandTest(3,1);
Matrix aTest(16,1);
void BaseController::receiveBaseCommandMessage(){
  maxXDot = maxYDot = maxYawDot = 1; //Until we start reading the xml file for parameters
 
  /*
  double vx = filters::clamp((double)baseCommandMessage.axes[1], -maxXDot, maxXDot);
  double vy = filters::clamp((double)baseCommandMessage.axes[0], -maxYDot, maxYDot);
  double vyaw = filters::clamp((double)baseCommandMessage.axes[2], -maxYawDot, maxYawDot);
  */
  double vx = filters::clamp((double)baseCommandMessage.vx, -maxXDot, maxXDot);
  double vy = filters::clamp((double)baseCommandMessage.vy, -maxYDot, maxYDot);
  double vyaw = filters::clamp((double)baseCommandMessage.vw, -maxYawDot, maxYawDot);

  printf(" receive vx: %f\n", vx);
  commandTest.element(0,0) = vx;
  commandTest.element(1,0) = vy;
  commandTest.element(2,0) = vyaw;


  /* 
  double vx = filters::clamp((double)baseCommandMessage.vx, -maxXDot, maxXDot);
  double vy = filters::clamp((double)baseCommandMessage.vy, -maxYDot, maxYDot);
  double vyaw = filters::clamp((double)baseCommandMessage.vw, -maxYawDot, maxYawDot);
  */
  setVelocity(vx, vy, vyaw);
}

void BaseController::initJointControllers() 
{
  this->baseJointControllers = new JointController[BASE_NUM_JOINTS];

  for(int ii = 0; ii < NUM_CASTERS; ii++){
    baseJointControllers[ii].init(pGainPos, iGainPos, dGainPos, iMax, iMin, ETHERDRIVE_SPEED, getTime(), maxEffort, minEffort, &(robot->joint[ii]));
    baseJointControllers[ii].enableController();
  }
  for(int ii = NUM_CASTERS; ii < BASE_NUM_JOINTS; ii++){
    baseJointControllers[ii].init(pGain, iGain, dGain, iMax, iMin, ETHERDRIVE_SPEED, getTime(), maxEffort, minEffort, &(robot->joint[ii]));
    baseJointControllers[ii].enableController();
  }
}  

double BaseController::getTime()
{
  struct timeval t;
  gettimeofday( &t, 0);
  return (double) (t.tv_usec *1e-6 + t.tv_sec);
}

double ModNPiBy2(double angle)
{
  if (angle < -M_PI/2) 
    angle += M_PI;
  if(angle > M_PI/2)
    angle -= M_PI;
  return angle;
}

void BaseController::update( )
{
  point drivePointVelocityL, drivePointVelocityR;
  double wheelSpeed[NUM_WHEELS];
  point steerPointVelocity[NUM_CASTERS];
  double steerAngle[NUM_CASTERS];
  point newDriveCenterL, newDriveCenterR;
  double errorSteer[NUM_CASTERS];
  double kp_local = 15;
  double cmdVel[NUM_CASTERS];

  if (pthread_mutex_trylock(&dataMutex) == 0){
    xDotCmd = xDotNew;
    yDotCmd = yDotNew;
    yawDotCmd = yawDotNew;
    pthread_mutex_unlock(&dataMutex);
  }
 
  for(int ii=0; ii < NUM_CASTERS; ii++){
    computePointVelocity(xDotCmd,yDotCmd,yawDotCmd,BASE_CASTER_OFFSET[ii].x,BASE_CASTER_OFFSET[ii].y,steerPointVelocity[ii].x,steerPointVelocity[ii].y);
    steerAngle[ii] = atan2(steerPointVelocity[ii].y,steerPointVelocity[ii].x);

    steerAngle[ii] = ModNPiBy2(steerAngle[ii]);//Clean steer Angle
    errorSteer[ii] = robot->joint[3*ii].position - steerAngle[ii];
    cmdVel[ii] = kp_local * errorSteer[ii];
    baseJointControllers[3*ii].setVelCmd(cmdVel[ii]);     
  }

  for(int ii = 0; ii < NUM_CASTERS; ii++){
#ifdef DEBUG
    printf("offset %d: %f, %f, %f, %f\n",ii,CASTER_DRIVE_OFFSET[ii*2].x,CASTER_DRIVE_OFFSET[ii*2].y,CASTER_DRIVE_OFFSET[ii*2+1].x,CASTER_DRIVE_OFFSET[ii*2+1].y);
#endif
    newDriveCenterL = Rot2D(CASTER_DRIVE_OFFSET[ii*2].x,CASTER_DRIVE_OFFSET[ii*2].y,steerAngle[ii]);
    newDriveCenterR = Rot2D(CASTER_DRIVE_OFFSET[ii*2+1].x,CASTER_DRIVE_OFFSET[ii*2+1].y,steerAngle[ii]);

#ifdef DEBUG
    printf("offset:: %f, %f, %f, %f\n",newDriveCenterL.x,newDriveCenterL.y,newDriveCenterR.x,newDriveCenterR.y);
#endif
    newDriveCenterL.x += BASE_CASTER_OFFSET[ii].x;
    newDriveCenterL.y += BASE_CASTER_OFFSET[ii].y;
    newDriveCenterR.x += BASE_CASTER_OFFSET[ii].x;
    newDriveCenterR.y += BASE_CASTER_OFFSET[ii].y;

    computePointVelocity(xDotCmd,yDotCmd,yawDotCmd,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
    computePointVelocity(xDotCmd,yDotCmd,yawDotCmd,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);

#ifdef DEBUG
    printf("CPV1:: %f, %f, %f, %f, %f, %f, %f\n",xDotCmd,yDotCmd,yawDotCmd,newDriveCenterL.x,newDriveCenterL.y,drivePointVelocityL.x,drivePointVelocityL.y);
    printf("CPV2:: %f, %f, %f, %f, %f, %f, %f\n",xDotCmd,yDotCmd,yawDotCmd,newDriveCenterR.x,newDriveCenterR.y,drivePointVelocityR.x,drivePointVelocityR.y);
#endif

    double steerXComponent = cos(robot->joint[ii*3].position);
    double steerYComponent = sin(robot->joint[ii*3].position);
    double dotProdL = steerXComponent*drivePointVelocityL.x + steerYComponent*drivePointVelocityL.y;
    double dotProdR = steerXComponent*drivePointVelocityR.x + steerYComponent*drivePointVelocityR.y;
#ifdef DEBUG
    printf("Actual CASTER:: %d, %f, %f, %f\n",ii,robot->joint[ii*3].position,drivePointVelocityL.x,drivePointVelocityR.x);
#endif
    wheelSpeed[ii*2  ] = (-dotProdL - cmdVel[ii] * CASTER_DRIVE_OFFSET[ii*2].y)/WHEEL_RADIUS;
    wheelSpeed[ii*2+1] = (dotProdR  +cmdVel[ii] * CASTER_DRIVE_OFFSET[ii*2+1].y)/WHEEL_RADIUS;

    baseJointControllers[ii*3+1].setVelCmd(wheelSpeed[ii*2]);
    baseJointControllers[ii*3+2].setVelCmd(wheelSpeed[ii*2+1]);

    #ifdef DEBUG       // send command
    printf("DRIVE::T:: %d, cmdVel:: %f, pos::%f \n", ii, wheelSpeed[ii*2+2], robot->joint[ii*3].position);
      printf("DRIVE::L:: %d, cmdVel:: %f, vel::%f \n",ii,wheelSpeed[ii*2], robot->joint[ii*3+1].velocity);
      printf("DRIVE::R:: %d, cmdVel:: %f, vel::%f \n",ii,wheelSpeed[ii*2+1], robot->joint[ii*3+2].velocity);
    #endif
  }
  for(int ii = 0; ii < BASE_NUM_JOINTS; ii++) 
    baseJointControllers[ii].update();


  static int counter = 0;
  if (counter++ > 250) {
  computeBaseVelocity();
  computeOdometry(getTime());
  //  printf("vx: %03f, vy: %03f, vw: %03f,", base_vx, base_vy, base_vw);


    printf("x: %03f, y: %03f, w: %03f\n",   base_x, base_y, base_w);
    (ros::g_node)->publish("odom", odomMsg);
   
      tf->sendInverseEuler("FRAMEID_ODOM",
			"FRAMEID_ROBOT",
			 odomMsg.pos.x,
			 odomMsg.pos.y,
			 0.0,
			 odomMsg.pos.th,
			 0.0,
			 0.0,
			 odomMsg.header.stamp);
    //rostime?
    //                      ros::Time(odomMsg.header.stamp.sec,
    //odomMsg.header.stamp.nsec));
    counter = 0;
  
  }
}


void BaseController::computeOdometry(double time) {
  double dt;
  dt = double(time-lastTime);
  //  printf("dt: %03f", dt); 
  base_x += Rot2D(base_vx*dt, base_vy*dt, base_w).x;
  base_y += Rot2D(base_vx*dt, base_vy*dt, base_w).y;
  base_w += base_vw*dt;
  lastTime = time;
  
  odomMsg.pos.x = base_x;
  odomMsg.pos.y = base_y;
  odomMsg.pos.th = base_w;
  odomMsg.vel.x = base_vx;
  odomMsg.vel.y = base_vy;
  odomMsg.vel.th = base_vw;
  
}


void BaseController::computeBaseVelocity(){

  Matrix A(2*NUM_WHEELS,1);
  //Matrix B(NUM_WHEELS,1);
  Matrix C(2*NUM_WHEELS,3);
  Matrix D(3,1);
  
  for(int i = 0; i < NUM_CASTERS; i++) {
    A.element(i*4,0) = cos(robot->joint[i*3].position) *WHEEL_RADIUS*((double)-1)*robot->joint[i*3+1].velocity;
    A.element(i*4+1,0) = sin(robot->joint[i*3].position) *WHEEL_RADIUS*((double)-1)*robot->joint[i*3+1].velocity;
    A.element(i*4+2,0) = cos(robot->joint[i*3].position) *WHEEL_RADIUS*robot->joint[i*3+2].velocity;
    A.element(i*4+3,0) = sin(robot->joint[i*3].position)* WHEEL_RADIUS*robot->joint[i*3+2].velocity;      
  }

  /*
  for(int i = 0; i < (NUM_WHEELS + NUM_CASTERS); i++) {
    printf("i: %i pos : %03f vel: %03f\n", i,robot->joint[i].position, robot->joint[i].velocity); 
  }
  */
  for(int i = 0; i < NUM_CASTERS; i++) {
    C.element(i*4, 0) = 1;
    C.element(i*4, 1) = 0;
    C.element(i*4, 2) = -(Rot2D(CASTER_DRIVE_OFFSET[i*2].x,CASTER_DRIVE_OFFSET[i*2].y,robot->joint[i*3].position).y + BASE_CASTER_OFFSET[i].y);
    C.element(i*4+1, 0) = 0;
    C.element(i*4+1, 1) = 1;
    C.element(i*4+1, 2) =  Rot2D(CASTER_DRIVE_OFFSET[i*2].x,CASTER_DRIVE_OFFSET[i*2].y,robot->joint[i*3].position).x + BASE_CASTER_OFFSET[i].x;
    C.element(i*4+2, 0) = 1;
    C.element(i*4+2, 1) = 0;
    C.element(i*4+2, 2) =  -(Rot2D(CASTER_DRIVE_OFFSET[i*2+1].x,CASTER_DRIVE_OFFSET[i*2+1].y,robot->joint[i*3].position).y + BASE_CASTER_OFFSET[i].y);
    C.element(i*4+3, 0) = 0;
    C.element(i*4+3, 1) = 1;
    C.element(i*4+3, 2) =  Rot2D(CASTER_DRIVE_OFFSET[i*2+1].x,CASTER_DRIVE_OFFSET[i*2+1].y,robot->joint[i*3].position).x + BASE_CASTER_OFFSET[i].x;
  }

  D = pseudoInverse(C)*A; 
  /*
  aTest = C*commandTest;
   cout << "A:" << endl;
  cout << A;
    cout << "C :" << endl;
    cout << C<< endl;
    cout << "commandTest: "<< endl;
    cout << commandTest << endl;
    cout << "aTest: "<< endl;
    cout << aTest << endl;
 //   
 //
 */
  base_vx = (double)D.element(0,0);
  base_vy = (double)D.element(1,0);
  base_vw = (double)D.element(2,0);
  //cout << "D :" << endl;  
  //cout << D << endl;
}


Matrix BaseController::pseudoInverse(const Matrix M)
{
  Matrix result;
   //int rows = this->rows();
   //int cols = this->columns();
   // calculate SVD decomposition
  Matrix U,V;
  DiagonalMatrix D;
   SVD(M,D,U,V, true, true);
   Matrix Dinv = D.i();
   result = V * Dinv * U.t();
   return result;
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

void BaseController::loadParam(std::string label, double &param)
{
  if(paramMap.find(label) != paramMap.end()) // if parameter value has been initialized in the xml file, set internal parameter value
    param = atof(paramMap[label].c_str());
}

void BaseController::loadParam(std::string label, int &param)
{
  if(paramMap.find(label) != paramMap.end())
    param = atoi(paramMap[label].c_str());
}

PR2::PR2_ERROR_CODE BaseController::setParam(std::string label,double value)
{
  return PR2::PR2_ALL_OK;
}

PR2::PR2_ERROR_CODE BaseController::setParam(std::string label,std::string value)
{   
  return PR2::PR2_ALL_OK;
}

// const double BaseController::PGain = 0.1; /**< Proportional gain for speed control */
// const double BaseController::IGain = 0.1; /**< Integral gain for speed control */
// const double BaseController::DGain = 0; /**< Derivative gain for speed control */
// const double BaseController::IMax  = 10; /**< Max integral error term */
// const double BaseController::IMin  = -10; /**< Min integral error term */
// const double BaseController::maxPositiveTorque = 0.75; /**< (in Nm) max current = 0.75 A. Torque constant = 70.4 mNm/A.Max Torque = 70.4*0.75 = 52.8 mNm */
// const double BaseController::maxNegativeTorque = -0.75; /**< max negative torque */
// const double BaseController::maxEffort = 0.75; /**< maximum effort */
// const double BaseController::PGain_Pos = 0.1; /**< Proportional gain for position control */
// const double BaseController::IGain_Pos = 0.1; /**< Integral gain for position control */
// const double BaseController::DGain_Pos = 0; /**< Derivative gain for position control */
