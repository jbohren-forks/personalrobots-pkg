#ifndef __PR2CORE_H__
#define __PR2CORE_H__

// TODO: put enum in some Pr2 accessible file e.g. pr2.h and include it here
#define MAX_JOINT_IDS 256

namespace PR2
{

   enum PR2_JOINT_CONTROL_MODE{ 
      TORQUE_CONTROL, 
      PD_CONTROL, 
      SPEED_CONTROL, 
      MAX_CONTROL_MODES 
   };

   enum PR2_JOINT_TYPE{ 
      PRISMATIC, ROTARY, 
      ROTARY_CONTINUOUS, 
      MAX_JOINT_TYPES 
   };

   enum PR2_JOINT_ID{
      CASTER_FL_STEER   , 
      CASTER_FL_DRIVE_L , 
      CASTER_FL_DRIVE_R ,
      CASTER_FR_STEER   , 
      CASTER_FR_DRIVE_L , 
      CASTER_FR_DRIVE_R ,
      CASTER_RL_STEER   , 
      CASTER_RL_DRIVE_L , 
      CASTER_RL_DRIVE_R ,
      CASTER_RR_STEER   , 
      CASTER_RR_DRIVE_L , 
      CASTER_RR_DRIVE_R ,
      SPINE_ELEVATOR    ,
      ARM_L_PAN         , 
      ARM_L_SHOULDER_PITCH, 
      ARM_L_SHOULDER_ROLL,
      ARM_L_ELBOW_PITCH , 
      ARM_L_ELBOW_ROLL  ,
      ARM_L_WRIST_PITCH , 
      ARM_L_WRIST_ROLL  ,
      ARM_R_PAN         , 
      ARM_R_SHOULDER_PITCH, 
      ARM_R_SHOULDER_ROLL,
      ARM_R_ELBOW_PITCH , 
      ARM_R_ELBOW_ROLL  ,
      ARM_R_WRIST_PITCH , 
      ARM_R_WRIST_ROLL  ,
      ARM_L_GRIPPER     ,
      ARM_R_GRIPPER     ,
      HEAD_YAW          , 
      HEAD_PITCH        ,
      HEAD_LASER_PITCH  ,
      HEAD_PTZ_L_PAN    , 
      HEAD_PTZ_L_TILT   ,
      HEAD_PTZ_R_PAN    , 
      HEAD_PTZ_R_TILT   ,
      MAX_JOINTS   
   };

   enum PR2_SENSOR_ID{
      CAMERA_GLOBAL        ,
      CAMERA_STEREO_LEFT   ,
      CAMERA_STEREO_RIGHT  ,
      CAMERA_HEAD_LEFT     ,
      CAMERA_HEAD_RIGHT    ,
      CAMERA_ARM_LEFT      ,
      CAMERA_ARM_RIGHT     ,
      LASER_HEAD           ,
      LASER_BASE           ,
      TACTILE_FINGER_LEFT  ,
      TACTILE_FINGER_RIGHT ,
      MAX_SENSORS
   };

   enum PR2_BODY_ID{
      CASTER_FL_WHEEL_L , 
      CASTER_FL_WHEEL_R , 
      CASTER_FL_BODY    ,
      CASTER_FR_WHEEL_L , 
      CASTER_FR_WHEEL_R , 
      CASTER_FR_BODY    ,
      CASTER_RL_WHEEL_L , 
      CASTER_RL_WHEEL_R , 
      CASTER_RL_BODY    ,
      CASTER_RR_WHEEL_L , 
      CASTER_RR_WHEEL_R , 
      CASTER_RR_BODY    ,
      BASE              ,
      TORSO             ,
      ARM_L_TURRET      , 
      ARM_L_SHOULDER    ,
      ARM_L_UPPERARM    , 
      ARM_L_ELBOW       ,
      ARM_L_FOREARM     , 
      ARM_L_WRIST       ,
      ARM_L_GRIPPER_TMP , 
      ARM_L_FINGER_1    ,
      ARM_L_FINGER_2    ,
      ARM_R_TURRET      , 
      ARM_R_SHOULDER    ,
      ARM_R_UPPERARM    , 
      ARM_R_ELBOW       ,
      ARM_R_FOREARM     , 
      ARM_R_WRIST       ,
      ARM_R_GRIPPER_TMP , 
      ARM_R_FINGER_1    ,
      ARM_R_FINGER_2    ,
      HEAD_BASE         , 
      LASER_BLOCK       ,
      STEREO_BLOCK      , 
      LASERBLOCK        ,
      MAX_BODY_IDS 
   };

   enum PR2_JOINT_PARAM_ID { 
      P_GAIN,
      I_GAIN,
      D_GAIN,
      I_CLAMP,
      MAX_TORQUE
   };

   typedef struct
   {
         double timestamp;
         double actualPosition;
         double actualSpeed;
         double actualEffectorForce;

         int controlMode;
         int jointType;

         double cmdPosition;
         double cmdSpeed;
         double cmdEffectorForce;

         int cmdEnableMotor;

         double pGain;
         double iGain;
         double dGain;
         double iClamp;
         double saturationTorque;
   } JointData;

   enum PR2_ERROR_CODE { 
      PR2_ERROR,
      PR2_ALL_OK,
      PR2_MAX_ERROR_CODE
   };

   enum PR2_CONTROL_MODE { 
      PR2_CARTESIAN_CONTROL,
      PR2_JOINT_CONTROL,
      PR2_MAX_CONTROL_MODE
   };

   enum PR2_ROBOT_STATE { 
      STATE_INITIALIZED,
      STATE_CALIBRATED,
      STATE_RUNNING,
      STATE_MOVING,
      STATE_STOPPED,
      MAX_ROBOT_STATES
   };

   typedef struct{
      double x;
      double y;
   }point;

   typedef struct{
      double x;
      double y;
      double z;
   }point3;

   // JointStart/JointEnd corresponds to the PR2_MODEL_ID, start and end id for each model
   enum PR2_MODEL_ID                  {PR2_BASE          ,PR2_SPINE      ,PR2_LEFT_ARM     ,PR2_RIGHT_ARM    ,PR2_LEFT_GRIPPER ,PR2_RIGHT_GRIPPER ,HEAD             , MAX_MODELS };
   const int JointStart[MAX_MODELS] = {CASTER_FL_STEER   ,SPINE_ELEVATOR ,ARM_L_PAN        ,ARM_R_PAN        ,ARM_L_GRIPPER    ,ARM_R_GRIPPER     ,HEAD_YAW         };
   const int JointEnd[MAX_MODELS]   = {CASTER_RR_DRIVE_R ,SPINE_ELEVATOR ,ARM_L_WRIST_ROLL ,ARM_R_WRIST_ROLL ,ARM_L_GRIPPER    ,ARM_R_GRIPPER     ,HEAD_LASER_PITCH };

// Geometric description for the base
   const float CASTER_FL_X_OFFSET = 0.25;
   const float CASTER_FL_Y_OFFSET = 0.25;

   const float CASTER_FR_X_OFFSET = 0.25;
   const float CASTER_FR_Y_OFFSET = -0.25;

   const float CASTER_RL_X_OFFSET = -0.25;
   const float CASTER_RL_Y_OFFSET = 0.25;

   const float CASTER_RR_X_OFFSET = -0.25;
   const float CASTER_RR_Y_OFFSET = -0.25;

   const float AXLE_WIDTH = 0.1;
   const float WHEEL_RADIUS = 0.08;

   const int NUM_CASTERS = 4;
   const int NUM_WHEELS = 8;

   const point BASE_CASTER_OFFSET[NUM_CASTERS] = {{0.25, 0.25}, {0.25, -0.25}, {-0.25,0.25}, {-0.25,-0.25}};
   const point CASTER_DRIVE_OFFSET[NUM_WHEELS] = {{0,AXLE_WIDTH}, {0,-AXLE_WIDTH}, {0,AXLE_WIDTH}, {0,-AXLE_WIDTH},{0,AXLE_WIDTH}, {0,-AXLE_WIDTH},{0,AXLE_WIDTH}, {0,-AXLE_WIDTH}};

   const point3 SPINE_ARM_PAN_OFFSET                 = {0       ,0        ,0     };
   const point3 ARM_PAN_SHOULDER_PITCH_OFFSET        = {0.1     ,0        ,0     };
   const point3 ARM_SHOULDER_PITCH_ROLL_OFFSET       = {0       ,0        ,0     };
   const point3 ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET = {0.4     ,0        ,0     };
   const point3 ELBOW_PITCH_ELBOW_ROLL_OFFSET        = {0.09085 ,0        ,0     };
   const point3 ELBOW_ROLL_WRIST_PITCH_OFFSET        = {0.2237  ,0        ,0     };
   const point3 WRIST_PITCH_WRIST_ROLL_OFFSET        = {0       ,0        ,0     };
   const point3 WRIST_ROLL_GRIPPER_OFFSET            = {0       ,0        ,0     };
   const point3 SPINE_RIGHT_ARM_OFFSET               = {0.0     ,  0.15   , 0.68 };
   const point3 SPINE_LEFT_ARM_OFFSET                = {0.0     , -0.15   , 0.68 };
}
#endif
