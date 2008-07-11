#ifndef __PR2CORE_H__
#define __PR2CORE_H__

#define MAX_JOINT_IDS 256

namespace PR2
{

   enum PR2_FRAMEID{
      FRAMEID_CASTER_FL_WHEEL_L = 101 ,  // FIXME: this is a hack until we have a frameid server
      FRAMEID_CASTER_FL_WHEEL_R , 
      FRAMEID_CASTER_FL_BODY    ,
      FRAMEID_CASTER_FR_WHEEL_L , 
      FRAMEID_CASTER_FR_WHEEL_R , 
      FRAMEID_CASTER_FR_BODY    ,
      FRAMEID_CASTER_RL_WHEEL_L , 
      FRAMEID_CASTER_RL_WHEEL_R , 
      FRAMEID_CASTER_RL_BODY    ,
      FRAMEID_CASTER_RR_WHEEL_L , 
      FRAMEID_CASTER_RR_WHEEL_R , 
      FRAMEID_CASTER_RR_BODY    ,
      FRAMEID_BASE              ,
      FRAMEID_TORSO             ,
      FRAMEID_ARM_L_TURRET      , 
      FRAMEID_ARM_L_SHOULDER    ,
      FRAMEID_ARM_L_UPPERARM    , 
      FRAMEID_ARM_L_ELBOW       ,
      FRAMEID_ARM_L_FOREARM     , 
      FRAMEID_ARM_L_WRIST       ,
      FRAMEID_ARM_L_HAND        , 
      FRAMEID_ARM_L_FINGER_1    ,
      FRAMEID_ARM_L_FINGER_2    ,
      FRAMEID_ARM_R_TURRET      , 
      FRAMEID_ARM_R_SHOULDER    ,
      FRAMEID_ARM_R_UPPERARM    , 
      FRAMEID_ARM_R_ELBOW       ,
      FRAMEID_ARM_R_FOREARM     , 
      FRAMEID_ARM_R_WRIST       ,
      FRAMEID_ARM_R_HAND        , 
      FRAMEID_ARM_R_FINGER_1    ,
      FRAMEID_ARM_R_FINGER_2    ,
      FRAMEID_HEAD_PAN_BASE     , 
      FRAMEID_HEAD_TILT_BASE    ,
      FRAMEID_STEREO_BLOCK      , 
      FRAMEID_TILT_LASER_BLOCK  ,
      FRAMEID_BASE_LASER_BLOCK  ,
      MAX_FRAMEIDS 
   };

   enum PR2_JOINT_CONTROL_MODE{ 
      TORQUE_CONTROL, 
      PD_CONTROL, 
      SPEED_CONTROL, 
      SPEED_TORQUE_CONTROL,
      PD_TORQUE_CONTROL,
      MAX_CONTROL_MODES,
      DISABLED 
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
      BASE_6DOF,
      PR2_WORLD,
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
      ARM_L_HAND        , 
      ARM_L_FINGER_1    ,
      ARM_L_FINGER_2    ,
      ARM_R_TURRET      , 
      ARM_R_SHOULDER    ,
      ARM_R_UPPERARM    , 
      ARM_R_ELBOW       ,
      ARM_R_FOREARM     , 
      ARM_R_WRIST       ,
      ARM_R_HAND        , 
      ARM_R_FINGER_1    ,
      ARM_R_FINGER_2    ,
      HEAD_PAN_BASE     , 
      HEAD_TILT_BASE    ,
      STEREO_BLOCK      , 
      TILT_LASER_BLOCK  ,
      BASE_LASER_BLOCK  ,
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

         // gripper specific
         double cmdGap;
         double actualFingerPosition[2];
         double actualFingerPositionRate[2];
         double actualGap;
         int cmdMode; // only use is GAZEBO_PR2GRIPPER_CMD_OPEN and _CLOSE for now, see libpr2HW
   } JointData;

   enum PR2_ERROR_CODE { 
      PR2_ERROR,
      PR2_ALL_OK,
      PR2_MAX_ERROR_CODE
   };

   enum PR2_CONTROL_MODE { 
      PR2_CARTESIAN_CONTROL,
      PR2_JOINT_CONTROL,
      PR2_MAX_CONTROL_MODE,
      PR2_SPEED_TORQUE_CONTROL,
      PR2_CARTESIAN_TORQUE_CONTROL
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


   class PR2State
   {
     public: 

      PR2State()      
      {
         pos.x = 0;
         pos.y = 0;
         pos.z = 0;
         roll = 0;
         pitch = 0;
         yaw = 0;
         spineExtension = 0;
         for(int ii =0; ii < 2; ii++)
         {
            headJntAngles[ii] = 0.0;
            headJntSpeeds[ii] = 0.0;
         }
         for(int jj =0; jj < 7; jj++)
         {
            rightArmJntAngles[jj] = 0;
            rightArmJntSpeeds[jj] = 0;
            leftArmJntAngles[jj] = 0;
            leftArmJntSpeeds[jj] = 0;
         }
      }

      virtual ~PR2State(){}

         point3 pos;

         double roll;

         double pitch;

         double yaw;

         double spineExtension;

         double headJntAngles[2];

         double headJntSpeeds[2];

         double rightArmJntAngles[7];

         double rightArmJntSpeeds[7];

         double leftArmJntAngles[7];

         double leftArmJntSpeeds[7];
   };

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

   const float AXLE_WIDTH = 0.049;
   const float WHEEL_RADIUS = 0.08;

   const int NUM_CASTERS = 4;
   const int NUM_WHEELS = 8;

   const point3 BASE_CASTER_OFFSET[NUM_CASTERS] = {{0.2255, 0.2255,0.08069}, {0.2255, -0.2255, 0.08069}, {-0.2255,0.2255,0.08069}, {-0.2255,-0.2255,0.08069}};
   const point3 BASE_DRIVE_WHEELS_OFFSET[NUM_WHEELS] = {{0.2255, 0.2745,0.08069},{0.2255, 0.1765,0.08069},{0.2255, -0.1765,0.08069},{0.2255, -0.2745,0.08069},{-0.2255, 0.2745,0.08069},{-0.2255, 0.1765,0.08069},{-0.2255, -0.1765,0.08069},{-0.2255, -0.2745,0.08069}};

   const point3 BASE_BODY_OFFSETS[NUM_CASTERS+NUM_WHEELS] = {{0.2255, 0.2255,0.08069},
                                                             {0.2255, 0.2745,0.08069},{0.2255, 0.1765,0.08069},
                                                             {0.2255, -0.2255, 0.08069},
                                                             {0.2255, -0.1765,0.08069},{0.2255, -0.2745,0.08069},
                                                             {-0.2255,0.2255,0.08069},
                                                             {-0.2255, 0.2745,0.08069},{-0.2255, 0.1765,0.08069},
                                                             {-0.2255,-0.2255,0.08069},
                                                             {-0.2255, -0.1765,0.08069},{-0.2255, -0.2745,0.08069}};

   const point3 CASTER_DRIVE_OFFSET[NUM_WHEELS] = {{0,AXLE_WIDTH}, {0,-AXLE_WIDTH}, {0,AXLE_WIDTH}, {0,-AXLE_WIDTH},{0,AXLE_WIDTH}, {0,-AXLE_WIDTH},{0,AXLE_WIDTH}, {0,-AXLE_WIDTH}};

   const point3 SPINE_ARM_PAN_OFFSET                 = {0       ,0        ,0     };
   const point3 BASE_TORSO_OFFSET                    = {-.1829361,        0, 0.40981 }; // FIXME: z
   const point3 TORSO_LEFT_ARM_PAN_OFFSET            = {     0   ,  0.1329361,    0    };
   const point3 TORSO_RIGHT_ARM_PAN_OFFSET           = {     0   , -0.1329361,    0    };
   const point3 ARM_PAN_SHOULDER_PITCH_OFFSET        = {0.1     ,0        ,0.4   }; // FIXME: what is z?
   const point3 ARM_SHOULDER_PITCH_ROLL_OFFSET       = {0       ,0        ,0     };
   const point3 ARM_SHOULDER_ROLL_ELBOW_PITCH_OFFSET = {0.4     ,0        ,0     };
   const point3 ELBOW_PITCH_ELBOW_ROLL_OFFSET        = {0.09085 ,0        ,0     };
   const point3 ELBOW_ROLL_WRIST_PITCH_OFFSET        = {0.2237  ,0        ,0     };
   const point3 WRIST_PITCH_WRIST_ROLL_OFFSET        = {0       ,0        ,0     };
   const point3 WRIST_ROLL_GRIPPER_OFFSET            = {0       ,0        ,0     };
   const point3 SPINE_RIGHT_ARM_OFFSET               = {0.0     ,-0.15    ,0.68  };
   const point3 SPINE_LEFT_ARM_OFFSET                = {0.0     ,0.15     ,0.68  };

   const point3 BASE_LEFT_ARM_OFFSET                 = {-.1829361, 0.1329361, 0.80981 };
   const point3 BASE_RIGHT_ARM_OFFSET                = {-.1829361, -0.1329361, 0.80981 };

   const point3 HEAD_PAN_HEAD_PITCH_OFFSET           = {0,0,0};

   const point3 BASE_HEAD_OFFSET                     = {-.1829361, 0, 0.80981};
}
#endif
