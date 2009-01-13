//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of the Willow Garage nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#ifndef KINEMATICS_HH
#define KINEMATICS_HH

#define MAX_NUM_JOINTS 64

#include <iostream>
#include <vector>
#include <newmat10/newmat.h>
#include <newmat10/newmatio.h>

//using namespace std;

namespace kinematics
{
   /*! \fn
   \brief Transform a twist from one coordinate frame to another
   \param g - NEWMAT matrix representing transformation,
   \param xi - twist vector that needs to be transformed,
   \return NEWMAT matrix representing the transformed twist vector. 
   */   
   NEWMAT::Matrix TransformTwist(const NEWMAT::Matrix& g, const NEWMAT::Matrix&  xi);

   /*! \fn
   \brief Return the twist matrix corresponding to a twist vector
   \param xi - input twist vector (as a NEWMAT matrix)
   \return NEWMAT matrix representing the transformed twist vector. 
   */   
   NEWMAT::Matrix TwistVectorToMatrix(const NEWMAT::Matrix& xi);

   /*! \fn
   \brief Convert twist matrix to vector representation
   \param xiHat - input twist in matrix form (as a NEWMAT matrix)
   \return NEWMAT vector representing the transformed twist matrix
   */   
   NEWMAT::Matrix TwistMatrixToVector(const NEWMAT::Matrix& xiHat);

   /*! \fn
   \brief Compute the transform corresponding to the exponential of a twist
   \param xi - input twist vector (as a NEWMAT matrix)
   \param theta - rotation angle/translational distance
   \return NEWMAT matrix corresponding to the matrix exponential
   */   
   NEWMAT::Matrix ExpTwist(NEWMAT::Matrix xi, double theta);

   /*! \fn
   \brief Compute the rotation matrix corresponding to rotation about an axis by a desired angle
   \param omega - axis of rotation
   \param theta - angle of rotation
   \return Output NEWMAT rotation matrix 
   */   
   NEWMAT::Matrix ExpRot(NEWMAT::Matrix omega, double theta);

   /*! \fn
   \brief Get the (3 x 1) axis corresponding to a twist vector
   \param xi - input twist vector (as a NEWMAT matrix)
   \return Output NEWMAT axis vector (3 x 1)
   */   
   NEWMAT::Matrix GetAxis(NEWMAT::Matrix xi);

   /*! \fn
   \brief Get the (3 x 1) speed part of a twist vector
   \param xi - input twist vector (as a NEWMAT matrix)
   \return Output NEWMAT speed vector (3 x 1)
   */   
   NEWMAT::Matrix GetSpeed(NEWMAT::Matrix xi);

   /*! \fn
   \brief Generate the homogeneous transformation matrix corresponding to a translation
   \param p - translation vector (3 x 1)
   \return Output homogeneous transformation matrix corresponding to a translation
   */   
   NEWMAT::Matrix Translate(double p[]);

   /*! \fn
   \brief Generate the homogeneous transformation matrix corresponding to a translation and roll, pitch and yaw euler angles
   \param p - translation vector (3 x 1)
   \param roll - roll angle
   \param pitch - pitch angle
   \param yaw - yaw angle
   \return Output homogeneous transformation matrix corresponding to a translation and roll, pitch and yaw euler angles
   */   
   NEWMAT::Matrix Transform(double p[],double roll, double pitch, double yaw);


   /*! \fn
   \brief Generate roll, pitch and yaw euler angles corresponding to the homogeneous transformation matrix 
   \param matrix_in - homogeneous matrix representation
   \param roll_out - roll angle
   \param pitch_out - pitch angle
   \param yaw_out - yaw angle
   */   
   void MatrixToEuler(NEWMAT::Matrix matrix_in, double &roll, double &pitch, double &yaw, double &roll2, double &pitch2, double &yaw2);


   /*! \fn
   \brief Get the position/translation part of a homogeneous transformation matrix 
   \param p - homogeneous transformation matrix
   \return Output position vector
   */   
   NEWMAT::Matrix GetPosition(const NEWMAT::Matrix& p);

   /*! \fn
   \brief Get the rotation matrix part of a homogeneous transformation matrix
   \param p - homogeneous transformation matrix
   \return Output rotation matrix
   */   
   NEWMAT::Matrix GetRotationMatrix(const NEWMAT::Matrix& p);

   /*! \fn
   \brief Get the skew-symmetric matrix corresponding to a rotation axis
   \param omega - (3 x 1) vector representing a rotation axis
   \return Output skew-symmetric rotation matrix
   */   
   NEWMAT::Matrix GetHatMatrix(NEWMAT::Matrix omega);

   /*! \fn
   \brief Compute the cross product of two vectors
   \param p1 - NEWMAT matrix representation of the first vector
   \param p2 - NEWMAT matrix representation of the second vector
   \return Output skew-symmetric rotation matrix
   */   
   NEWMAT::Matrix cross(NEWMAT::Matrix p1, NEWMAT::Matrix p2);

   /*! \fn
   \brief Compute the cross product of two vectors
   \param p1 - array representation of the first vector
   \param p2 - array representation of the second vector
   \param p3 - array representation of the resultant vector from the cross-product
   */   
   void cross(const double p1[], const double p2[], double p3[]);

   /*! \fn
   \brief Compute the magnitude of a vector
   \param omega - input vector (as a NEWMAT matrix)
   */   
   double GetNorm(NEWMAT::Matrix omega);

   /*! \fn
\brief Solution of the first form of the Paden-Kahan subproblems. This formulation solves for theta in exp(xi theta) p  = q
\param p NEWMAT matrix representation of a point on which the exponential operates
\param q NEWMAT matrix representation of the resultant point after rotation through thets
\param r point on the axis about which the rotation is happening
\param omega direction vector for the axis about which the rotation is happening. The twist vector xi is a combination of this vector and a linear velocity term v. 
\return theta solution to the above expression
   */
   double SubProblem1(NEWMAT::Matrix p, NEWMAT::Matrix q, NEWMAT::Matrix r, NEWMAT::Matrix omega);


   /*! \fn
\brief Solution of the second form of the Paden-Kahan subproblems. This formulation solves for theta in exp(xi1 theta1) exp(xi2 theta2) p  = q
\param p NEWMAT matrix representation of a point on which the exponential operates
\param q NEWMAT matrix representation of the resultant point after rotation through thets
\param r point on the axis about which the rotation is happening
\param omega direction vector for the axis about which the rotation is happening. The twist vectors xi1 and xi2 are a combination of this vector and linear velocity terms v1 and v2.
\param theta[] a 4x1 vector containing the two solutions to this problem.
   */
   void SubProblem2(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix omega1, NEWMAT::Matrix omega2, double theta[]);


/*! \fn 
\brief Print the matrix in a nice form
\param m Matrix to be printed
\param c string representing matrix specification
*/
   void PrintMatrix(NEWMAT::Matrix m, std::string c);


/*! \enum robot_JOINT_TYPE
 * Joint types
*/
   enum PR2_JOINT_TYPE{ 
      PRISMATIC, 
      ROTARY, 
      ROTARY_CONTINUOUS, 
      MAX_JOINT_TYPES 
   };

   class Joint
   {
     public: 
   
/*! \fn 
  Default constructor
*/
      Joint(){};
   
/*! \fn 
  Default destructor
*/
      virtual ~Joint(){};

      int jointId;  /**< joint ID */

      NEWMAT::Matrix linkPose; /**< link pose */

      NEWMAT::Matrix twist; /**< link twist */

      NEWMAT::Matrix axis;

      NEWMAT::Matrix anchor;
   };

   /*! \class 
     \brief The serial robot class allows the construction of a serial manipulator using a series of links and joints
   */
   class SerialRobot
   {

     public:

      /*! \fn 
        \brief Construct a serial robot with n joints
        \param nJoints - number of joints
      */
      SerialRobot(int nJoints);

      SerialRobot();

      /*! \fn 
        \brief Default destructor
      */
      virtual ~SerialRobot();

      /*! \fn 
        \brief Initialize the joint structure of the serial robot
        \param nJoints - number of joints 
      */
      void Initialize(int nJoints);

      Joint *joints; /**< pointer to set of joints in the robot */

      /*! \fn
        \brief Add a new joint to the serial robot,
        \param joint - twist vector corresponding to the joint
      */
      void AddJoint(NEWMAT::Matrix joint, double p[]); 

      /*! \fn
        \brief Add a new joint to the serial robot,
        \param joint - twist vector corresponding to the joint
      */
      void AddJoint(double p[], double axis[], int jointType);

      /*! \fn
        \brief Add a new joint to the serial robot,
        \param joint - twist vector corresponding to the joint
      */
      void AddJoint(NEWMAT::Matrix p, NEWMAT::Matrix axis, string joint_type);

      /*! \fn
        \brief Compute the inverse kinematics for a serial manipulator (NOT IMPLEMENTED YET)
        \param p - homogeneous transformation matrix representing an end-effector pose
        \param jointangle - return array of computed joint angles
      */
      void ComputeIK(NEWMAT::Matrix p, double jointAngles[]);

      /*! \fn
        \brief Get the COG pose for a particular body (NOT IMPLEMENTED YET)
      */
      void GetCOGPose(int id);

      /*! \fn
        \brief Get the pose for a particular link in the serial manipulator
        \param id - body/link ID
        \param jointangle - input array of joint angles
      */
      NEWMAT::Matrix GetPose(int id, double jointAngles[]);

      /*! \fn
        \brief Get the joint twist for a particular joint in the serial manipulator
        \param id - body/link ID
        \return twist vector for the joint
      */
      NEWMAT::Matrix GetTwist(int id);

      /*! \fn
        \brief Get the joint axis for a particular joint in the serial manipulator
        \param id - body/link ID
        \return axis for the joint
      */
      NEWMAT::Matrix GetJointAxis(int id);

      /*! \fn
        \brief Get the location of a point on the joint axis for a particular joint in the serial manipulator IN THE HOME POSITION
        \param id - body/link ID
        \return axis for the joint
      */
      NEWMAT::Matrix GetJointAxisPoint(int id);

      /*! \fn
        \brief Get the exponential corresponding to the joint twist for a particular joint
        \param id - joint id
        \param theta - joint angle
        \return twist vector for the joint
      */
      NEWMAT::Matrix GetJointExponential(int id, double theta);


      /*! \fn
        \brief Compute the forward kinematics given a particular set of joint angles
        \return Homogeneous transformation matrix corresponding to the pose of the end effector
      */
      NEWMAT::Matrix ComputeFK(double jointAngles[]);

      /*! \fn
        \brief Compute the forward kinematics upto a particular link/body given a particular set of joint angles
        \param jointAngles - array of joint angles
        \param bodyId - Id for a body part/link
        \return Homogeneous transformation matrix corresponding to the pose of a link/body
      */
      NEWMAT::Matrix ComputeFK(double jointAngles[], int bodyId);

      /*! \fn
        \brief Compute the manipulator jacobian matrix given a particular set of joint angles 
        \param jointAngles - array of joint angles
        \return Manipulator jacobian matrix
      */
      NEWMAT::Matrix ComputeManipulatorJacobian(double jointAngles[]);

      /*! \fn
        \brief Get the pose of a particular link/body 
        \param angles - array of joint angles
        \return Get the homogeneous transformation matrix corresponding to the pose of a link/body
      */
      NEWMAT::Matrix GetLinkPose(int id, double angles[]);
      
      /*! \fn
        \brief Compute the end effector velocity given a set of joint angles and joint speeds. 
        \param jointAngles - array of joint angles
        \param jointSpeeds - array of joint speeds
        \return end effector velocity corresponding to the given joint angles and joint speeds
      */
      NEWMAT::Matrix ComputeEndEffectorVelocity(double jointAngles[], double jointSpeeds[]);

      /*! \fn
        \brief Return the default/home position of the end-effector. 
        \return Homogeneous matrix representation of the default/home position of the robot
      */
      NEWMAT::Matrix GetHomePosition();

      /*! \fn
        \brief Set the default/home position of the end-effector. 
        \return Homogeneous matrix representation of the default/home position of the robot
      */
      void SetHomePosition(NEWMAT::Matrix g);

    void SetJointLimits(const std::vector<double> &min_joint_limits, const std::vector<double> &max_joint_limits);

    bool CheckJointLimits(const std::vector<double> &joint_values);

    bool CheckJointLimits(const double &joint_value, const int &joint_num);

    std::vector<double> min_joint_limits_;

    std::vector<double> max_joint_limits_;

     protected:
      
      NEWMAT::Matrix homePosition; /**< Homogeneous matrix representation of the default/home position of the robot */ 

     private: 

      int numberJoints; /**< number of joints in the manipulators */

      int jointCount; /**< internal counter */
   };
}

#endif
