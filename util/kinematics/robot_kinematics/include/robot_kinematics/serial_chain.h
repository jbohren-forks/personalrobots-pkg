//Software License Agreement (BSD License)
//
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef SERIAL_CHAIN_H
#define SERIAL_CHAIN_H

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/utilities/utility.h>
#include <kdl/chainidsolver_newtoneuler.hpp>

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <urdf/URDF.h>
#include <libTF/Pose3D.h>

double modulus_double(double a, double b);
double angle_within_mod180(double ang);
void angle_within_mod180(KDL::JntArray &q, int nJnts);

namespace robot_kinematics
{
  /* \class
   * \brief The SerialChain class is a wrapper for the serial chain class found in the KDL library. It allows the additional 
   * specification of a transformation from the externally-defined "link" frame to the joint coordinate frame defined by KDL.
   */
  class SerialChain
  {
    public:
     
    SerialChain();

    ~SerialChain(){
        delete[] link_kdl_frame_;
        delete q_IK_result;
        delete q_IK_guess;
/*
        if (joints_)
          delete[] joints_;
        if (joint_names_)
          delete[] joint_names_;
*/
     };
     
    /*! \brief String name for the serial chain, eg : "rightArm"
     */
    std::string name;

    /*! \brief KDL::Chain representation of the serial chain
     */
    KDL::Chain chain;

    /*! \brief A frame representing the transformation from the link frame (specified externally) to the KDL frame 
     * (specified by KDL at the joint axis)
     */
    KDL::Frame *link_kdl_frame_;

    /*! \brief Number of joints in the robot
     */
    int num_joints_;

    /*! \brief Compute the forward kinematics and return as a KDL::frame
     * \param q - input joint angles 
     * \param f - output end effector frame (result of the fwd kinematics)
     * returns True if ok, False if error.
     */
    bool computeFK(const KDL::JntArray &q, KDL::Frame &f);


    /*! \brief Compute the forward kinematics and return as a KDL::frame
     * \param q - input joint angles 
     * \param f - output end effector frame (result of the fwd kinematics)
     * \param link number - 0 is the base frame and will return an identity matrix for the transformation, frame 1 rotates with the 1st joint and so on until frame num_joints;
     * returns True if ok, False if error.
     */
    bool computeFK(const KDL::JntArray &q, KDL::Frame &f, int frame_number);

    /*! \brief Compute the forward kinematics and return as a NEWMAT::Matrix
     * \param q - input joint angles 
     * \param f - output end effector frame (result of the fwd kinematics)
     * returns True if ok, False if error.
     */
    bool computeFK(const KDL::JntArray &q, NEWMAT::Matrix &f);

    /*! \brief Compute the inverse kinematics and return as a KDL::JntArray
     * \param q_init - initial guess for the inverse kinematics solution
     * \param f - end effector frame 
     * \param q_out - final solution to the IK 
     * returns True if ok, False if error.
     */
    bool computeIK(const KDL::JntArray &q_init, const KDL::Frame &f, KDL::JntArray &q_out);

    /*! \brief Compute the inverse kinematics and return as a KDL::JntArray.
     * This uses the stored previous guess for the inverse kinematics as a starting point for the solution
     * \param f - end effector frame 
     * \param q_out - final solution to the IK 
     * returns True if ok, False if error.
     */
    bool computeIK(const KDL::Frame &f, KDL::JntArray &q_out);

    /*! \brief Compute the inverse kinematics
     * This uses the stored previous guess for the inverse kinematics as a starting point for the solution 
     * and stores the result in q_IK_result
     * \param f - end effector frame 
     * returns True if ok, False if error.
     */
    bool computeIK(const KDL::Frame &f);

    /*! \brief Compute the inverse differential kinematics
     * \param q_in - input joint array 
     * \param v_in - input end-effector twist
     * \param qdot_out - output joint velocities
     * returns True if ok, False if error.
     */
    bool computeDKInv(const KDL::JntArray & q_in, const KDL::Twist &v_in, KDL::JntArray & qdot_out);


    /*! \brief Compute the differential kinematics
     * \param q_in - input joint velocities 
     * \param v_in - 
     * returns True if ok, False if error.
     */
    bool computeDK(const KDL::JntArrayVel & q_in, KDL::FrameVel & out);


    /*! \brief Compute the inverse dynamics
     * \param q - input joint array 
     * \param q_dot - input joint velocities
     * \param q_dotdot - input joint accelerations
     * \param torque - output vector of torques required to acheive desired joint trajectory
     * \return True if ok, False if error.
     */
    bool computeInverseDynamics(const KDL::JntArray &q, const KDL::JntArray &q_dot, const KDL::JntArray &q_dotdot, KDL::Vector* torque);

    bool computeGravityTerms(const KDL::JntArray &q, KDL::Vector* torque);

    void computeMassMatrix(const KDL::JntArray &q, KDL::Vector* torque, NEWMAT::Matrix &mass);

    void computeChristoffelSymbols(const KDL::JntArray &q, KDL::Vector* torque, NEWMAT::Matrix &christoffel);

    KDL::JntArray *q_IK_result; //< KDL::JntArray that stores result of IK

    /*! \brief Finalize the chain after all the segments have been added. This must be done AFTER all links have been added using addLink 
     */
    void finalize();

    int getID(std::string name);//< returns an int corresponding to the input name. 

    KDL::JntArray *q_IK_guess; //< is used as the IK guess.

    KDL::ChainFkSolverPos_recursive *forwardKinematics; /*< pointer to the forward kinematics solver for this chain*/

    KDL::ChainIkSolverVel_pinv *differentialKinematicsInv; /*< pointer to the differential kinematics solver for this chain */

    KDL::ChainIkSolverPos_NR *inverseKinematics; /*< pointer to the inverse kinematics sovler for this case */

    KDL::ChainIdSolver_NE *inverseDynamics; /*< pointer to the inverse dynamics solver for this case */

    KDL::ChainFkSolverVel_recursive *differentialKinematics;

    std::map<std::string, int> joint_id_map_;

    private:
/*
    mechanism::Joint * joints_;
*/

  };
}

#endif
