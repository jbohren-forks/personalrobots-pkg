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

#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

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

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <urdf/URDF.h>
#include <robot_kinematics/serial_chain.h>

namespace robot_kinematics
{
  /*! \class
   * \brief RobotKinematics is a class to easily construct serial robot chain descriptions in the KDL library. 
   * It reads an xml file (the URDF - Universal Robot Description Format) and generates a serial chain robot from the description.
   * It returns pointers to different serial chains and allows access to the underlying kinematics solvers provided by KDL
   * In all serial chains defined by RobotKinematics from the URDF, the KDL frame is always positioned on the joint axis with the joint axis along the Z axis.
   * This class simplifies the description of joints whose axes do not lie along coordinate directions in the home position.
   */
  class RobotKinematics
  {
    public:

    /*!
     * \brief Default constructor
     */
    RobotKinematics();

    /*!
     * \brief Default destructor
     */
    ~RobotKinematics();

    /*!
     * \brief load the robot kinematics from an XML string
     *
     * \param xml_content - xml content
     */
    void loadXMLString(std::string xml_content);

    /*!
     * \brief load the robot kinematics from an XML file using the string name of the file
     *
     * \param filename - xml file name
     */
    void loadXML(std::string filename);

    /*!
     * \brief load the robot kinematics from an XML file using a direct c_string representation of the file
     *        returns true if scuccessful, false if fails to load correctly
     *
     * \param filename - xml file name
     */
    bool loadString(const char* model_string);

    /*!
     * \brief load the robot kinematics from a URDF model
     *
     * \param filename - xml file name
     */
    void loadModel(const robot_desc::URDF &model);

    /*!
     * \brief get back a pointer to a serial chain inside the robot kinematics
     * \param name - name of the serial chain, eg. "rightArm", 
     * this name must match the name given to the serial chain in the XML description
     */
    SerialChain* getSerialChain(std::string name) const;

    /*! \fn
     *\brief Solution of the first form of the Paden-Kahan subproblems. This formulation solves for theta in exp(xi1 theta1) exp(xi2 theta2) p  = q
     *\param p NEWMAT matrix representation of a point on which the exponential operates
     *\param q NEWMAT matrix representation of the resultant point after rotation through thets
     *\param r point on the axis about which the rotation is happening
     *\param omega direction vector for the axis about which the rotation is happening. The twist vectors xi1 and xi2 are a combination of this vector and linear velocity terms v1 and v2.
     *\param theta[] a 4x1 vector containing the two solutions to this problem.
     */
    double subProblem1(NEWMAT::Matrix p, NEWMAT::Matrix q, NEWMAT::Matrix r, NEWMAT::Matrix omega);

    /*! \fn
      \brief Solution of the first form of the Paden-Kahan subproblems. This formulation solves for theta in exp(xi1 theta1) exp(xi2 theta2) p  = q
      \param p NEWMAT matrix representation of a point on which the exponential operates
      \param q NEWMAT matrix representation of the resultant point after rotation through thets
      \param r point on the axis about which the rotation is happening
      \param omega direction vector for the axis about which the rotation is happening. The twist vectors xi1 and xi2 are a combination of this vector and linear velocity terms v1 and v2.
      \param theta[] a 4x1 vector containing the two solutions to this problem.
    */
    void subProblem2(NEWMAT::Matrix pin, NEWMAT::Matrix qin, NEWMAT::Matrix rin, NEWMAT::Matrix omega1, NEWMAT::Matrix omega2, double theta[]);

    private:

    int num_chains_; /*< num of chains in the Robot Kinematics class */

    SerialChain *chains_; /*< pointer to array of serial chains inside RobotKinematics */

    /*!
     * \brief Create a serial chain for the robot using a "group" descripition from the URDF. 
     * A group is a series of links forming a SERIAL manipulator
     * Groups in the URDF must be specified with a flag = "kinematic"
     *
     * \param group - pointer to a URDF group. See URDF.h for more information on groups.
     */ 
    void createChain(robot_desc::URDF::Group* group); 

    /*!
     * \brief cross product helper function
     */
    void cross(const double p1[], const double p2[], double p3[]);

    /*! \fn
      \brief Compute the cross product of two vectors
      \param p1 - NEWMAT matrix representation of the first vector
      \param p2 - NEWMAT matrix representation of the second vector
      \return Output skew-symmetric rotation matrix
    */   
    NEWMAT::Matrix cross(NEWMAT::Matrix p1, NEWMAT::Matrix p2);

    /*! 
     * \brief get angle between two vectors 
     */ 
    double getAngleBetweenVectors(double p1[],double p2[]);

    /*!
     * \brief convert a newmat matrix to KDL::frame format
     *
     * \param NEWMAT::Matrix
     * \return KDL::Frame
     */
    KDL::Frame convertNewmatMatrixToKDL(NEWMAT::Matrix m);

    /*!
     * \brief get the transformation from the current KDL joint frame to the next KDL joint frame
     * \param link - pointer to current link
     * \param link_plus_one - pointer to next link
     * \return KDL::frame representation of the required transformation
     */
    KDL::Frame getKDLNextJointFrame(robot_desc::URDF::Link *link, robot_desc::URDF::Link *link_plus_one);

    std::vector<robot_desc::URDF::Group*> groups_; /*< vector of group pointers defined in the XML file */

    /*! 
     * \brief Compute and get the transformation from the XML/Link frame to the KDL joint frame
     * The KDL frame is always positioned on the joint axis with the joint axis along the Z axis.
     * 
     * \param link - pointer to link in URDF model 
     * \param frame - reference to frame containing KDL::frame description of the required transformation
     */ 
    void getKDLJointInXMLFrame(robot_desc::URDF::Link *link, KDL::Frame &frame);

    /*! 
     * \brief Compute and get the transformation from the XML/Link frame to the KDL joint frame as a NEWMAT::Matrix
     * The KDL frame is always positioned on the joint axis with the joint axis along the Z axis.
     * 
     * \param link - pointer to link in URDF model 
     * \param frame - reference to frame containing KDL::frame description of the required transformation
     */ 
    NEWMAT::Matrix getKDLJointInXMLFrame(robot_desc::URDF::Link *link);

    /*!
     * \brief find the next child link that also belongs to a particular group 
     *
     * \param link_current - pointer to current link. This functions examines the children of this link, 
     * determines which one lies in the requested group and returns a pointer to the child link. 
     * It returns null if it does not find a child.
     * \param group - group to which child must belong to be returned
     */
    robot_desc::URDF::Link* findNextLinkInGroup(robot_desc::URDF::Link *link_current, robot_desc::URDF::Group* group);
    

    int chain_counter_; /*< chain count */

    KDL::Inertia getInertiaInKDLFrame(robot_desc::URDF::Link *link_current, KDL::Vector &pos);

    protected:

    std::map<std::string, SerialChain*> serial_chain_map_; /*< map from pointers to the chains to string names for the chains */

  };
}
#endif


