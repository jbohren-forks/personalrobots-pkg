/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#ifndef COLLISION_SPACE_ENVIRONMENT_MODEL_
#define COLLISION_SPACE_ENVIRONMENT_MODEL_

#include <planning_models/kinematic.h>
#include <planning_models/output.h>
#include <LinearMath/btVector3.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


/** Main namespace */
namespace collision_space
{
    /** \brief   
    A class describing an environment for a kinematic robot. This is
    the base (abstract) definition. Different implementations are
    possible. The class is aware of a certain set of fixed
    (addStatic*) obstacles that never change, a set of obstacles that
    can change (removed by clearObstacles()) and a kinematic
    robot model. The class provides functionality for checking whether a
    given robot is in collision. 
    */
    class EnvironmentModel
    {
    public:
	
	/** \brief Definition of a contact point */
	struct Contact
	{
	    btVector3                              pos;     // contact position
	    btVector3                              normal;  // normal unit vector at contact 
	    double                                 depth;   // depth (penetration between bodies)
	    planning_models::KinematicModel::Link *link1;   // first link involved in contact
	    planning_models::KinematicModel::Link *link2;   // if the contact is between two links, this is not NULL
	};
	
	EnvironmentModel(void)
	{
	    m_selfCollision = true;
	    m_verbose = false;
	}
	
	virtual ~EnvironmentModel(void)
	{
	}

	/**********************************************************************/
	/* Collision Environment Configuration                                */
	/**********************************************************************/
	
	/** \brief Set the status of self collision */
	void setSelfCollision(bool selfCollision);
	
	/** \brief Check if self collision is enabled */
	bool getSelfCollision(void) const;
			
	/** \brief Add a group of links to be checked for self collision */
	virtual void addSelfCollisionGroup(std::vector<std::string> &links);

	/** \brief Enable/Disable collision checking for specific links. Return the previous value of the state (1 or 0) if succesful; -1 otherwise */
	virtual int setCollisionCheck(const std::string &link, bool state) = 0;

	/** \brief Add a robot model. Ignore robot links if their name is not
	    specified in the string vector. The scale argument can be
	    used to increase or decrease the size of the robot's
	    bodies (multiplicative factor). The padding can be used to
	    increase or decrease the robot's bodies with by an
	    additive term */
	virtual void addRobotModel(const boost::shared_ptr<planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale = 1.0, double padding = 0.0);

	/** \brief Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(void) = 0;

	/** \brief Update the set of bodies that are attached to the robot (re-creates them) */
	virtual void updateAttachedBodies(void) = 0;
		
	/** \brief Get the robot model */
	boost::shared_ptr<planning_models::KinematicModel> getRobotModel(void) const;
	

	/**********************************************************************/
	/* Collision Checking Routines                                        */
	/**********************************************************************/
	

	/** \brief Check if a model is in collision. Contacts are not computed */
	virtual bool isCollision(void) = 0;
	
	/** \brief Check for self collision. Contacts are not computed */
	virtual bool isSelfCollision(void) = 0;
	
	/** \brief Get the list of contacts (collisions) */
	virtual bool getCollisionContacts(std::vector<Contact> &contacts, unsigned int max_count = 1) = 0;

	
	/**********************************************************************/
	/* Collision Bodies Definition (Dynamic)                              */
	/**********************************************************************/
	
	/** \brief Remove all obstacles from collision model */
	virtual void clearObstacles(void) = 0;
	
	/** \brief Add a point cloud to the collision space */
	virtual void addPointCloud(unsigned int n, const double* points) = 0;

	/**********************************************************************/
	/* Collision Bodies Definition (Static)                               */
	/**********************************************************************/
	
	/** \brief Add a plane to the collision space. Equation it satisfies is a*x+b*y+c*z = d*/
	virtual void addStaticPlane(double a, double b, double c, double d) = 0;

	/**********************************************************************/
	/* Miscellaneous Routines                                             */
	/**********************************************************************/

	/** \brief Provide interface to a lock. Use carefully! */
	void lock(void);
	
	/** \brief Provide interface to a lock. Use carefully! */
	void unlock(void);
	
	/** \brief Enable/disable verbosity */
	void setVerbose(bool verbose);
	
	/** \brief Check the state of verbosity */
	bool getVerbose(void) const;
	
    protected:
        
	boost::mutex                                       m_lock;
	std::vector<std::string>                           m_collisionLinks;
	std::map<std::string, unsigned int>                m_collisionLinkIndex;
	std::vector< std::vector<bool> >                   m_selfCollisionTest;
	
	bool                                               m_selfCollision;
	bool                                               m_verbose;
	planning_models::msg::Interface                    m_msg;
	
	/** \brief List of loaded robot models */	
	boost::shared_ptr<planning_models::KinematicModel> m_robotModel;
	
    };
}

#endif
    
