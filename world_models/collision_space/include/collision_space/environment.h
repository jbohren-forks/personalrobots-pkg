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

#include "collision_space/environment_objects.h"
#include <planning_models/kinematic.h>
#include <planning_models/output.h>
#include <LinearMath/btVector3.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


/** \brief Main namespace */
namespace collision_space
{
    
    /** \brief A class describing an environment for a kinematic
    robot. This is the base (abstract) definition. Different
    implementations are possible. The class is aware of a set of
    obstacles and a robot model. The obstacles are placed in different
    namespaces so they can be added and removed selectively.
    */
    class EnvironmentModel
    {
    public:
	
	/** \brief Definition of a contact point */
	struct Contact
	{
	    /** \brief contact position */
	    btVector3                              pos;     
	    /** \brief normal unit vector at contact */
	    btVector3                              normal;  
	    /** \brief depth (penetration between bodies) */
	    double                                 depth;
	    /** \brief first link involved in contact */
	    planning_models::KinematicModel::Link *link1; 
	    /** \brief if the contact is between two links, this is not NULL */
	    planning_models::KinematicModel::Link *link2; 
	};
	
	EnvironmentModel(void)
	{
	    m_selfCollision = true;
	    m_verbose = false;
	    m_objects = new EnvironmentObjects();
	}
	
	virtual ~EnvironmentModel(void)
	{
	    if (m_objects)
		delete m_objects;
	}

	/**********************************************************************/
	/* Collision Environment Configuration                                */
	/**********************************************************************/
	
	/** \brief Set the status of self collision */
	void setSelfCollision(bool selfCollision);
	
	/** \brief Check if self collision is enabled */
	bool getSelfCollision(void) const;
			
	/** \brief Add a group of links to be checked for self collision */
	virtual void addSelfCollisionGroup(const std::vector<std::string> &links);

	/** \brief Enable/Disable collision checking for specific links. Return the previous value of the state (1 or 0) if succesful; -1 otherwise */
	virtual int setCollisionCheck(const std::string &link, bool state) = 0;

	/** \brief Add a robot model. Ignore robot links if their name is not
	    specified in the string vector. The scale argument can be
	    used to increase or decrease the size of the robot's
	    bodies (multiplicative factor). The padding can be used to
	    increase or decrease the robot's bodies with by an
	    additive term */
	virtual void setRobotModel(const boost::shared_ptr<const planning_models::KinematicModel> &model, const std::vector<std::string> &links, double scale = 1.0, double padding = 0.0);

	/** \brief Get robot scale */
	double getRobotScale(void) const
	{
	    return m_robotScale;
	}

	/** \brief Get robot padding */
	double getRobotPadding(void) const
	{
	    return m_robotPadd;
	}
	
	/** \brief Update the positions of the geometry used in collision detection */
	virtual void updateRobotModel(void) = 0;

	/** \brief Update the set of bodies that are attached to the robot (re-creates them) */
	virtual void updateAttachedBodies(void) = 0;
		
	/** \brief Get the robot model */
	const boost::shared_ptr<const planning_models::KinematicModel>& getRobotModel(void) const
	{
	    return m_robotModel;
	}	
	

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
	/* Collision Bodies                                                   */
	/**********************************************************************/
	
	/** \brief Remove all objects from collision model */
	virtual void clearObjects(void) = 0;
	
	/** \brief Remove objects from a specific namespace in the collision model */
	virtual void clearObjects(const std::string &ns) = 0;
	
	/** \brief Add a static collision object to the map. The user releases ownership of the passed object. */
	virtual void addObject(const std::string &ns, const shapes::StaticShape *shape) = 0;

	/** \brief Add a collision object to the map. The user releases ownership of the passed object.*/
	virtual void addObject(const std::string &ns, const shapes::Shape* shape, const btTransform &pose) = 0;

	/** \brief Add a set of collision objects to the map. The user releases ownership of the passed objects. */
	virtual void addObjects(const std::string &ns, const std::vector<shapes::Shape*> &shapes, const std::vector<btTransform> &poses) = 0;

	/** \brief Get the objects currently contained in the model */
	const EnvironmentObjects* getObjects(void) const;
	
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
	
	/** \brief Clone the environment. */
	virtual EnvironmentModel* clone(void) const = 0;
	
    protected:
        
	/** \brief Mutex used to lock the datastructure */
	boost::mutex                                             m_lock;

	/** \brief List of links (names) from the robot model that are considered for collision checking */
	std::vector<std::string>                                 m_collisionLinks;

	/** \brief Map used internally to find the index of a link that we do collision checking for */
	std::map<std::string, unsigned int>                      m_collisionLinkIndex;

	/** \brief Matrix of booleans indicating whether pairs of links can self collide */
	std::vector< std::vector<bool> >                         m_selfCollisionTest;
	
	/** \brief Flag to indicate whether self collision checking is enabled */
	bool                                                     m_selfCollision;
	
	/** \brief Flag to indicate whether verbose mode is on */
	bool                                                     m_verbose;

	/** \brief Interface to printing information */
	planning_models::msg::Interface                          m_msg;
	
	/** \brief Loaded robot model */	
	boost::shared_ptr<const planning_models::KinematicModel> m_robotModel;

	/** \brief List of objects contained in the environment */
	EnvironmentObjects                                      *m_objects;
	
	/** \brief Scaling used for robot links */
	double                                                   m_robotScale;

	/** \brief Padding used for robot links */
	double                                                   m_robotPadd;	
	
    };
}

#endif

