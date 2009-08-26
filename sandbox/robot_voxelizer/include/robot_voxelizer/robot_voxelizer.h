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
/** \Author: Benjamin Cohen  **/

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <LinearMath/btTransform.h>
#include <geometric_shapes/bodies.h>
#include <planning_environment/models/robot_models.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_models/kinematic.h>
#include <mechanism_msgs/MechanismState.h>
#include "visualization_msgs/Marker.h"

class RobotVoxelizer
{
	public:
	
		double resolution_; /** resolution of voxels in meters */
		double padding_; /** padding of bodies in meters */

		/** \brief Default constructor pulls params off server and instantiates the planning environment variables */
		RobotVoxelizer();
		
		/** \brief Destructor. */
		~RobotVoxelizer();
		
		/** \brief Initialize everything. Get list of bodies from collision yaml file. */
		bool init();
		
		/** \brief Transform stored bodies into their current position. */
		void getCurrentBodies();
		
		/** \brief Returns a list of voxels occupied by a body. The list is a 2D vector of doubles. */
		void getVoxelsInBody(const bodies::Body &body_in, std::vector<std::vector<double> > &cells);
		
		/** \brief Returns a list of voxels occupied by a body. The list is a vector of btVector3. */
		void getVoxelsInBody(const bodies::Body &body, std::vector<btVector3> &voxels);
		
		// old function
		void updateSelfCollisionBodies();
		
		void getVoxelsInSelfCollisionBodies(std::vector<btVector3> &voxels);
		
		/** \brief Publish the visualization markers to be displayed in rviz. */
		void updateVisualizations(const std::vector<std::vector<double> > &cells);
		
		/** \brief Publish the visualization markers to be displayed in rviz. */
		void updateVisualizations(const std::vector<btVector3> &voxels);

		void updateSCGBodies(int group, int subgroup);
		
		void getVoxelsInSCG(int group, int subgroup, std::vector<btVector3> &voxels);
				
		void updateBodies(std::vector<std::string> &link_names);
		
	private:
	
		ros::NodeHandle node_;
		ros::Publisher marker_publisher_;
		ros::Subscriber joint_states_subscriber_;
		std::string robot_description_;
		std::string working_frame_;
		bool bVisualize_;
		double inv_resolution_;
		btVector3 origin_;
		
		tf::TransformListener tf_;
		shapes::Shape * shape_;
		planning_models::KinematicModel::Link *link_;
		planning_environment::RobotModels *robot_model_;
		boost::shared_ptr<planning_models::KinematicModel> kmodel_;
		planning_environment::KinematicModelStateMonitor *monitor_;

		bodies::BoundingSphere bounding_sphere_;
		std::vector<btVector3> self_collision_voxels_;
		std::vector<bodies::Body *> bodies_;
		std::vector<std::string> link_names_;
		std::vector<btVector3> occupied_cells_;
		std::vector<std::vector<double> > cells_;
		std::vector< std::pair < std::vector<std::string>, std::vector<std::string> > > scg_link_names_;
		std::vector< std::pair < std::vector<bodies::Body *>, std::vector<bodies::Body *> > > scg_bodies_;
		
		/** \brief Callback function for mechanismState that updates the pose of the bodies. */
		void jointStatesCallback(const mechanism_msgs::JointStatesConstPtr &joint_states);

		/** \brief Convert from world coordinates to voxel grid coordinates. */
		void worldToGrid(btVector3 origin, double wx, double wy, double wz, int &gx, int &gy, int &gz) const {
			gx = (int)((wx - origin.x()) * inv_resolution_);
			gy = (int)((wy - origin.y()) * inv_resolution_);
			gz = (int)((wz - origin.z()) * inv_resolution_);
		}

		/** \brief Convert from voxel grid coordinates to world coordinates. */
		void gridToWorld(btVector3 origin, int gx, int gy, int gz, double &wx, double &wy, double &wz) const {
			wx = gx * resolution_ + origin.x();
			wy = gy * resolution_ + origin.y();
			wz = gz * resolution_ + origin.z();
		}
};

