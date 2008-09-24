/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
#ifndef P3D_HH
#define P3D_HH

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>

#include <ros/node.h>
#include <std_msgs/Pose3DStamped.h>
#include <std_msgs/TransformWithRateStamped.h>

namespace gazebo
{
   class PositionIface;

/// \addtogroup gazebo_controller
   /// \{
   /** \defgroup p3D p3D

   \brief P3D controller.

  \verbatim
  <model:physical name="camera_model">
    <controller:P3D name="p3d_base_controller" plugin="libP3D.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <bodyName>base</bodyName>
      <topicName>base_pose_ground_truth</topicName>
      <frameName>map</frameName>
      <xyzOffsets>25.65 25.65 0</xyzOffsets> <!-- initialize odometry for fake localization-->
      <rpyOffsets>0 0 0</rpyOffsets>
      <interface:position name="p3d_base_position"/>
    </controller:P3D>
  </model:phyiscal>
  \endverbatim
   
   \{
   */

   /// \brief P3D controller
   /// This is a controller that simulates a 6 dof position sensor
   class P3D : public Controller
   {
         /// Constructor
      public: P3D(Entity *parent );

         /// Destructor
      public: virtual ~P3D();

         /// Load the controller
              /// \param node XML config node
              /// \return 0 on success
      protected: virtual void LoadChild(XMLConfigNode *node);

         /// Init the controller
                 /// \return 0 on success
      protected: virtual void InitChild();

         /// Update the controller
                 /// \return 0 on success
      protected: virtual void UpdateChild();

         /// Finalize the controller
                 /// \return 0 on success
      protected: virtual void FiniChild();

         /// The actarray interface
      private: PositionIface *myIface;

         /// The parent Model
      private: Model *myParent;

      private: Body *myBody; //Gazebo/ODE body



      // added for ros message
      // pointer to ros node
      private: ros::node *rosnode;

      // ros message
      private: std_msgs::TransformWithRateStamped transformMsg;

      // topic name
      private: std::string topicName;

      // frame transform name, should match link name
      // FIXME: extract link name directly?
      private: std::string frameName;

      // allow specifying constant xyz and rpy offsets
      private: Vector3 xyzOffsets;
      private: Vector3 rpyOffsets;

      // A mutex to lock access to fields that are used in message callbacks
      private: ros::thread::mutex lock;

   };

/** \} */
/// \}


}

#endif

