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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Nathan Koenig
 * Date: 19 Sept 2007
 * SVN: $Id$
 */
#ifndef PR2_ACTARRAY_HH
#define PR2_ACTARRAY_HH

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <pr2Core/pr2Core.h>

#include <vector>
#include <generic_controllers/pid.h>

namespace gazebo
{
   class HingeJoint;
   class PositionIface;

/// \addtogroup gazebo_controller
   /// \{
   /** \defgroup pr2_actarray pr2_actarray

   \brief Pr2 controller using Gazebo's Actuator array interface.

   \verbatim
   <controller:pr2_actarray name="controller-name">
   <interface:actarray name="iface-name"/>
   </controller:pr2_actarray>
   \endverbatim

   \{
   */

   /// \brief Pr2 actuator array controller
   /// This is a controller that simulates a Pr2 torso
   class Pr2_Actarray : public Controller
   {
         /// Constructor
      public: Pr2_Actarray(Entity *parent );

         /// Destructor
      public: virtual ~Pr2_Actarray();

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
      private: PR2ArrayIface *myIface;

         /// The parent Model
      private: Model *myParent;

      // Gazebo/ODE joints
      private: Joint *joints[GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // we'll declare a pid controller for each hinger/slider/... joint
      private: controller::Pid *pids[GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // number of joints in this array
      private: int num_joints;

      // save last time for dt calc for pid's
      private: double lastTime;

      // explicit damping coefficient for the joint
      private: double dampCoef;

      // get joints names from xml fields
      private: std::string finger_l_name     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: std::string finger_tip_l_name [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: std::string finger_r_name     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: std::string finger_tip_r_name [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // get the joints from parent
      private: HingeJoint *finger_l_joint     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: HingeJoint *finger_tip_l_joint [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: HingeJoint *finger_r_joint     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: HingeJoint *finger_tip_r_joint [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // assign pid for each finger for PD_CONTROL
      private: controller::Pid *finger_l_pids     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: controller::Pid *finger_tip_l_pids [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: controller::Pid *finger_r_pids     [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];
      private: controller::Pid *finger_tip_r_pids [GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // get name of each child, e.g. front_left_caster_steer
      std::string actarrayName[GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

      // get type of each child, only check for special case for grippers for now.  All others ignored. (TODO)
      std::string actarrayType[GAZEBO_PR2ARRAY_MAX_NUM_ACTUATORS];

   };

/** \} */
/// \}


}

#endif

