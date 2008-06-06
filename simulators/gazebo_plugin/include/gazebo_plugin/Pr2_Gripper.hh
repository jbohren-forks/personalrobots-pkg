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
 * Desc: Controller for a pr2 gripper
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef PR2_GRIPPER_HH
#define PR2_GRIPPER_HH

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <pr2Core/pr2Core.h>

namespace gazebo
{
  class SliderJoint;
  class PR2GripperIface;

  /// \addtogroup gazebo_controller
  /// \{
  /** \defgroup pr2_gripper pr2_gripper

    \brief Pr 2 Position2D controller.

    This is a controller that simulates a Pr 2 Gripper

    \verbatim
    <controller:pr2_gripper name="controller-name">
      <leftJoint>left-joint-name</leftJoint>
      <rightJoint>right-join-name</rightJoint>
      <interface:position name="iface-name"/>
    </controller:pr2_gripper>
    \endverbatim
    
    \{
  */

  /// \brief Pr 2 DX Position2D controller.
  /// This is a controller that simulates a Pr 2 motion
  class Pr2_Gripper : public Controller
  {
    /// Constructor
    public: Pr2_Gripper(Entity *parent);

    /// Destructor
    public: virtual ~Pr2_Gripper();

    /// Load the controller
    /// \param node XML config node
    /// \return 0 on success
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// Init the controller
    /// \return 0 on success
    protected: virtual void InitChild();

    /// Update the controller
    /// \return 0 on success
    protected: virtual void UpdateChild(UpdateParams &params);

    /// Finalize the controller
    /// \return 0 on success
    protected: virtual void FiniChild();

    /// The Position interface
    private: PR2GripperIface *myIface;

    /// The parent Model
    private: Model *myParent;

    private: SliderJoint *joints[2];

  };

  /** \} */
  /// \}

}

#endif

