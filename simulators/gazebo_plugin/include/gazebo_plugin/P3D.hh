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
#include <pr2Core/pr2Core.h>

namespace gazebo
{
   class PositionIface;

/// \addtogroup gazebo_controller
   /// \{
   /** \defgroup p3D p3D

   \brief P3D controller.

   \verbatim
   <controller:P3D name="controller-name">
   <interface:actarray name="iface-name"/>
   </controller:P3D>
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
      protected: virtual void UpdateChild(UpdateParams &params);

         /// Finalize the controller
                 /// \return 0 on success
      protected: virtual void FiniChild();

         /// The actarray interface
      private: PositionIface *myIface;

         /// The parent Model
      private: Model *myParent;

      private: Body *myBody; //Gazebo/ODE body
   };

/** \} */
/// \}


}

#endif

