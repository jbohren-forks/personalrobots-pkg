/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Rob Wheeler
 */

#ifndef DYNAMIC_LOADER_CONTROLLER_H
#define DYNAMIC_LOADER_CONTROLLER_H

#include "mechanism_model/controller.h"
#include <ltdl.h>

namespace controller {

/***************************************************/
/*! \class controller::DynamicLoaderController
    \brief Dynamic Loader

    This class implements a pseudo-controller that can dynamically
    load a package's shared object and instantiate controllers from
    that shared object.

    When the DynamicLoaderController is killed, it shuts down the
    controllers that it started and unloads the shared object.

    Example configuration:
    <pre>
      <controller name="dynamic_loader" type="DynamicLoaderController"
                  package="my_controllers" lib="libmy_controllers">

        <controllers>

          <controller type="MyController" name="my_controller1">
            <joint name="joint_to_control" />
          </controller><br>

          <controller type="MyController" name="my_controller2">
            <joint name="another_joint_to_control" />
          </controller><br>

        </controllers>

      </controller>
    </pre>

    The above example creates an instance of the DynamicLoaderController
    that loads the shared object libmy_controllers.so from the
    my_controllers package.  It then instantiates two controllers,
    my_controller1 and my_controller2, from that shared object.
    When the DynamicLoaderController is killed, it will kill the two
    controllers it started and unload the libmy_controllers.so shared object.

*/
/***************************************************/

class DynamicLoaderController :  public Controller
{
public:
  DynamicLoaderController();
  ~DynamicLoaderController();

   /*!
   * \brief Specifies the package and shared object to load.
   * \param *robot The robot (not used by this controller).
   * \param *config The XML configuration for this controller
   */
  bool initXml(mechanism::RobotState *robot, TiXmlElement *config);

   /*!
   * \brief This function is called in the control loop.  For this
   * pseudo-controller, the update function is a noop.
   */
  void update();

private:
  std::vector<std::string> names_;
  lt_dlhandle handle_;

  void loadLibrary(std::string& xml);
  static void unloadLibrary(std::vector<std::string> names, lt_dlhandle handle);
};

}

#endif
