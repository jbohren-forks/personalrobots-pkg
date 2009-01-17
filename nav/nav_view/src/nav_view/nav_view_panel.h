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

#ifndef NAV_VIEW_NAV_VIEW_PANEL_H
#define NAV_VIEW_NAV_VIEW_PANEL_H

#include "nav_view_panel_generated.h"

#include "std_msgs/ParticleCloud2D.h"
#include "robot_msgs/Planner2DGoal.h"
#include "std_msgs/Polyline2D.h"
#include "std_msgs/Pose2DFloat32.h"
#include "std_srvs/StaticMap.h"

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreRenderOperation.h>

#include <list>

/**

@mainpage

@htmlinclude manifest.html

@b nav_view is a GUI for 2-D navigation.  It can:
 - display a map
 - display a robot's pose
 - display a cloud of particles (e.g., from a localization system)
 - display a path (e.g., from a path planner)
 - display a set of obstacles (and inflated obstacles)
 - send a goal to a planner

<hr>

@section usage Usage
@verbatim
$ nav_view
@endverbatim

@par Example

@verbatim
$ nav_view
@endverbatim

@par GUI controls
Mouse controls:
 - Movement tool (shortcut: m)
  - Left mouse -- pan the view
  - Right mosue -- zoom the view
 - Goal tool (shortcut: g)
  - Left mouse -- click and drag to select position + heading
 - Pose tool (shortcut: p)
  - Left mouse -- click and drag to select initial position + heading estimate

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "particlecloud"/ParticleCloud2D : a set particles from a probabilistic localization system.  Rendered is a set of small arrows.
- @b "gui_path"/Polyline2D : a path from a planner.  Rendered as a dashed line.
- @b "gui_laser"/Polyline2D : re-projected laser scan from a planner.  Rendered as a set of points.
- @b "local_path"/Polyline2D : local path from a planner.  Rendered as a dashed line.
- @b "robot_footprint"/Polyline2D : Box "footprint" around the robot.  Rendered as a dashed line
- @b "raw_obstacles"/Polyline2D : Raw obstacle data.  Rendered as points
- @b "inflated_obstacles"/Polyline2D : Inflated obstacle data. Rendered as points

Publishes to (name / type):
- @b "goal"/Planner2DGoal : goal for planner.  Sent when using the Goal tool
- @b "initialpose"/Pose2DFloat32 : pose to initialize localization system.  Sent when using the Pose tool

<hr>

@section parameters ROS parameters

- None

 **/

namespace ros
{
class Node;
}

namespace ogre_tools
{
class wxOgreRenderWindow;
}

namespace Ogre
{
class Root;
class SceneManager;
class Camera;
class ManualObject;
}

namespace tf
{
class TransformListener;
}

class wxTimer;
class wxTimerEvent;

namespace nav_view
{

class Tool;

class NavViewPanel : public NavViewPanelGenerated
{
public:
  NavViewPanel( wxWindow* parent );
  virtual ~NavViewPanel();

  void setTransient( bool transient );

  void queueRender();

  float getScale() { return scale_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }
  Ogre::Camera* getCamera() { return camera_; }
  Ogre::SceneNode* getRootNode() { return root_node_; }
  ogre_tools::wxOgreRenderWindow* getRenderPanel() { return render_panel_; }
  ros::Node* getROSNode() { return ros_node_; }
  float getMapResolution() { return map_resolution_; }
  int getMapWidth() { return map_width_; }
  int getMapHeight() { return map_height_; }

protected:

  void onRender( wxCommandEvent& event );
  void onUpdate( wxTimerEvent& event );
  virtual void onToolClicked( wxCommandEvent& event );
  virtual void onReloadMap( wxCommandEvent& event );
  virtual void onChar( wxKeyEvent& event );

  void loadMap();
  void clearMap();
  void incomingParticleCloud();
  void incomingGuiPath();
  void incomingLocalPath();
  void incomingRobotFootprint();
  void incomingInflatedObstacles();
  void incomingRawObstacles();
  void incomingGuiLaser();

  void processParticleCloud();
  void processGuiPath();
  void processLocalPath();
  void processRobotFootprint();
  void processInflatedObstacles();
  void processRawObstacles();
  void processGuiLaser();

  void onRenderWindowMouseEvents( wxMouseEvent& event );

  void createRadiusObject();
  void updateRadiusPosition();

  void createObjectFromPolyLine( Ogre::ManualObject*& object, std_msgs::Polyline2D& path, Ogre::RenderOperation::OperationType op, float depth, bool loop );

  void createTransientObject();

  Ogre::Root* ogre_root_;
  Ogre::SceneManager* scene_manager_;
  Ogre::Camera* camera_;
  ogre_tools::wxOgreRenderWindow* render_panel_;

  ros::Node* ros_node_;
  tf::TransformListener* tf_client_;

  float map_resolution_;
  int map_width_;
  int map_height_;
  Ogre::TexturePtr map_texture_;

  std_msgs::ParticleCloud2D cloud_;
  robot_msgs::Planner2DGoal goal_;
  std_msgs::Polyline2D path_line_;
  std_msgs::Polyline2D local_path_;
  std_msgs::Polyline2D robot_footprint_;
  std_msgs::Polyline2D laser_scan_;
  std_msgs::Polyline2D inflated_obstacles_;
  std_msgs::Polyline2D raw_obstacles_;

  Ogre::ManualObject* map_object_;
  Ogre::MaterialPtr map_material_;

  Ogre::ManualObject* cloud_object_;
  Ogre::ManualObject* radius_object_;
  Ogre::ManualObject* path_line_object_;
  Ogre::ManualObject* local_path_object_;
  Ogre::ManualObject* footprint_object_;
  Ogre::ManualObject* inflated_obstacles_object_;
  Ogre::ManualObject* raw_obstacles_object_;
  Ogre::ManualObject* laser_scan_object_;

  Ogre::SceneNode* root_node_;

  // Mouse handling
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  float scale_;

  bool new_cloud_;
  bool new_gui_path_;
  bool new_local_path_;
  bool new_robot_footprint_;
  bool new_inflated_obstacles_;
  bool new_raw_obstacles_;
  bool new_gui_laser_;
  bool new_occ_diff_;

  wxTimer* update_timer_;

  Tool* current_tool_;
};

} // namespace nav_view

#endif
