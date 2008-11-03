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

#include "nav_view_panel.h"

#include "ogre_tools/wx_ogre_render_window.h"

#include "ros/common.h"
#include "ros/node.h"
#include <tf/transform_listener.h>

#include <Ogre.h>

#include "ogre_tools/axes.h"

#include "tools.h"

#define MAP_DEPTH (-1.0f)
#define RADIUS_DEPTH (-0.99f)
#define CLOUD_DEPTH (-0.98f)
#define PATH_LINE_DEPTH (-0.97f)
#define LOCAL_PATH_DEPTH (-0.96f)
#define FOOTPRINT_DEPTH (-0.95f)
#define INFLATED_OBSTACLES_DEPTH (-0.94f)
#define RAW_OBSTACLES_DEPTH (-0.93f)
#define LASER_SCAN_DEPTH (-0.92f)

#define ROBOT_RADIUS (0.3f)

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_RENDER, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_RENDER)

namespace nav_view
{

NavViewPanel::NavViewPanel( wxWindow* parent )
: NavViewPanelGenerated( parent )
, ogre_root_( Ogre::Root::getSingletonPtr() )
, map_object_( NULL )
, cloud_object_( NULL )
, radius_object_( NULL )
, path_line_object_( NULL )
, local_path_object_( NULL )
, footprint_object_( NULL )
, inflated_obstacles_object_( NULL )
, raw_obstacles_object_( NULL )
, laser_scan_object_( NULL )
, mouse_x_(0)
, mouse_y_(0)
, scale_(10.0f)
, new_cloud_( false )
, new_gui_path_( false )
, new_local_path_( false )
, new_robot_footprint_( false )
, new_inflated_obstacles_( false )
, new_raw_obstacles_( false )
, new_gui_laser_( false )
, new_occ_diff_( false )
, current_tool_( NULL )
{
  ros_node_ = ros::node::instance();

  /// @todo This should go away once creation of the ros::node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::node( "NavViewPanel", ros::node::DONT_HANDLE_SIGINT );
  }
  ROS_ASSERT( ros_node_ );

  tf_client_ = new tf::TransformListener( *ros_node_ );

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );
  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  render_panel_ = new ogre_tools::wxOgreRenderWindow( ogre_root_, this );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  camera_ = scene_manager_->createCamera( "OrthoTopDown" );
  camera_->setProjectionType( Ogre::PT_ORTHOGRAPHIC );
  camera_->setPosition(0.0f, 0.0f, 30.0f);
  camera_->lookAt( 0.0f, 0.0f, 0.0f );
  camera_->setNearClipDistance(0.001f);
  camera_->setFarClipDistance(50.0f);

  render_panel_->getViewport()->setCamera( camera_ );

  ros_node_->advertise<std_msgs::Planner2DGoal>("goal", 1);
  ros_node_->advertise<std_msgs::Pose2DFloat32>("initialpose", 1);
  ros_node_->subscribe("particlecloud", cloud_, &NavViewPanel::incomingParticleCloud, this, 1);
  ros_node_->subscribe("gui_path", path_line_, &NavViewPanel::incomingGuiPath, this, 1);
  ros_node_->subscribe("local_path", local_path_, &NavViewPanel::incomingLocalPath, this, 1);
  ros_node_->subscribe("robot_footprint", robot_footprint_, &NavViewPanel::incomingRobotFootprint, this, 1);
  ros_node_->subscribe("inflated_obstacles", inflated_obstacles_, &NavViewPanel::incomingInflatedObstacles, this, 1);
  ros_node_->subscribe("raw_obstacles", raw_obstacles_, &NavViewPanel::incomingRawObstacles, this, 1);
  ros_node_->subscribe("gui_laser", laser_scan_, &NavViewPanel::incomingGuiLaser, this, 1);

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->SetFocus();

  render_panel_->Connect( wxEVT_CHAR, wxKeyEventHandler( NavViewPanel::onChar ), NULL, this );

  Connect( EVT_RENDER, wxCommandEventHandler( NavViewPanel::onRender ), NULL, this );

  render_panel_->setOrthoScale( scale_ );

  update_timer_ = new wxTimer( this );
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( NavViewPanel::onUpdate ), NULL, this );
  update_timer_->Start( 100 );

  createRadiusObject();

  current_tool_ = new MoveTool( this );
}

NavViewPanel::~NavViewPanel()
{
  Disconnect( EVT_RENDER, wxCommandEventHandler( NavViewPanel::onRender ), NULL, this );

  render_panel_->Disconnect( wxEVT_CHAR, wxKeyEventHandler( NavViewPanel::onChar ), NULL, this );

  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( NavViewPanel::onRenderWindowMouseEvents ), NULL, this );

  ros_node_->unadvertise("goal");
  ros_node_->unadvertise("initialpose");
  ros_node_->unsubscribe("particlecloud", &NavViewPanel::incomingParticleCloud, this);
  ros_node_->unsubscribe("gui_path", &NavViewPanel::incomingGuiPath, this);
  ros_node_->unsubscribe("local_path", &NavViewPanel::incomingLocalPath, this);
  ros_node_->unsubscribe("robot_footprint", &NavViewPanel::incomingRobotFootprint, this);
  ros_node_->unsubscribe("inflated_obstacles", &NavViewPanel::incomingInflatedObstacles, this);
  ros_node_->unsubscribe("raw_obstacles", &NavViewPanel::incomingRawObstacles, this);
  ros_node_->unsubscribe("gui_laser", &NavViewPanel::incomingGuiLaser, this);

  delete update_timer_;
  delete current_tool_;
  delete tf_client_;

  ogre_root_->destroySceneManager( scene_manager_ );
}

void NavViewPanel::queueRender()
{
  wxCommandEvent event( EVT_RENDER, GetId() );
  wxPostEvent( this, event );
}

void NavViewPanel::onRender( wxCommandEvent& event )
{
  render_panel_->Refresh();
}

void NavViewPanel::loadMap()
{
  std_srvs::StaticMap::request  req;
  std_srvs::StaticMap::response resp;
  printf("Requesting the map...\n");
  if( !ros::service::call("static_map", req, resp) )
  {
    printf("request failed\n");

    return;
  }
  printf("Received a %d X %d map @ %.3f m/pix\n",
         resp.map.width,
         resp.map.height,
         resp.map.resolution);

  map_resolution_ = resp.map.resolution;

  // Pad dimensions to power of 2
  map_width_ = resp.map.width;//(int)pow(2,ceil(log2(resp.map.width)));
  map_height_ = resp.map.height;//(int)pow(2,ceil(log2(resp.map.height)));

  //printf("Padded dimensions to %d X %d\n", map_width_, map_height_);

  // Expand it to be RGB data
  int pixels_size = map_width_ * map_height_ * 3;
  unsigned char* pixels = new unsigned char[pixels_size];
  memset(pixels, 255, pixels_size);

  for(unsigned int j=0;j<resp.map.height;j++)
  {
    for(unsigned int i=0;i<resp.map.width;i++)
    {
      unsigned char val;
      if(resp.map.data[j*resp.map.width+i] == 100)
        val = 0;
      else if(resp.map.data[j*resp.map.width+i] == 0)
        val = 255;
      else
        val = 127;

      int pidx = 3*(j*map_width_ + i);
      pixels[pidx+0] = val;
      pixels[pidx+1] = val;
      pixels[pidx+2] = val;
    }
  }

  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( pixels, pixels_size ));
  static int tex_count = 0;
  std::stringstream ss;
  ss << "NavViewMapTexture" << tex_count++;
  map_texture_ = Ogre::TextureManager::getSingleton().loadRawData( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                                   pixel_stream, map_width_, map_height_, Ogre::PF_BYTE_RGB, Ogre::TEX_TYPE_2D,
                                                                   0);

  delete [] pixels;

  Ogre::SceneNode* map_node = NULL;
  if ( !map_object_ )
  {
    static int map_count = 0;
    std::stringstream ss;
    ss << "NavViewMapObject" << map_count++;
    map_object_ = scene_manager_->createManualObject( ss.str() );
    map_node = root_node_->createChildSceneNode();
    map_node->attachObject( map_object_ );

    ss << "Material";
    map_material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    map_material_->setReceiveShadows(false);
    map_material_->getTechnique(0)->setLightingEnabled(false);
  }
  else
  {
    map_node = map_object_->getParentSceneNode();
  }

  Ogre::Pass* pass = map_material_->getTechnique(0)->getPass(0);
  Ogre::TextureUnitState* tex_unit = NULL;
  if (pass->getNumTextureUnitStates() > 0)
  {
    tex_unit = pass->getTextureUnitState(0);
  }
  else
  {
    tex_unit = pass->createTextureUnitState();
  }

  tex_unit->setTextureName(map_texture_->getName());
  tex_unit->setTextureFiltering( Ogre::TFO_NONE );

  map_object_->clear();
  map_object_->begin(map_material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
  {
    // First triangle
    {
      // Top left
      map_object_->position( 0.0f, 0.0f, 0.0f );
      map_object_->textureCoord(0.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      map_object_->position( map_resolution_*map_width_, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(1.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom left
      map_object_->position( 0.0f, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(0.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );
    }

    // Second triangle
    {
      // Top left
      map_object_->position( 0.0f, 0.0f, 0.0f );
      map_object_->textureCoord(0.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Top right
      map_object_->position( map_resolution_*map_width_, 0.0f, 0.0f );
      map_object_->textureCoord(1.0f, 0.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );

      // Bottom right
      map_object_->position( map_resolution_*map_width_, map_resolution_*map_height_, 0.0f );
      map_object_->textureCoord(1.0f, 1.0f);
      map_object_->normal( 0.0f, 0.0f, 1.0f );
    }
  }
  map_object_->end();

  root_node_->setPosition(Ogre::Vector3(-map_width_*map_resolution_/2, -map_height_*map_resolution_/2, 0.0f));
  map_node->setPosition(Ogre::Vector3(0.0f, 0.0f, MAP_DEPTH));
}

void NavViewPanel::clearMap()
{
  if ( map_object_ )
  {
    scene_manager_->destroyManualObject( map_object_ );
    map_object_ = NULL;

    std::string tex_name = map_texture_->getName();
    map_texture_.setNull();
    Ogre::TextureManager::getSingleton().unload( tex_name );
  }
}

void NavViewPanel::onUpdate( wxTimerEvent& event )
{
  if ( !map_object_ )
  {
    static int count = 0;
    --count;

    if ( count < 0 )
    {
      loadMap();
      count = 100;
    }
  }

  if ( new_cloud_ )
  {
    processParticleCloud();
  }

  if ( new_gui_path_ )
  {
    processGuiPath();
  }

  if ( new_local_path_ )
  {
    processLocalPath();
  }

  if ( new_robot_footprint_ )
  {
    processRobotFootprint();
  }

  if ( new_inflated_obstacles_ )
  {
    processInflatedObstacles();
  }

  if ( new_raw_obstacles_ )
  {
    processRawObstacles();
  }

  if ( new_gui_laser_ )
  {
    processGuiLaser();
  }

  updateRadiusPosition();
}

void NavViewPanel::createRadiusObject()
{
  static int count = 0;
  std::stringstream ss;
  ss << "NavViewRadius" << count++;
  radius_object_ = scene_manager_->createManualObject( ss.str() );
  Ogre::SceneNode* node = root_node_->createChildSceneNode();
  node->attachObject( radius_object_ );

  radius_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
  {
    Ogre::ColourValue color( 0.2, 1.0, 0.4 );
    for( float f = 0; f < Ogre::Math::TWO_PI; f += 0.5f )
    {
      radius_object_->position( ROBOT_RADIUS * cos(f), ROBOT_RADIUS * sin(f), 0.0f );
      radius_object_->colour(color);
    }

    radius_object_->position( ROBOT_RADIUS , 0.0f, 0.0f );
    radius_object_->colour(color);

    radius_object_->position( 0.0f, 0.0f, 0.0f );
    radius_object_->colour(color);
  }
  radius_object_->end();

  updateRadiusPosition();
}

void NavViewPanel::updateRadiusPosition()
{
  try
  {
    tf::Stamped<tf::Pose> robot_pose(btTransform(btQuaternion(0,0,0), btVector3(0,0,0)), ros::Time(0), "base");
    tf::Stamped<tf::Pose> map_pose;

    tf_client_->transformPose("map", robot_pose, map_pose);
    double yaw, pitch, roll;
    map_pose.getBasis().getEulerZYX(yaw, pitch, roll);

    Ogre::SceneNode* node = radius_object_->getParentSceneNode();
    node->setPosition( Ogre::Vector3(map_pose.getOrigin().x(), map_pose.getOrigin().y(), RADIUS_DEPTH) );
    node->setOrientation( Ogre::Quaternion( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Z ) );

    queueRender();
  }
  catch ( tf::TransformException& e )
  {
  }
}

void NavViewPanel::processParticleCloud()
{
  cloud_.lock();
  new_cloud_ = false;

  if ( !cloud_object_ )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewCloud" << count++;
    cloud_object_ = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( cloud_object_ );
  }

  cloud_object_->clear();

  Ogre::ColourValue color( 1.0f, 0.0f, 0.0f, 1.0f );
  int num_particles = cloud_.get_particles_size();
  cloud_object_->estimateVertexCount( num_particles * 8 );
  cloud_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
  for( int i=0; i < num_particles; ++i)
  {
    Ogre::Vector3 pos( cloud_.particles[i].x, cloud_.particles[i].y, 0.0f );
    Ogre::Quaternion orient( Ogre::Quaternion( Ogre::Radian( cloud_.particles[i].th ), Ogre::Vector3::UNIT_Z ) );

    Ogre::Vector3 vertices[8];
    vertices[0] = pos;
    vertices[1] = pos + orient * Ogre::Vector3(ROBOT_RADIUS, 0.0f, 0.0f);
    vertices[2] = vertices[1];
    vertices[3] = pos + orient * Ogre::Vector3(0.75*ROBOT_RADIUS, -0.25*ROBOT_RADIUS, 0.0f);
    vertices[4] = vertices[3];
    vertices[5] = pos + orient * Ogre::Vector3(0.75*ROBOT_RADIUS, 0.25*ROBOT_RADIUS, 0.0f);
    vertices[6] = vertices[5];
    vertices[7] = pos + orient * Ogre::Vector3(ROBOT_RADIUS, 0.0f, 0.0f);

    for ( int i = 0; i < 8; ++i )
    {
      cloud_object_->position( vertices[i] );
      cloud_object_->colour( color );
    }
  }
  cloud_object_->end();


  cloud_object_->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, CLOUD_DEPTH ) );

  cloud_.unlock();

  queueRender();
}

void NavViewPanel::createObjectFromPolyLine( Ogre::ManualObject*& object, std_msgs::Polyline2D& path, Ogre::RenderOperation::OperationType op, float depth, bool loop )
{
  path.lock();

  if ( !object )
  {
    static int count = 0;
    std::stringstream ss;
    ss << "NavViewPathLine" << count++;
    object = scene_manager_->createManualObject( ss.str() );
    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    node->attachObject( object );
  }

  object->clear();

  Ogre::ColourValue color( path.color.r, path.color.g, path.color.b );
  int num_points = path.get_points_size();
  object->estimateVertexCount( num_points);
  object->begin( "BaseWhiteNoLighting", op );
  for( int i=0; i < num_points; ++i)
  {
    object->position(path.points[i].x, path.points[i].y, 0.0f);
    object->colour( color );
  }

  if ( loop && num_points > 0 )
  {
    object->position(path.points[0].x, path.points[0].y, 0.0f);
    object->colour( color );
  }

  object->end();

  object->getParentSceneNode()->setPosition( Ogre::Vector3( 0.0f, 0.0f, depth ) );

  path.unlock();

  queueRender();
}

void NavViewPanel::processGuiPath()
{
  new_gui_path_ = false;

  createObjectFromPolyLine( path_line_object_, path_line_, Ogre::RenderOperation::OT_LINE_LIST, PATH_LINE_DEPTH, false );
}

void NavViewPanel::processLocalPath()
{
  new_local_path_ = false;

  createObjectFromPolyLine( local_path_object_, local_path_, Ogre::RenderOperation::OT_LINE_LIST, LOCAL_PATH_DEPTH, false );
}

void NavViewPanel::processRobotFootprint()
{
  new_robot_footprint_ = false;

  createObjectFromPolyLine( footprint_object_, robot_footprint_, Ogre::RenderOperation::OT_LINE_LIST, FOOTPRINT_DEPTH, true );
}

void NavViewPanel::processInflatedObstacles()
{
  new_inflated_obstacles_ = false;

  createObjectFromPolyLine( inflated_obstacles_object_, inflated_obstacles_, Ogre::RenderOperation::OT_POINT_LIST, INFLATED_OBSTACLES_DEPTH, false );
}

void NavViewPanel::processRawObstacles()
{
  new_raw_obstacles_ = false;

  createObjectFromPolyLine( raw_obstacles_object_, raw_obstacles_, Ogre::RenderOperation::OT_POINT_LIST, RAW_OBSTACLES_DEPTH, false );
}

void NavViewPanel::processGuiLaser()
{
  new_gui_laser_ = false;

  createObjectFromPolyLine( laser_scan_object_, laser_scan_, Ogre::RenderOperation::OT_POINT_LIST, LASER_SCAN_DEPTH, false );
}

void NavViewPanel::incomingParticleCloud()
{
  new_cloud_ = true;
}

void NavViewPanel::incomingGuiPath()
{
  new_gui_path_ = true;
}

void NavViewPanel::incomingLocalPath()
{
  new_local_path_ = true;
}

void NavViewPanel::incomingRobotFootprint()
{
  new_robot_footprint_ = true;
}

void NavViewPanel::incomingInflatedObstacles()
{
  new_inflated_obstacles_ = true;
}

void NavViewPanel::incomingRawObstacles()
{
  new_raw_obstacles_ = true;
}

void NavViewPanel::incomingGuiLaser()
{
  new_gui_laser_ = true;
}

void NavViewPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event.GetX();
  mouse_y_ = event.GetY();

  int flags = current_tool_->processMouseEvent( event, last_x, last_y, scale_ );

  if ( flags & Tool::Render )
  {
    render_panel_->setOrthoScale( scale_ );
    queueRender();
  }

  if ( flags & Tool::Finished )
  {
    delete current_tool_;
    current_tool_ = new MoveTool( this );

    toolbar_->ToggleTool( ID_MOVE_TOOL, true );
  }
}

void NavViewPanel::onChar( wxKeyEvent& event )
{
  switch( event.GetKeyCode() )
  {
  case WXK_ESCAPE:
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
      toolbar_->ToggleTool( ID_MOVE_TOOL, true );
    }
    break;
  case 'm':
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
      toolbar_->ToggleTool( ID_MOVE_TOOL, true );
    }
    break;

  case 'g':
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, true );
      toolbar_->ToggleTool( ID_GOAL_TOOL, true );
    }
    break;

  case 'p':
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, false );
      toolbar_->ToggleTool( ID_POSE_TOOL, true );
    }
    break;

  default:
    event.Skip();
    break;
  }
}

void NavViewPanel::onToolClicked( wxCommandEvent& event )
{
  switch( event.GetId() )
  {
  case ID_MOVE_TOOL:
    {
      delete current_tool_;
      current_tool_ = new MoveTool( this );
    }
    break;

  case ID_GOAL_TOOL:
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, true );
    }
    break;

  case ID_POSE_TOOL:
    {
      delete current_tool_;
      current_tool_ = new PoseTool( this, false );
    }
    break;

  default:
    ROS_BREAK();
  }

  ROS_ASSERT( current_tool_ );
}

void NavViewPanel::onReloadMap( wxCommandEvent& event )
{
  clearMap();
  loadMap();
}

} // namespace nav_view
