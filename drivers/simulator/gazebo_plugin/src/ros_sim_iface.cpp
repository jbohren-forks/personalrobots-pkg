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
 @mainpage
   Desc: RosSimIface plugin for maanipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosSimIface plugin reads ROS PoseWithRatesStamped messages
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/ros_sim_iface.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_sim_iface", RosSimIface);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosSimIface::RosSimIface(Entity *parent)
    : Controller(parent)
{
  this->myParent = parent;

  if (!this->myParent)
    gzthrow("RosSimIface controller requires an Entity as its parent");


  // set parent sensor to active automatically
  this->myParent->SetActive(true);


  rosnode = ros::g_node; // comes from where?
  int argc = 0;
  char** argv = NULL;
  if (rosnode == NULL)
  {
    ros::init(argc,argv);
    rosnode = new ros::Node("ros_gazebo",ros::Node::DONT_HANDLE_SIGINT);
    ROS_DEBUG("Starting node in simiface");
  }

  Param::Begin(&this->parameters);
  this->topicNameP = new ParamT<std::string>("topicName","simiface_pose", 0);
  this->frameNameP = new ParamT<std::string>("frameName","map", 0);
  this->xyzP  = new ParamT<Vector3>("xyz" ,Vector3(0,0,0), 0);
  this->rpyP  = new ParamT<Vector3>("rpy" ,Vector3(0,0,0), 0);
  this->velP  = new ParamT<Vector3>("vel" ,Vector3(0,0,0), 0);
  this->angVelP  = new ParamT<Vector3>("angVel" ,Vector3(0,0,0), 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosSimIface::~RosSimIface()
{

  delete this->topicNameP;
  delete this->frameNameP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->velP;
  delete this->angVelP;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosSimIface::LoadChild(XMLConfigNode *node)
{
  this->topicNameP->Load(node);
  this->frameNameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->velP->Load(node);
  this->angVelP->Load(node);

  this->topicName = this->topicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->xyz = this->xyzP->GetValue();
  this->rpy = this->rpyP->GetValue();
  this->vel = this->velP->GetValue();
  this->angVel = this->angVelP->GetValue();

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosSimIface::InitChild()
{

  ROS_DEBUG("================= %s", this->topicName.c_str());
  rosnode->advertise<image_msgs::Image>(this->topicName,1);

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosSimIface::UpdateChild()
{

  const unsigned char *src;

  // Get a pointer to image data
  src = this->myParent->GetImageData(0);

  //std::cout << " updating SimIface " << this->topicName << " " << data->image_size << std::endl;
  if (src)
  {
    //double tmpT0 = Simulator::Instance()->GetWallTime();

    this->lock.lock();
    // copy data into image
    this->imageMsg.header.frame_id = this->frameName;
    this->imageMsg.header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
    this->imageMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->imageMsg.header.stamp.sec) );

    //double tmpT1 = Simulator::Instance()->GetWallTime();
    //double tmpT2;

    /// @todo: don't bother if there are no subscribers
    if (this->rosnode->numSubscribers(this->topicName) > 0)
    {
      // copy from src to imageMsg
      fillImage(this->imageMsg   ,"image_raw" ,
                this->height     ,this->width ,this->depth,
                "mono"            ,"uint8"     ,
                (void*)src );

      //tmpT2 = Simulator::Instance()->GetWallTime();

      // publish to ros
      rosnode->publish(this->topicName,this->imageMsg);
    }

    //double tmpT3 = Simulator::Instance()->GetWallTime();

    this->lock.unlock();
  }




  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  //simIface->data->reset = 1;

  // Example of how to move a model (box1_model)
  uint8_t name[512] = "pioneer2dx_model1";

  for (int i=0; i< 10; i++)
  {
    simIface->Lock(1);

    gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);

    request->type = gazebo::SimulationRequestData::SET_POSE2D;
    memcpy(request->modelName, name, 512);

    request->modelPose.pos.x = i+.1;
    request->modelPose.pos.y = 0;

    simIface->Unlock();

    usleep(100000);
  }


  // Example of resetting the simulator
  /*simIface->Lock(1);
  simIface->data->reset = 1;
  simIface->Unlock();
  */

  usleep(1000000);

  simIface->Close();
  delete simIface;


}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosSimIface::FiniChild()
{
  rosnode->unadvertise(this->topicName);
  this->myParent->SetActive(false);
}


