/*****************************************************************
 *
 * This file is part of the GMAPPING project
 *
 * Player wrapper code (c) 2008 Brian Gerkey
 *
 * GMAPPING Copyright (c) 2004 Giorgio Grisetti, 
 * Cyrill Stachniss, and Wolfram Burgard
 *
 * This software is licensed under the "Creative Commons 
 * License (Attribution-NonCommercial-ShareAlike 2.0)" 
 * and is copyrighted by Giorgio Grisetti, Cyrill Stachniss, 
 * and Wolfram Burgard.
 * 
 * Further information on this license can be found at:
 * http://creativecommons.org/licenses/by-nc-sa/2.0/
 * 
 * GMAPPING is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 *
 *****************************************************************/


#include "playergfswrapper.h"

using namespace GMapping;

PlayerGFSWrapper::PlayerGFSWrapper(ConfigFile* cf, int section)
    : Driver(cf, section)
{
  memset(&this->position_id, 0, sizeof(player_devaddr_t));
  memset(&this->map_id, 0, sizeof(player_devaddr_t));

  // Do we create a position interface?
  if(cf->ReadDeviceAddr(&(this->position_id), section, "provides",
                        PLAYER_POSITION2D_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->position_id))
    {
      this->SetError(-1);
      return;
    }
  }
  
  // Do we create a map interface?
  if(cf->ReadDeviceAddr(&(this->map_id), section, "provides",
                        PLAYER_MAP_CODE, -1, NULL) == 0)
  {
    if(this->AddInterface(this->map_id))
    {
      this->SetError(-1);
      return;
    }
  }

  // Must have a laser device to read from
  if(cf->ReadDeviceAddr(&(this->laser_id), section, "requires",
                        PLAYER_LASER_CODE, -1, NULL) != 0)
  {
    PLAYER_ERROR("must have a laser device from which to read");
    this->SetError(-1);
    return;
  }
}

int
PlayerGFSWrapper::Setup()
{
  // Subscribe to the laser device.
  if(!(this->laser = deviceTable->GetDevice(this->laser_id)))
  {
    PLAYER_ERROR("unable to locate suitable laser device");
    return(-1);
  }
  if(this->laser->Subscribe(this->InQueue) != 0)
  {
    PLAYER_ERROR("unable to subscribe to laser device");
    return(-1);
  }

  pthread_mutex_init(&this->rangeDeque_mutex, NULL);

  // TODO: get laser geometry

  this->gsp = new GridSlamProcessorThread();

  // TODO: create this->laser_rs

  // TODO: compute GFS "sensorMap"

  // start GFS thread
  this->gsp->start(this);

  this->StartThread();
  return(0);
}

int
PlayerGFSWrapper::Shutdown()
{
  this->StopThread();

  this->laser->Unsubscribe(this->InQueue);

  // stop GFS thread
  this->gsp->stop();

  delete this->gsp;

  pthread_mutex_destroy(&this->rangeDeque_mutex);

  return(0);
}

void
PlayerGFSWrapper::Main()
{
  for(;;)
  {
    this->ProcessMessages();

    // TODO: check for new pose / map from GFS thread
  }
}

bool
PlayerGFSWrapper::getReading(RangeReading& reading)
{
  bool ret=false;
  pthread_mutex_lock(&this->rangeDeque_mutex);
  if(!this->rangeDeque.empty())
  {
    reading=this->rangeDeque.front();
    this->rangeDeque.pop_front();
    ret=true;
  }
  pthread_mutex_unlock(&this->rangeDeque_mutex);
  return(ret);
}

int 
PlayerGFSWrapper::ProcessMessage(QueuePointer & resp_queue,
                                 player_msghdr * hdr,
                                 void * data)
{
  // TODO: handle:
  //   map requests
  //
  //   laser scanpose messages (convert and cache them for consumption by
  //   GFS thread)
  if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                           PLAYER_LASER_DATA_SCANPOSE,
                           this->laser_id))
  {
    this->ProcessLaser(hdr, (player_laser_data_scanpose_t*)data);
    return(0);
  }
  else
    return(-1);
}

void
PlayerGFSWrapper::ProcessLaser(player_msghdr_t* hdr,
                               player_laser_data_scanpose_t* data)
{
  RangeReading reading(this->laser_rs,hdr->timestamp);
  for(unsigned int i=0;i<data->scan.ranges_count;i++)
    reading[i] = data->scan.ranges[i];
  reading.setPose(OrientedPoint(data->pose.px,
                                data->pose.py,
                                data->pose.pa));

  pthread_mutex_lock(&this->rangeDeque_mutex);
  this->rangeDeque.push_back(reading);
  pthread_mutex_unlock(&this->rangeDeque_mutex);
}
