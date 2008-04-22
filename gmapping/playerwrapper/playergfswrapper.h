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


#ifndef PLAYERWRAPPER_H
#define PLAYERWRAPPER_H

#include <deque>
#include <pthread.h>

#include <gui/gsp_thread.h>

#include <sensor/sensor_range/rangesensor.h>
#include <sensor/sensor_range/rangereading.h>

#include <libplayercore/playercore.h>

class PlayerGFSWrapper : public Driver
{
  private:
    virtual void Main();
    int SetupLaser();

    // Devices we provide
    player_devaddr_t position_id;
    player_devaddr_t map_id;

    // Devices to which we subscribe
    player_devaddr_t laser_id;
    Device* laser;
    GMapping::RangeSensor* laser_rs;

    GMapping::SensorMap sensorMap;
    bool sensorMap_ready;

    // Queue of range readings, with mutex
    std::deque<RangeReading*> rangeDeque;
    pthread_mutex_t rangeDeque_mutex;
    pthread_cond_t rangeDeque_cond;
    pthread_mutex_t rangeDeque_cond_mutex;

    void ProcessLaser(player_msghdr_t* hdr,
                      player_laser_data_scanpose_t* data);
    void ProcessMapDataRequest(QueuePointer& resp_queue,
                               player_map_data_t* mapreq);
    void ProcessMapInfoRequest(QueuePointer& resp_queue);

    void ProcessGFSEvents();

    pthread_t gui_thread;

    // Current map
    player_map_data_t map;
    double map_resolution;
    bool enable_gui;

  public:
    PlayerGFSWrapper(ConfigFile* cf, int section);
    virtual int Setup();
    virtual int Shutdown();
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);

    // Interface used by the GFS thread
    GMapping::RangeReading* getReading();
    bool sensorMapComputed() { return(this->sensorMap_ready); }
    const SensorMap& getSensorMap() { return(this->sensorMap); }
    
    // The GFS object (public so that it can be accessed from the GUI
    // thread)
    GridSlamProcessorThread* gsp;
};

#endif
