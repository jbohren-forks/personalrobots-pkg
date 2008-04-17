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

//#include <iostream>
#include <deque>
#include <pthread.h>
//#include <semaphore.h>

//#include <carmen/carmen.h>
//#include <carmen/global.h>
//#include <log/carmenconfiguration.h>

#include <libplayercore/playercore.h>

#include <gui/gsp_thread.h>

//#include <sensor/sensor_base/sensor.h>
//#include <log/sensorstream.h>
//#include <log/sensorlog.h>
#include <sensor/sensor_range/rangesensor.h>
#include <sensor/sensor_range/rangereading.h>

namespace GMapping {

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
    RangeSensor* laser_rs;

    // The GFS object
    GridSlamProcessorThread* gsp;

    // Queue of range readings, with mutex
    std::deque<RangeReading> rangeDeque;
    pthread_mutex_t rangeDeque_mutex;

    void ProcessLaser(player_msghdr_t* hdr,
                      player_laser_data_scanpose_t* data);

  public:
    PlayerGFSWrapper(ConfigFile* cf, int section);
    virtual int Setup();
    virtual int Shutdown();
    virtual int ProcessMessage(QueuePointer & resp_queue,
                               player_msghdr * hdr,
                               void * data);

    bool getReading(RangeReading& reading);

    /*
    void initializeIPC(const char* name);
    bool start(const char* name);
    bool isRunning();
    void lock();
    void unlock();
    int registerLocalizationMessages();

    int queueLength();
    OrientedPoint getTruePos();
    bool getReading(RangeReading& reading);
    void addReading(RangeReading& reading);
    const SensorMap& sensorMap();
    bool sensorMapComputed();
    bool isStopped();

    // conversion function  
    carmen_robot_laser_message reading2carmen(const RangeReading& reading);
    RangeReading carmen2reading(const carmen_robot_laser_message& msg);
    carmen_point_t point2carmen (const OrientedPoint& p);
    OrientedPoint carmen2point (const carmen_point_t& p);


    // carmen interaction
    void robot_frontlaser_handler(carmen_robot_laser_message* frontlaser);
    void robot_rearlaser_handler(carmen_robot_laser_message* frontlaser);
    void simulator_truepos_handler(carmen_simulator_truepos_message* truepos);
    //babsi:
    void navigator_go_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) ;
    void navigator_stop_handler(MSG_INSTANCE msgRef, BYTE_ARRAY callData, void*) ;

    //babsi:
    void publish_globalpos(carmen_localize_summary_p summary);
    void publish_particles(carmen_localize_particle_filter_p filter, 
                                  carmen_localize_summary_p summary);

    void shutdown_module(int sig);

  private:
    std::deque<RangeReading> m_rangeDeque;
    sem_t m_dequeSem;
    pthread_mutex_t m_mutex, m_lock;  
    pthread_t m_readingThread;
    void * m_reading_function(void*);
    bool m_threadRunning;
    SensorMap m_sensorMap;
    RangeSensor* m_frontLaser, *m_rearLaser;
    OrientedPoint m_truepos;
    bool stopped;
    */
};

} //end namespace



#endif
/*
int main (int argc, char** argv) {

	CarmenWrapper::init_carmen(argc, argv);
	while (true) {
		IPC_listenWait(100);
	}    
	return 1;
}
*/
