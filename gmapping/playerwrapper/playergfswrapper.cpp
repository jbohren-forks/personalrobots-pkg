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

#include <gui/qparticleviewer.h>
#include <gui/qgraphpainter.h>

#include <qapplication.h>
#include <qframe.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qvbox.h>
#include <qmainwindow.h>

#include "playergfswrapper.h"

class GFSMainWindow: public QMainWindow{
public:
  GFSMainWindow(GridSlamProcessorThread* t){
    gsp_thread=t;
    QVBoxLayout* layout=new QVBoxLayout(this);
    pviewer=new QParticleViewer(this,0,0,gsp_thread);
    pviewer->setGeometry(0,0,500,500);
    pviewer->setFocusPolicy(QParticleViewer::ClickFocus);
    layout->addWidget(pviewer);
						
    gpainter=new QGraphPainter(this);
    gpainter->setFixedHeight(100);
    layout->addWidget(gpainter);
    gpainter->setRange(0,1);
    gpainter->setTitle("Neff");
		
    help = new QLabel(QString("+/- - zoom | b - show/hide best path | p - show/hide all paths | c - center robot "),this); 
    help->setMaximumHeight(30);
    layout->addWidget(help);
	
    QObject::connect( pviewer, SIGNAL(neffChanged(double) ), gpainter, SLOT(valueAdded(double)) );
    setTabOrder(pviewer, pviewer);
  }
		
  void start(int c){
    pviewer->start(c);
    gpainter->start(c);
  }

protected:
  GridSlamProcessorThread* gsp_thread;
  QVBoxLayout* layout;
  QParticleViewer* pviewer;
  QGraphPainter* gpainter;
  QLabel* help;
};


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

  this->laser_rs = NULL;
  this->sensorMap_ready = false;
}

void*
GUI_thread(void* d)
{
  PlayerGFSWrapper* pd = (PlayerGFSWrapper*)d;
  int argc;
  char* argv[1];

  argv[0] = "playergfswrapper";
  argc=1;

  QApplication app(argc,argv);
  GFSMainWindow* mainWin=new GFSMainWindow(pd->gsp);
  app.setMainWidget(mainWin);
  mainWin->show();
  //pd->gsp->setEventBufferSize(10000);
  pd->gsp->start(pd);
  mainWin->start(1000);
  app.exec();
  return(NULL);
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
  pthread_cond_init(&this->rangeDeque_cond, NULL);
  pthread_mutex_init(&this->rangeDeque_cond_mutex, NULL);

  // TODO: get laser geometry

  this->gsp = new GridSlamProcessorThread();

  if(this->laser_rs)
  {
    delete this->laser_rs;
    this->laser_rs = NULL;
  }

  // start GFS thread
  //this->gsp->start(this);
  pthread_create(&this->gui_thread, 0, GUI_thread, this);

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
  pthread_mutex_destroy(&this->rangeDeque_cond_mutex);
  pthread_cond_destroy(&this->rangeDeque_cond);

  return(0);
}

void
PlayerGFSWrapper::Main()
{
  for(;;)
  {
    pthread_testcancel();

    this->InQueue->Wait();

    this->ProcessMessages();

    // TODO: check for new pose / map from GFS thread
  }
}

GMapping::RangeReading*
PlayerGFSWrapper::getReading()
{
  // Do we need to wait?
  bool needtowait;
  pthread_mutex_lock(&this->rangeDeque_mutex);
  needtowait = this->rangeDeque.empty();
  pthread_mutex_unlock(&this->rangeDeque_mutex);

  if(needtowait)
  {
    pthread_cleanup_push((void(*)(void*))pthread_mutex_unlock,
                         (void*)&this->rangeDeque_cond_mutex);
    pthread_mutex_lock(&this->rangeDeque_cond_mutex);
    pthread_cond_wait(&this->rangeDeque_cond,&this->rangeDeque_cond_mutex);
    pthread_mutex_unlock(&this->rangeDeque_cond_mutex);
    pthread_cleanup_pop(0);
  }

  // Now there must be data
  pthread_mutex_lock(&this->rangeDeque_mutex);
  assert(!this->rangeDeque.empty());

  // remove the reading
  GMapping::RangeReading* ret=this->rangeDeque.front();
  this->rangeDeque.pop_front();

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
  if(!this->laser_rs)
  {
    // create this->laser_rs
    std::string lasername = "FLASER";
    this->laser_rs = 
            new GMapping::RangeSensor(lasername,
                            data->scan.ranges_count,
                            data->scan.resolution,
                            //OrientedPoint(0,0,laser_pose.pa-robot_pose.theta),
                            GMapping::OrientedPoint(0,0,-data->pose.pa),
                            0, data->scan.max_range);
    this->laser_rs->updateBeamsLookup();
    this->sensorMap.insert(make_pair(lasername, this->laser_rs));
    this->sensorMap_ready = true;
  }

  GMapping::RangeReading* reading = new GMapping::RangeReading(this->laser_rs,hdr->timestamp);
  reading->resize(data->scan.ranges_count);
  for(unsigned int i=0;i<data->scan.ranges_count;i++)
    (*reading)[i] = data->scan.ranges[i];
  reading->setPose(GMapping::OrientedPoint(data->pose.px,
                                data->pose.py,
                                data->pose.pa));

  pthread_mutex_lock(&this->rangeDeque_mutex);
  this->rangeDeque.push_back(reading);

  // Signal that data is available on the queue
  pthread_mutex_lock(&this->rangeDeque_cond_mutex);
  pthread_cond_broadcast(&this->rangeDeque_cond);
  pthread_mutex_unlock(&this->rangeDeque_cond_mutex);

  printf("queue size: %d\n", this->rangeDeque.size());
  pthread_mutex_unlock(&this->rangeDeque_mutex);
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.

// A factory creation function, declared outside of the class so that it
// can be invoked without any object context (alternatively, you can
// declare it static in the class).  In this function, we create and return
// (as a generic Driver*) a pointer to a new instance of this driver.
Driver* 
PlayerGFSWrapper_Init(ConfigFile* cf, int section)
{
  // Create and return a new instance of this driver
  return((Driver*)(new PlayerGFSWrapper(cf, section)));
}

/* need the extern to avoid C++ name-mangling  */
extern "C" {
  int player_driver_init(DriverTable* table)
  {
    puts("PlayerGFSWrapper driver initializing");
    table->AddDriver("playergfswrapper", PlayerGFSWrapper_Init);
    puts("PlayerGFSWrapper driver done");
    return(0);
  }
}

