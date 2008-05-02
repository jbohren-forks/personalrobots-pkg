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

#include <float.h>

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

  memset(&this->map,0,sizeof(player_map_data_t));
  this->map_resolution = 0.0;

  int eg = cf->ReadInt(section, "enable_gui", 0);
  this->enable_gui = eg ? true : false;
}

QApplication* app;

void*
GUI_thread(void* d)
{
  PlayerGFSWrapper* pd = (PlayerGFSWrapper*)d;
  int argc;
  char* argv[1];

  argv[0] = "playergfswrapper";
  argc=1;

  app = new QApplication(argc,argv);
  GFSMainWindow* mainWin=new GFSMainWindow(pd->gsp);
  app->setMainWidget(mainWin);
  mainWin->show();
  pd->gsp->start(pd);
  mainWin->start(1000);
  app->exec();
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
  // Set the size of the mapper's outgoing event queue.  We service the
  // queue to recieve, e.g., news that a new map is ready.
  this->gsp->setEventBufferSize(10000);

  if(this->laser_rs)
  {
    delete this->laser_rs;
    this->laser_rs = NULL;
  }

  // start GFS thread
  if(this->enable_gui)
    pthread_create(&this->gui_thread, 0, GUI_thread, this);
  else
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
  if(this->enable_gui)
    app->exit();
  else
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
    // Give chance to be shutdown
    pthread_testcancel();

    // Wait for incoming Player messages (e.g., laser scans)
    this->InQueue->Wait();

    // Process Player messages (includes handing laser scans off the
    // mapper)
    this->ProcessMessages();

    // Process events coming from mapper thread (e.g., new map is ready)
    //this->ProcessGFSEvents();
  }
}

void
PlayerGFSWrapper::ProcessGFSEvents()
{
  // check for new pose / map from GFS thread
  GridSlamProcessorThread::EventDeque events=this->gsp->getEvents();
  for(GridSlamProcessorThread::EventDeque::const_iterator it=events.begin(); 
      it!=events.end();
      it++)
  {
    GridSlamProcessorThread::MapEvent* mapEvent = 
            dynamic_cast<GridSlamProcessorThread::MapEvent*>(*it);
    if(mapEvent)
    {
      unsigned int sx, sy;
      sx = mapEvent->pmap->getMapSizeX();
      sy = mapEvent->pmap->getMapSizeY();

      if((this->map.width != sx) ||
         (this->map.height != sy))
      {
        this->map.width = sx;
        this->map.height = sy;
        this->map.data_count = this->map.width * this->map.height;
        this->map.data = (int8_t*)realloc(this->map.data,
                                          sizeof(int8_t) *
                                          this->map.data_count);
        assert(this->map.data);
      }
      this->map_resolution = mapEvent->pmap->getResolution();

      printf("got a %d X %d map @ %.3f m / pix\n", sx,sy,this->map_resolution);

      double v;
      double minv=DBL_MAX;
      double maxv=-DBL_MAX;
      for(unsigned int j=0;j<sy;j++)
      {
        for(unsigned int i=0;i<sx;i++)
        {
          //v = mapEvent->pmap->cell(i,j);
          IntPoint p(i,j);
          v = mapEvent->pmap->cell(p);
          if((v != 1.0) && (v != -1.0))
          {
            printf("v: %.9lf\n", v);
          }
          //int grayValue=255-(int)(255.*v);
          if(v < 0)
            this->map.data[i + j * this->map.width] = 0;
          else
          {
            if(v < minv)
              minv = v;
            if(v > maxv)
              maxv = v;

              /*
            if(v > .9)
              this->map.data[i + j * this->map.width] = 1;
            else
            {
              puts("free");
              this->map.data[i + j * this->map.width] = -1;
            }
            */
          }
        }
      }
      printf("minv: %.6lf  max: %.6lf\n", minv, maxv);
      // The MapEvent destructor will delete the pmap field if it's
      // non-NULL
      //delete mapEvent->pmap;
      delete mapEvent;
    }
    else
    {
      GridSlamProcessorThread::DoneEvent* doneEvent = 
              dynamic_cast<GridSlamProcessorThread::DoneEvent*>(*it);
      if(doneEvent)
      {
        this->gsp->stop();
        delete doneEvent;
      } 
      else
      {
        // TODO: handle other events somehow
        //history.push_back(*it);
      }
    }	
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
  // Is it a request for map meta-data?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_INFO, 
                                this->device_addr))
  {
    this->ProcessMapInfoRequest(resp_queue);
    return(0);
  }
  // Is it a request for a map tile?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, 
                           PLAYER_MAP_REQ_GET_DATA,
                           this->device_addr))
  {
    this->ProcessMapDataRequest(resp_queue,
                                (player_map_data_t*)data);
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

void
PlayerGFSWrapper::ProcessMapInfoRequest(QueuePointer& resp_queue)
{
  player_map_info_t info;
  info.scale = this->map_resolution;
  info.width = this->map.width;
  info.height = this->map.height;
  info.origin.px = 0.0;
  info.origin.py = 0.0;
  info.origin.pa = 0.0;

  this->Publish(this->map_id, resp_queue,
                PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_MAP_REQ_GET_INFO,
                (void*)&info);
}

// check that given coords are valid (i.e., on the map)
#define MAP_VALID(mf, i, j) ((i >= 0) && (i < mf.width) && (j >= 0) && (j < mf.height))
// compute linear index for given map coords
#define MAP_IDX(mf, i, j) ((mf.width) * (j) + (i))

void
PlayerGFSWrapper::ProcessMapDataRequest(QueuePointer& resp_queue,
                                        player_map_data_t* mapreq)
{
  player_map_data_t mapresp;

  unsigned int i, j;
  unsigned int oi, oj, si, sj;

  // Construct reply
  oi = mapresp.col = mapreq->col;
  oj = mapresp.row = mapreq->row;
  si = mapresp.width = mapreq->width;
  sj = mapresp.height = mapreq->height;
  mapresp.data_count = mapresp.width * mapresp.height;
  mapresp.data = new int8_t [mapresp.data_count];
  assert(mapresp.data);

  // Grab the pixels from the map
  for(j = 0; j < sj; j++)
  {
    for(i = 0; i < si; i++)
    {
      if(MAP_VALID(this->map, i + oi, j + oj))
        mapresp.data[i + j * si] = 
                this->map.data[MAP_IDX(this->map, i+oi, j+oj)];
      else
      {
        PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
        mapresp.data[i + j * si] = 0;
      }
    }
  }

  this->Publish(this->map_id, resp_queue,
                PLAYER_MSGTYPE_RESP_ACK,
                PLAYER_MAP_REQ_GET_DATA,
                (void*)&mapresp);
  delete [] mapresp.data;
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

