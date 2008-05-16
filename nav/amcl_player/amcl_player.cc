/*
 * amcl_player
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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/node.h>

// Messages that I need
#include <std_msgs/MsgLaserScan.h>
#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgParticleCloud2D.h>

// For transform support
#include <rosTF/rosTF.h>

#include <gdk-pixbuf/gdk-pixbuf.h>
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
// check that given coords are valid (i.e., on the map)
#define MAP_VALID(mf, i, j) ((i >= 0) && (i < mf->sx) && (j >= 0) && (j < mf->sy))
double get_time(void);
int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
       			const char* fname, int negate);

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* AdaptiveMCL_Init(ConfigFile* cf, int section);

class AmclNode: public ros::node, public Driver
{
  public:
    AmclNode(char* fname, double res);
    ~AmclNode();

    int Setup() {return(0);}
    int Shutdown() {return(0);}
    int ProcessMessage(QueuePointer &resp_queue, 
                       player_msghdr * hdr,
                       void * data);

    int start();
    int stop();

    int process();

    int setPose(double x, double y, double a);

  private:
    rosTFServer* tf;
    ConfigFile* cf;

    // incoming messages
    MsgRobotBase2DOdom localizedOdomMsg;
    MsgParticleCloud2D particleCloudMsg;
    MsgRobotBase2DOdom odomMsg;
    MsgLaserScan laserMsg;
    
    // Message callbacks
    void odomReceived();
    void laserReceived();

    // These are the devices that amcl offers, and to which we subscribe
    Driver* driver;
    Device* pdevice;
    Device* ldevice;
    player_devaddr_t oposition2d_addr;
    player_devaddr_t olocalize_addr;

    // These are the devices that amcl requires, and so which we must
    // provide
    player_devaddr_t position2d_addr;
    player_devaddr_t laser_addr;
    player_devaddr_t map_addr;
    Device* position2d_dev;
    Device* laser_dev;
    Device* map_dev;

    char* mapdata;
    int sx, sy;
    double resolution;
};

#define USAGE "USAGE: amcl_player <map.png> <res> <x> <y> <a>"

int
main(int argc, char** argv)
{
  if(argc < 6)
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc, argv);

  AmclNode an(argv[1],atof(argv[2]));
  assert(an.setPose(atof(argv[3]),atof(argv[4]),DTOR(atof(argv[5]))) == 0);

  // Start up the robot
  if(an.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  while(an.ok())
    an.process();
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  an.stop();

  ros::fini();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode(char* fname, double res) : 
        ros::node("amcl_player"), 
        Driver(NULL,-1,false,PLAYER_QUEUE_LEN)
{
  // libplayercore boiler plate
  player_globals_init();
  itable_init();

  // TODO: remove XDR dependency
  playerxdr_ftable_init();

  advertise<MsgRobotBase2DOdom>("localizedpose");
  advertise<MsgParticleCloud2D>("particlecloud");
  subscribe("odom", odomMsg, &AmclNode::odomReceived);
  subscribe("scan", laserMsg, &AmclNode::laserReceived);

  // TODO: get map via RPC
  assert(read_map_from_image(&this->sx, &this->sy, &this->mapdata, fname, 0)
         == 0);
  this->resolution = res;

  // TODO: automatically convert between string and player_devaddr_t
  // representations

  // The Player address that will be assigned to this device.  The format
  // is interface:index.  The interface must match what the driver is
  // expecting to provide.  The value of the index doesn't really matter, 
  // but 0 is most common.
  const char* oposition2d_saddr = "position2d:0";
  this->oposition2d_addr.host = 0;
  this->oposition2d_addr.robot = 0;
  this->oposition2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->oposition2d_addr.index = 0;

  const char* olocalize_saddr = "localize:0";
  this->olocalize_addr.host = 0;
  this->olocalize_addr.robot = 0;
  this->olocalize_addr.interf = PLAYER_LOCALIZE_CODE;
  this->olocalize_addr.index = 0;

  const char* position2d_saddr = "odometry:::position2d:1";
  this->position2d_addr.host = 0;
  this->position2d_addr.robot = 0;
  this->position2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->position2d_addr.index = 1;

  const char* laser_saddr = "laser:0";
  this->laser_addr.host = 0;
  this->laser_addr.robot = 0;
  this->laser_addr.interf = PLAYER_LASER_CODE;
  this->laser_addr.index = 0;

  const char* map_saddr = "laser:::map:0";
  this->map_addr.host = 0;
  this->map_addr.robot = 0;
  this->map_addr.interf = PLAYER_MAP_CODE;
  this->map_addr.index = 0;

  this->position2d_dev = deviceTable->AddDevice(this->position2d_addr, 
                                                NULL, false);
  assert(this->position2d_dev);
  this->position2d_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->position2d_dev->driver = (Driver*)this;

  this->laser_dev = deviceTable->AddDevice(this->laser_addr, 
                                           NULL, false);
  assert(this->laser_dev);
  this->laser_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->laser_dev->driver = (Driver*)this;

  this->map_dev = deviceTable->AddDevice(this->map_addr, 
                                         NULL, false);
  assert(this->map_dev);
  this->map_dev->InQueue = QueuePointer(this->Driver::InQueue);
  this->map_dev->driver = (Driver*)this;

  // Create a ConfigFile object, into which we'll stuff parameters.
  // Drivers assume that this object will persist throughout execution
  // (e.g., they store pointers to data inside it).  So it must NOT be
  // deleted until after the driver is shut down.
  this->cf = new ConfigFile();

  // Insert (name,value) pairs into the ConfigFile object.  These would
  // presumably come from the param server
  this->cf->InsertFieldValue(0,"provides",oposition2d_saddr);
  this->cf->InsertFieldValue(1,"provides",olocalize_saddr);

  // Fill in the requires fields for the device that this device
  // subscribes to
  this->cf->InsertFieldValue(0,"requires",position2d_saddr);
  this->cf->InsertFieldValue(1,"requires",laser_saddr);
  this->cf->InsertFieldValue(2,"requires",map_saddr);

  // Options
  //this->cf->InsertFieldValue(0,"enable_gui","1");

  // Create an instance of the driver, passing it the ConfigFile object.
  // The -1 tells it to look into the "global" section of the ConfigFile,
  // which is where ConfigFile::InsertFieldValue() put the parameters.
  assert((this->driver = AdaptiveMCL_Init(cf, -1)));

  // Print out warnings about parameters that were set, but which the
  // driver never looked at.
  cf->WarnUnused();

  // Grab from the global deviceTable a pointer to the Device that was 
  // created as part of the driver's initialization.
  assert((this->pdevice = deviceTable->GetDevice(oposition2d_saddr,false)));
  assert((this->ldevice = deviceTable->GetDevice(olocalize_addr,false)));

  this->tf = new rosTFServer(*this);
}

AmclNode::~AmclNode()
{
  delete this->cf;
  player_globals_fini();
  free(this->mapdata);
}

int 
AmclNode::ProcessMessage(QueuePointer &resp_queue, 
                         player_msghdr * hdr,
                         void * data)
{
  // Is it a new pose from amcl?
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_DATA, 
                           PLAYER_POSITION2D_DATA_STATE,
                           this->oposition2d_addr))
  {
    // Cast the message payload appropriately 
    player_position2d_data_t* pdata = 
            (player_position2d_data_t*)data;

    /*
    // publish new transform map->odom
    printf("lpose: %.3f %.3f %.3f\n",
           odomMsg.pos.x,
           odomMsg.pos.y,
           RTOD(odomMsg.pos.th));
    this->tf->sendEuler(ROSTF_FRAME_ODOM,
                        ROSTF_FRAME_MAP,
                        pdata->pos.px-this->odomMsg.pos.x,
                        pdata->pos.py-this->odomMsg.pos.y,
                        0.0,
                        pdata->pos.pa-this->odomMsg.pos.th,
                        0.0,
                        0.0,
                        (long long unsigned int)floor(hdr->timestamp),
                        (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL));

    printf("pose: (%.3f %.3f %.3f) @ (%llu:%llu)\n",
           pdata->pos.px,
           pdata->pos.py,
           RTOD(pdata->pos.pa),
           (long long unsigned int)floor(hdr->timestamp),
           (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 
                          1000000000ULL));
    */
    printf("pose: (%.3f %.3f %.3f)\n",
           pdata->pos.px,
           pdata->pos.py,
           RTOD(pdata->pos.pa));
    localizedOdomMsg.pos.x = pdata->pos.px;
    localizedOdomMsg.pos.y = pdata->pos.py;
    localizedOdomMsg.pos.th = pdata->pos.pa;
    localizedOdomMsg.header.stamp_secs = (unsigned long)floor(hdr->timestamp);
    localizedOdomMsg.header.stamp_nsecs = 
            (unsigned long)rint(1e9 * (hdr->timestamp -
                                       localizedOdomMsg.header.stamp_secs));
    localizedOdomMsg.__timestamp_override = true;
    publish("localizedpose", localizedOdomMsg);

    // Also request and publish the particle cloud
    Message* msg;
    if((msg = this->ldevice->Request(this->Driver::InQueue,
                                     PLAYER_MSGTYPE_REQ,
                                     PLAYER_LOCALIZE_REQ_GET_PARTICLES,
                                     NULL, 0, NULL, true)))
    {
      player_localize_get_particles_t* resp =
              (player_localize_get_particles_t*)(msg->GetPayload());
      particleCloudMsg.set_particles_size(resp->particles_count);
      for(unsigned int i=0;i<resp->particles_count;i++)
      {
        particleCloudMsg.particles[i].x = resp->particles[i].pose.px;
        particleCloudMsg.particles[i].y = resp->particles[i].pose.py;
        particleCloudMsg.particles[i].th = resp->particles[i].pose.pa;
      }
      publish("particlecloud", particleCloudMsg);
      delete msg;
    }
    else
    {
      puts("Warning: failed to get particle cloud from amcl");
    }

    return(0);
  }
  // Is it a request for the map metadata?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_INFO,
                                this->map_addr))
  {
    player_map_info_t info;
    info.scale = this->resolution;
    info.width = this->sx;
    info.height = this->sy;
    info.origin.px = 0;
    info.origin.py = 0;
    info.origin.pa = 0;
    this->Publish(this->map_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_MAP_REQ_GET_INFO,
                  (void*)&info);
    return(0);
  }
  // Is it a request for a map tile?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_DATA,
                                this->map_addr))
  {
    player_map_data_t* mapreq = (player_map_data_t*)data;

    player_map_data_t mapresp;

    int i, j;
    int oi, oj, si, sj;

    // Construct reply
    oi = mapresp.col = mapreq->col;
    oj = mapresp.row = mapreq->row;
    si = mapresp.width = mapreq->width;
    sj = mapresp.height = mapreq->height;
    mapresp.data_count = mapresp.width * mapresp.height;
    mapresp.data = new int8_t [mapresp.data_count];
    // Grab the pixels from the map
    for(j = 0; j < sj; j++)
    {
      for(i = 0; i < si; i++)
      {
        if(MAP_VALID(this, i + oi, j + oj))
          mapresp.data[i + j * si] = this->mapdata[MAP_IDX(this->sx, i+oi, j+oj)];
        else
        {
          PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
          mapresp.data[i + j * si] = 0;
        }
      }
    }

    this->Publish(this->map_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_MAP_REQ_GET_DATA,
                  (void*)&mapresp);
    delete [] mapresp.data;
    return(0);
  }
  // Is it a request for laser geometry (pose of laser wrt parent)?
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,
                                 PLAYER_LASER_REQ_GET_GEOM,
                                 this->laser_addr))
  {
    player_laser_geom_t geom;
    memset(&geom, 0, sizeof(geom));
    geom.pose.px = 0.05;
    geom.pose.py = 0;
    geom.pose.pyaw = 0;
    geom.size.sl = 0.06;
    geom.size.sw = 0.06;

    this->Publish(this->laser_addr,
                  resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_LASER_REQ_GET_GEOM,
                  (void*)&geom);
    return(0);
  }
  // Is it an ACK from a request that I sent earlier?
  else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_RESP_ACK,
                                 -1))
    return(0);
  else
  {
    printf("Unhandled Player message %d:%d:%d:%d\n",
           hdr->type,
           hdr->subtype,
           hdr->addr.interf,
           hdr->addr.index);
    return(-1);
  }
}

int 
AmclNode::start()
{
  // Subscribe to device, which causes it to startup
  if(this->pdevice->Subscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to subscribe the driver");
    return(-1);
  }
  else
    return(0);
}

int 
AmclNode::stop()
{
  // Unsubscribe from the device, which causes it to shutdown
  if(pdevice->Unsubscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to start the driver");
    return(-1);
  }
  else
  {
    // Give the driver a chance to shutdown.  Wish there were a way to
    // detect when that happened.
    usleep(1000000);
    return(0);
  }
}

int 
AmclNode::process()
{
  // Can't block here, because we won't exit cleanly.  The Wait() call
  // blocks on pthread_cond_wait(), which is a cancellation point, but in
  // this case we're the main thread and noone will try to cancel us.
  //
  // Block until there's a message on our queue
  //this->Driver::InQueue->Wait();

  if(!this->Driver::InQueue->Empty())
    this->Driver::ProcessMessages();
  else
    usleep(100000);

  return(0);
}

int
AmclNode::setPose(double x, double y, double a)
{
  player_localize_set_pose_t p;

  p.mean.px = x;
  p.mean.py = y;
  p.mean.pa = a;

  p.cov[0] = 0.5*0.5;
  p.cov[1] = 0.5*0.5;
  p.cov[2] = (M_PI/6.0)*(M_PI/6.0);

  this->ldevice->PutMsg(this->Driver::InQueue,
                        PLAYER_MSGTYPE_REQ,
                        PLAYER_LOCALIZE_REQ_SET_POSE,
                        (void*)&p,0,NULL);
  return(0);
}

void
AmclNode::laserReceived()
{
  // Got new scan; reformat and pass it on
  player_laser_data_t pdata;
  pdata.min_angle = this->laserMsg.angle_min;
  pdata.max_angle = this->laserMsg.angle_max;
  pdata.resolution = this->laserMsg.angle_increment;
  pdata.max_range = this->laserMsg.range_max;
  pdata.ranges_count = this->laserMsg.get_ranges_size();
  pdata.ranges = new float[pdata.ranges_count];
  assert(pdata.ranges);
  for(unsigned int i=0;i<pdata.ranges_count;i++)
    pdata.ranges[i] = this->laserMsg.ranges[i];
  pdata.intensity_count = this->laserMsg.get_intensities_size();
  pdata.intensity = new uint8_t[pdata.intensity_count];
  assert(pdata.intensity);
  for(unsigned int i=0;i<pdata.intensity_count;i++)
    pdata.intensity[i] = this->laserMsg.intensities[i];
  pdata.id = this->laserMsg.header.seq;

  double timestamp = this->laserMsg.header.stamp_secs + 
          this->laserMsg.header.stamp_nsecs / 1e9;

  this->Driver::Publish(this->laser_addr,
                        PLAYER_MSGTYPE_DATA,
                        PLAYER_LASER_DATA_SCAN,
                        (void*)&pdata,0,
                        &timestamp);

  delete[] pdata.ranges;
  delete[] pdata.intensity;
}

void
AmclNode::odomReceived()
{
  // Got new odom; reformat and pass it on
  player_position2d_data_t pdata;
  pdata.pos.px = this->odomMsg.pos.x;
  pdata.pos.py = this->odomMsg.pos.y;
  pdata.pos.pa = this->odomMsg.pos.th;
  pdata.vel.px = this->odomMsg.vel.x;
  pdata.vel.py = this->odomMsg.vel.y;
  pdata.vel.pa = this->odomMsg.vel.th;
  pdata.stall = this->odomMsg.stall;

  double timestamp = this->odomMsg.header.stamp_secs + 
          this->odomMsg.header.stamp_nsecs / 1e9;

  this->Driver::Publish(this->position2d_addr,
                        PLAYER_MSGTYPE_DATA,
                        PLAYER_POSITION2D_DATA_STATE,
                        (void*)&pdata,0,
                        &timestamp,true);
}

int
read_map_from_image(int* size_x, int* size_y, char** mapdata, 
                    const char* fname, int negate)
{
  GdkPixbuf* pixbuf;
  guchar* pixels;
  guchar* p;
  int rowstride, n_channels, bps;
  GError* error = NULL;
  int i,j,k;
  double occ;
  int color_sum;
  double color_avg;

  // Initialize glib
  g_type_init();

  printf("MapFile loading image file: %s...", fname);
  fflush(stdout);

  // Read the image
  if(!(pixbuf = gdk_pixbuf_new_from_file(fname, &error)))
  {
    printf("failed to open image file %s", fname);
    return(-1);
  }

  *size_x = gdk_pixbuf_get_width(pixbuf);
  *size_y = gdk_pixbuf_get_height(pixbuf);

  assert(*mapdata = (char*)malloc(sizeof(char) * (*size_x) * (*size_y)));

  rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  //if(gdk_pixbuf_get_has_alpha(pixbuf))
    //n_channels++;

  // Read data
  pixels = gdk_pixbuf_get_pixels(pixbuf);
  for(j = 0; j < *size_y; j++)
  {
    for (i = 0; i < *size_x; i++)
    {
      p = pixels + j*rowstride + i*n_channels*bps;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k * bps));
      color_avg = color_sum / (double)n_channels;

      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      if(occ > 0.95)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = +1;
      else if(occ < 0.1)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = -1;
      else
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = 0;
    }
  }

  gdk_pixbuf_unref(pixbuf);

  puts("Done.");
  printf("MapFile read a %d X %d map\n", *size_x, *size_y);
  return(0);
}
