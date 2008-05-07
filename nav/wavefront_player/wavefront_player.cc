#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/node.h>
// Messages that I need
#include <std_msgs/MsgPlanner2D.h>
#include <std_msgs/MsgOccMap2D.h>

#define PLAYER_QUEUE_LEN 32

// TODO: remove this code when we can get the map via RPC
#include <gdk-pixbuf/gdk-pixbuf.h>
int
read_map_from_image(int* size_x, int* size_y, char** mapdata, 
                    const char* fname, int negate);

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* Wavefront_Init( ConfigFile* cf, int section);

class WavefrontNode: public ros::node, public Driver
{
  public:
    WavefrontNode();
    ~WavefrontNode();

    int Setup() {return(0);}
    int Shutdown() {return(0);}
    int ProcessMessage(QueuePointer &resp_queue, 
                       player_msghdr * hdr,
                       void * data);

    int start();
    int stop();

    int process();

  private:
    ConfigFile* cf;

    // These are the devices that wavefront offers, and to which we subscribe
    Driver* driver;
    Device* device;
    player_devaddr_t planner_addr;

    // These are the devices that wavefront requires, and so which we must
    // provide
    player_devaddr_t position2d_addr;
    player_devaddr_t laser_addr;
    player_devaddr_t map_addr;
    Device* position2d_dev;
    Device* laser_dev;
    Device* map_dev;

    MsgPlanner2D plandata;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  WavefrontNode wn;

  // Start up the robot
  if(wn.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
    wn.process();
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  wn.stop();

  // To quote Morgan, Hooray!
  return(0);
}

WavefrontNode::WavefrontNode() : 
        ros::node("erratic"), 
        Driver(NULL,-1,false,PLAYER_QUEUE_LEN)
{
  this->ros::node::advertise<MsgPlanner2D>("plandata");

  // libplayercore boiler plate
  player_globals_init();
  itable_init();

  // TODO: remove XDR dependency
  playerxdr_ftable_init();

  // TODO: automatically convert between string and player_devaddr_t
  // representations

  // The Player address that will be assigned to this device.  The format
  // is interface:index.  The interface must match what the driver is
  // expecting to provide.  The value of the index doesn't really matter, 
  // but 0 is most common.
  const char* planner_saddr = "planner:0";
  this->planner_addr.host = 0;
  this->planner_addr.robot = 0;
  this->planner_addr.interf = PLAYER_PLANNER_CODE;
  this->planner_addr.index = 0;

  const char* out_position2d_saddr = "output:::position2d:0";
  const char* in_position2d_saddr = "input:::position2d:0";
  this->position2d_addr.host = 0;
  this->position2d_addr.robot = 0;
  this->position2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->position2d_addr.index = 0;

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
  this->cf->InsertFieldValue(0,"provides",planner_saddr);

  // Fill in the requires fields for the device that this device
  // subscribes to
  this->cf->InsertFieldValue(0,"requires",out_position2d_saddr);
  this->cf->InsertFieldValue(1,"requires",in_position2d_saddr);
  this->cf->InsertFieldValue(2,"requires",laser_saddr);
  this->cf->InsertFieldValue(3,"requires",map_saddr);
  this->cf->DumpTokens();

  // Create an instance of the driver, passing it the ConfigFile object.
  // The -1 tells it to look into the "global" section of the ConfigFile,
  // which is where ConfigFile::InsertFieldValue() put the parameters.
  assert((this->driver = Wavefront_Init(cf, -1)));

  // Print out warnings about parameters that were set, but which the
  // driver never looked at.
  cf->WarnUnused();

  // Grab from the global deviceTable a pointer to the Device that was 
  // created as part of the driver's initialization.
  assert((this->device = deviceTable->GetDevice(planner_addr,false)));
}

WavefrontNode::~WavefrontNode()
{
  delete driver;
  delete cf;
  player_globals_fini();
}

int 
WavefrontNode::ProcessMessage(QueuePointer &resp_queue, 
                         player_msghdr * hdr,
                         void * data)
{
  printf("Player message %d:%d:%d:%d\n",
         hdr->type,
         hdr->subtype,
         hdr->addr.interf,
         hdr->addr.index);

  // Is it a new planner data?
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_DATA, 
                           PLAYER_PLANNER_DATA_STATE,
                           this->planner_addr))
  {
    /*
    // Cast the message payload appropriately 
    player_position2d_data_t* pdata = 
            (player_position2d_data_t*)data;

    // Translate from Player data to ROS data
    this->odom.px = pdata->pos.px;
    this->odom.py = pdata->pos.py;
    this->odom.pyaw = pdata->pos.pa;
    this->odom.vx = pdata->vel.px;
    this->odom.vy = pdata->vel.py;
    this->odom.vyaw = pdata->vel.pa;
    this->odom.stall = pdata->stall;

    // Publish the new data
    this->ros::node::publish("odom", this->odom);

    printf("Published new odom: (%.3f,%.3f,%.3f)\n", 
           this->odom.px, this->odom.py, this->odom.pyaw);
           */
    return(0);
  }
  // Is it a request for the robot geometry?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_POSITION2D_REQ_GET_GEOM,
                                this->position2d_addr))
  {
    // TODO: get this data via ROSRPC
    player_position2d_geom_t geom;
    memset(&geom,0,sizeof(player_position2d_geom_t));
    geom.size.sw = 0.5;
    geom.size.sl = 0.5;
    geom.size.sh = 0.25;

    this->Publish(this->position2d_addr, resp_queue,
                  PLAYER_MSGTYPE_RESP_ACK,
                  PLAYER_POSITION2D_REQ_GET_GEOM,
                  (void*)&geom);
  }
  // Is it a request for the map metadata?
  else if(Message::MatchMessage(hdr,
                                PLAYER_MSGTYPE_REQ, 
                                PLAYER_MAP_REQ_GET_INFO,
                                this->map_addr))
  {
    player_map_info_t info;
    info.scale = 0.1;
    info.width = 100;
    info.height = 100;
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
    // Send a NACK
    return(-1);
  }
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
WavefrontNode::start()
{
  // Subscribe to device, which causes it to startup
  if(this->device->Subscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to subscribe the driver");
    return(-1);
  }
  else
    return(0);
}

int 
WavefrontNode::stop()
{
  // Unsubscribe from the device, which causes it to shutdown
  if(device->Unsubscribe(this->Driver::InQueue) != 0)
  {
    puts("Failed to start the driver");
    return(-1);
  }
  else
    return(0);
}

int 
WavefrontNode::process()
{
  // Block until there's a message on our queue
  this->Driver::InQueue->Wait();

  this->Driver::ProcessMessages();

  return(0);
}

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

// TODO: remove this code when we can get the map via RPC
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
