#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/node.h>
// Messages that I need
#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgRobotBase2DCmdVel.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* AdaptiveMCL_Init(ConfigFile* cf, int section);

class AmclNode: public ros::node, public Driver
{
  public:
    AmclNode();
    ~AmclNode();

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

    // These are the devices that amcl offers, and to which we subscribe
    Driver* driver;
    Device* device;
    player_devaddr_t oposition2d_addr;

    // These are the devices that amcl requires, and so which we must
    // provide
    player_devaddr_t position2d_addr;
    player_devaddr_t laser_addr;
    player_devaddr_t map_addr;
    Device* position2d_dev;
    Device* laser_dev;
    Device* map_dev;

    MsgRobotBase2DOdom odom;

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  AmclNode an;

  // Start up the robot
  if(an.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
    an.process();
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  an.stop();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() : 
        ros::node("erratic"), 
        Driver(NULL,-1,false,PLAYER_QUEUE_LEN)
{
  this->ros::node::advertise<MsgRobotBase2DOdom>("odom");

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
  const char* oposition2d_saddr = "position2d:0";
  this->oposition2d_addr.host = 0;
  this->oposition2d_addr.robot = 0;
  this->oposition2d_addr.interf = PLAYER_POSITION2D_CODE;
  this->oposition2d_addr.index = 0;

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

  // Fill in the requires fields for the device that this device
  // subscribes to
  this->cf->InsertFieldValue(0,"requires",position2d_saddr);
  this->cf->InsertFieldValue(1,"requires",laser_saddr);
  this->cf->InsertFieldValue(2,"requires",map_saddr);
  this->cf->DumpTokens();

  // Create an instance of the driver, passing it the ConfigFile object.
  // The -1 tells it to look into the "global" section of the ConfigFile,
  // which is where ConfigFile::InsertFieldValue() put the parameters.
  assert((this->driver = AdaptiveMCL_Init(cf, -1)));

  // Print out warnings about parameters that were set, but which the
  // driver never looked at.
  cf->WarnUnused();

  // Grab from the global deviceTable a pointer to the Device that was 
  // created as part of the driver's initialization.
  assert((this->device = deviceTable->GetDevice(oposition2d_saddr,false)));
}

AmclNode::~AmclNode()
{
  delete driver;
  delete cf;
  player_globals_fini();
}

int 
AmclNode::ProcessMessage(QueuePointer &resp_queue, 
                         player_msghdr * hdr,
                         void * data)
{
  printf("Player message %d:%d:%d:%d\n",
         hdr->type,
         hdr->subtype,
         hdr->addr.interf,
         hdr->addr.index);

  // Is it a new pose from amcl?
  if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_DATA, 
                           PLAYER_POSITION2D_DATA_STATE,
                           this->oposition2d_addr))
  {
    // Cast the message payload appropriately 
    player_position2d_data_t* pdata = 
            (player_position2d_data_t*)data;

    // Translate from Player data to ROS data
    this->odom.pos.x = pdata->pos.px;
    this->odom.pos.y = pdata->pos.py;
    this->odom.pos.th = pdata->pos.pa;
    this->odom.vel.x = pdata->vel.px;
    this->odom.vel.y = pdata->vel.py;
    this->odom.vel.th = pdata->vel.pa;
    this->odom.stall = pdata->stall;

    // Publish the new data
    this->ros::node::publish("odom", this->odom);

    printf("Published new odom: (%.3f,%.3f,%.3f)\n", 
           this->odom.pos.x, this->odom.pos.y, this->odom.pos.th);
    return(0);
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
AmclNode::start()
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
AmclNode::stop()
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
AmclNode::process()
{
  // Block until there's a message on our queue
  this->Driver::InQueue->Wait();

  this->Driver::ProcessMessages();

  return(0);
}
