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

class AmclNode: public ros::node
{
  public:
    AmclNode() : ros::node("erratic")
    {
      advertise("odom", odom);

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
      this->position2d_dev->InQueue = QueuePointer(this->q);
      this->laser_dev = deviceTable->AddDevice(this->laser_addr, 
                                               NULL, false);
      assert(this->laser_dev);
      this->laser_dev->InQueue = QueuePointer(this->q);
      this->map_dev = deviceTable->AddDevice(this->map_addr, 
                                             NULL, false);
      assert(this->map_dev);
      this->map_dev->InQueue = QueuePointer(this->q);

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

      // Create a message queue
      this->q = QueuePointer(false,PLAYER_QUEUE_LEN);
    }

    ~AmclNode()
    {
      delete driver;
      delete cf;
      player_globals_fini();
    }

    int start()
    {
      // Subscribe to device, which causes it to startup
      if(this->device->Subscribe(this->q) != 0)
      {
        puts("Failed to subscribe the driver");
        return(-1);
      }
      else
        return(0);
    }

    int stop()
    {
      // Unsubscribe from the device, which causes it to shutdown
      if(device->Unsubscribe(this->q) != 0)
      {
        puts("Failed to start the driver");
        return(-1);
      }
      else
        return(0);
    }

    int process()
    {
      // Block until there's a message on our queue
      this->q->Wait();

      Message* msg;

      // Pop off one message (we own the resulting memory)
      assert((msg = this->q->Pop()));

      player_msghdr_t hdr = *msg->GetHeader();

      // Is it a new pose from amcl?
      if(Message::MatchMessage(&hdr,
                               PLAYER_MSGTYPE_DATA, 
                               PLAYER_POSITION2D_DATA_STATE,
                               this->oposition2d_addr))
      {
        // Cast the message payload appropriately 
        player_position2d_data_t* pdata = 
                (player_position2d_data_t*)msg->GetPayload();

        // Translate from Player data to ROS data
        this->odom.px = pdata->pos.px;
        this->odom.py = pdata->pos.py;
        this->odom.pyaw = pdata->pos.pa;
        this->odom.vx = pdata->vel.px;
        this->odom.vy = pdata->vel.py;
        this->odom.vyaw = pdata->vel.pa;
        this->odom.stall = pdata->stall;

        // Publish the new data
        this->publish("odom", this->odom);

        printf("Published new odom: (%.3f,%.3f,%.3f)\n", 
               this->odom.px, this->odom.py, this->odom.pyaw);
      }
      // Is it a request for the map metadata?
      else if(Message::MatchMessage(&hdr,
                                    PLAYER_MSGTYPE_REQ, 
                                    PLAYER_MAP_REQ_GET_INFO,
                                    this->map_addr))
      {
        // Send a NACK
        hdr.type = PLAYER_MSGTYPE_RESP_NACK;
        this->device->PutMsg(this->q, &hdr, NULL);
      }
      // Is it a request for a map tile?
      else if(Message::MatchMessage(&hdr,
                                    PLAYER_MSGTYPE_REQ, 
                                    PLAYER_MAP_REQ_GET_DATA,
                                    this->map_addr))
      {
        // Send a NACK
        hdr.type = PLAYER_MSGTYPE_RESP_NACK;
        this->device->PutMsg(this->q, &hdr, NULL);
      }
      // Is some other request?
      else if(hdr.type == PLAYER_MSGTYPE_REQ)
      {
        // Send a NACK
        hdr.type = PLAYER_MSGTYPE_RESP_NACK;
        this->device->PutMsg(this->q, &hdr, NULL);
      }
      else
      {
        printf("Unhandled Player message %d:%d:%d:%d\n",
               hdr.type,
               hdr.subtype,
               hdr.addr.interf,
               hdr.addr.index);
      }

      // We're done with the message now
      delete msg;

      return(0);
    }

  private:
    ConfigFile* cf;

    // This is "our" queue, which we will subscribe to the underlying amcl
    // driver.
    QueuePointer q;

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
