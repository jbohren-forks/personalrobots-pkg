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

class DummyDriver : public Driver
{
  int Setup() {}
  int Shutdown() {}
  int ProcessMessage() {}
}

class AmclNode: public ros::node
{
  public:
    QueuePointer q;

    MsgRobotBase2DOdom odom;

    AmclNode() : ros::node("erratic")
    {
      advertise("odom", odom);

      // libplayercore boiler plate
      player_globals_init();
      itable_init();
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      // The Player address that will be assigned to this device.  The format
      // is interface:index.  The interface must match what the driver is
      // expecting to provide.  The value of the index doesn't really matter, 
      // but 0 is most common.
      const char* player_addr = "position2d:0";

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();

      // Insert (name,value) pairs into the ConfigFile object.  These would
      // presumably come from the param server
      this->cf->InsertFieldValue(0,"provides",player_addr);
      this->cf->InsertFieldValue(0,"port","/dev/ttyUSB0");

      // Create an instance of the driver, passing it the ConfigFile object.
      // The -1 tells it to look into the "global" section of the ConfigFile,
      // which is where ConfigFile::InsertFieldValue() put the parameters.
      assert((this->driver = AdaptiveMCL_Init(cf, -1)));

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->device = deviceTable->GetDevice(player_addr,false)));

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

  private:
    Driver* driver;
    Device* device;
    ConfigFile* cf;
};

int
main(void)
{
  AmclNode an;
  Message* msg;

  // Start up the robot
  if(an.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
  {
    // Block until there's a message on the queue
    an.q->Wait();

    // Pop off one message (we own the resulting memory)
    assert((msg = an.q->Pop()));

    // Is the message one we care about?
    player_msghdr_t* hdr = msg->GetHeader();
    if((hdr->type == PLAYER_MSGTYPE_DATA) && 
       (hdr->subtype == PLAYER_POSITION2D_DATA_STATE))
    {
      // Cast the message payload appropriately 
      player_position2d_data_t* pdata = (player_position2d_data_t*)msg->GetPayload();
      
      // Translate from Player data to ROS data
      an.odom.px = pdata->pos.px;
      an.odom.py = pdata->pos.py;
      an.odom.pyaw = pdata->pos.pa;
      an.odom.vx = pdata->vel.px;
      an.odom.vy = pdata->vel.py;
      an.odom.vyaw = pdata->vel.pa;
      an.odom.stall = pdata->stall;

      // Publish the new data
      publish("odom", odom);

      printf("Published new odom: (%.3f,%.3f,%.3f)\n", 
             an.odom.px, an.odom.py, an.odom.pyaw);
    }
    else
    {
      printf("Unhandled Player message %d:%d:%d:%d\n",
             hdr->type,
             hdr->subtype,
             hdr->addr.interf,
             hdr->addr.index);

    }

    // We're done with the message now
    delete msg;
  }
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  an.stop();

  // To quote Morgan, Hooray!
  return(0);
}
