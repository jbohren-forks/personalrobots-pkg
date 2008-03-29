#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/ros_slave.h>
// Flows that I need
#include <common_flows/FlowRobotBase2DCmdVel.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* LinuxJoystick_Init(ConfigFile* cf, int section);

class LinuxJoystickNode: public ROS_Slave
{
  public:
    QueuePointer q;

    FlowRobotBase2DCmdVel* cmdvel;

    LinuxJoystickNode() : ROS_Slave()
    {
      // libplayercore boiler plate
      player_globals_init();
      itable_init();
      
      // get more debug info from underlying driver
      ErrorInit(9);
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      // The Player address that will be assigned to this device.  The format
      // is interface:index.  The interface must match what the driver is
      // expecting to provide.  The value of the index doesn't really matter, 
      // but 0 is most common.
      const char* player_addr = "joystick:0";

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();

      // Insert (name,value) pairs into the ConfigFile object.  These would
      // presumably come from the param server
      this->cf->InsertFieldValue(0,"provides",player_addr);
      this->cf->InsertFieldValue(0,"port","/dev/input/js0");

      // Create an instance of the driver, passing it the ConfigFile object.
      // The -1 tells it to look into the "global" section of the ConfigFile,
      // which is where ConfigFile::InsertFieldValue() put the parameters.
      assert((this->driver = LinuxJoystick_Init(cf, -1)));

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->device = deviceTable->GetDevice(player_addr,false)));

      // Create a message queue
      this->q = QueuePointer(false,PLAYER_QUEUE_LEN);

      this->register_source(this->cmdvel = new FlowRobotBase2DCmdVel("cmdvel"));
    }

    ~LinuxJoystickNode()
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

    // TODO: move the hardcoded stuff from here to param server, e.g.:
    //   - axis assignment
    //   - axes max / min
    void putPositionCommand(player_joystick_data_t* pdata)
    {
      double scaled[3];
      double speed[3];
      //struct timeval curr;
      //double diff;
      int axes_min[3] = {5000, 5000, 5000};
      int axes_max[3] = {32767, 32767, 32767};
      double max_speed[3] = {0.5, 0.0, DTOR(30.0)};

      for(int i=0;i<3;i++)
      {
        if(abs(pdata->pos[i]) < axes_min[i])
          scaled[i] = 0;
        else
          scaled[i] = pdata->pos[i] / (double) axes_max[i];
      }

      // sanity check
      if((scaled[0] > 1.0) || (scaled[0] < -1.0))
      {
        PLAYER_ERROR2("X position (%d) outside of axis max (+-%d); ignoring", 
                      pdata->pos[0], axes_max[0]);
        return;
      }
      if((scaled[1] > 1.0) || (scaled[1] < -1.0))
      {
        PLAYER_ERROR2("Y position (%d) outside of axis max (+-%d); ignoring", 
                      pdata->pos[1], axes_max[1]);
        return;

      }
      if((scaled[2] > 1.0) || (scaled[2] < -1.0))
      {
        PLAYER_ERROR2("Yaw position (%d) outside of axis max (+-%d); ignoring", 
                      pdata->pos[2], axes_max[2]);
        return;
      }

      // As joysticks axes are backwards with respect to intuitive driving
      // controls, we invert the values here.
      speed[0] = -scaled[0] * max_speed[0];
      speed[1] = -scaled[1] * max_speed[1];
      speed[2] = -scaled[2] * max_speed[2];

#if 0
      // Make sure we've gotten a joystick fairly recently.
      GlobalTime->GetTime(&curr);
      diff = (curr.tv_sec - curr.tv_usec/1e6) -
              (this->lastread.tv_sec - this->lastread.tv_usec/1e6);
      if(this->timeout && (diff > this->timeout) && (speed[0] || speed[1] || speed[2]))
      {
        PLAYER_WARN("Timeout on joystick; stopping robot");
        speed[0] = speed[1] = speed[2] = 0.0;
      }
#endif

      PLAYER_MSG3(2,"sending speeds: (%f,%f,%f)", speed[0], speed[1], speed[2]);

      this->cmdvel->vx = speed[0];
      this->cmdvel->vy = speed[1];
      this->cmdvel->vyaw = speed[2];
      this->cmdvel->publish();
    }

  private:
    Driver* driver;
    Device* device;
    ConfigFile* cf;
};

int
main(void)
{
  LinuxJoystickNode jn;
  Message* msg;

  // Start up the robot
  if(jn.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
  {
    // Block until there's a message on the queue
    jn.q->Wait();

    // Pop off one message (we own the resulting memory)
    assert((msg = jn.q->Pop()));

    // Is the message one we care about?
    player_msghdr_t* hdr = msg->GetHeader();
    if((hdr->type == PLAYER_MSGTYPE_DATA) && 
       (hdr->subtype == PLAYER_JOYSTICK_DATA_STATE))
    {
      // Cast the message payload appropriately 
      player_joystick_data_t* pdata = (player_joystick_data_t*)msg->GetPayload();
      jn.putPositionCommand(pdata);
    }
    else
    {
      printf("%d:%d:%d:%d\n",
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
  jn.stop();

  // To quote Morgan, Hooray!
  return(0);
}
