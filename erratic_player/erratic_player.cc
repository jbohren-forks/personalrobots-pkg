#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/ros_slave.h>
// I'm using a RobotBase2D flow
#include <common_flows/FlowRobotBase2D.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* Erratic_Init(ConfigFile* cf, int section);

class ErraticNode: public ROS_Slave
{
  public:
    QueuePointer q;

    ErraticNode() : ROS_Slave()
    {
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
      assert((this->driver = Erratic_Init(cf, -1)));

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->device = deviceTable->GetDevice(player_addr,false)));

      // Create a message queue
      this->q = QueuePointer(false,PLAYER_QUEUE_LEN);
    }

    ~ErraticNode()
    {
      delete driver;
      delete cf;
      player_globals_fini();
    }

    int Start()
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

    int Stop()
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
  ErraticNode en;
  Message* msg;

  // Start up the robot
  if(en.Start() != 0)
    exit(-1);

  // Declare a flow
  FlowRobotBase2D fl(".odometry");

#if 0
  // Enable the motors
  player_position2d_power_config_t motorconfig;
  motorconfig.state = 1;
  if(!(msg = this->position->Request(this->InQueue,
                                     PLAYER_MSGTYPE_REQ,
                                     PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                     (void*)&motorconfig,
                                     sizeof(motorconfig), NULL, false)))
  {
    PLAYER_WARN("failed to enable motors");
  }
  else
    delete msg;
#endif

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
  {
    // Block until there's a message on the queue
    en.q->Wait();

    // Pop off one message (we own the resulting memory)
    assert((msg = en.q->Pop()));

    // Is the message one we care about?
    player_msghdr_t* hdr = msg->GetHeader();
    if((hdr->type == PLAYER_MSGTYPE_DATA) && 
       (hdr->subtype == PLAYER_POSITION2D_DATA_STATE))
    {
      // Cast the message payload appropriately 
      player_position2d_data_t* pdata = (player_position2d_data_t*)msg->GetPayload();
      
      // Translate from Player data to ROS data
      fl.px = pdata->pos.px;
      fl.py = pdata->pos.py;
      fl.pyaw = pdata->pos.pa;
      fl.vx = pdata->vel.px;
      fl.vy = pdata->vel.py;
      fl.vyaw = pdata->vel.pa;
      fl.stall = pdata->stall;

      // Publish the new data
      fl.publish();

      printf("Published new odom: (%.3f,%.3f,%.3f)\n", fl.px,fl.py,fl.pyaw);
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
  en.Stop();

  // To quote Morgan, Hooray!
  return(0);
}
