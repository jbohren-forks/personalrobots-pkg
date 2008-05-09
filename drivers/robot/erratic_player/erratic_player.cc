#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/node.h>
// Messages that I need
#include <std_msgs/MsgRobotBase2DOdom.h>
//#include <std_msgs/MsgRobotBase2DCmdVel.h>
#include <std_msgs/MsgBaseVel.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* Erratic_Init(ConfigFile* cf, int section);

class ErraticNode: public ros::node
{
  public:
    QueuePointer q;

    MsgRobotBase2DOdom odom;
    //MsgRobotBase2DCmdVel cmdvel;
    MsgBaseVel cmdvel;

    ErraticNode() : ros::node("erratic_player")
    {
      // libplayercore boiler plate
      player_globals_init();
      puts("init");
      itable_init();
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      advertise<MsgRobotBase2DOdom>("odom");

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
      {
        subscribe("cmd_vel", cmdvel, &ErraticNode::cmdvelReceived);
        return(0);
      }
    }

    int stop()
    {
      // Unsubscribe from the device, which causes it to shutdown
      if(this->device->Unsubscribe(this->q) != 0)
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

    int setMotorState(uint8_t state)
    {
      Message* msg;
      // Enable the motors
      player_position2d_power_config_t motorconfig;
      motorconfig.state = state;
      if(!(msg = this->device->Request(this->q,
                                       PLAYER_MSGTYPE_REQ,
                                       PLAYER_POSITION2D_REQ_MOTOR_POWER,
                                       (void*)&motorconfig,
                                       sizeof(motorconfig), NULL, false)))
      {
        return(-1);
      }
      else
      {
        delete msg;
        return(0);
      }
    }

    void cmdvelReceived()
    {
      /*
      printf("received cmd: (%.3f,%.3f,%.3f)\n",
             this->cmdvel.vel.x,
             this->cmdvel.vel.y,
             this->cmdvel.vel.th);
             */

      player_position2d_cmd_vel_t cmd;
      memset(&cmd, 0, sizeof(cmd));

      //cmd.vel.px = this->cmdvel.vel.x;
      //cmd.vel.py = this->cmdvel.vel.y;
      //cmd.vel.pa = this->cmdvel.vel.th;
      cmd.vel.px = this->cmdvel.vx;
      cmd.vel.py = 0.0;
      cmd.vel.pa = this->cmdvel.vw;
      cmd.state = 1;


      this->device->PutMsg(this->q,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           (void*)&cmd,sizeof(cmd),NULL);
    }

  private:
    Driver* driver;
    Device* device;
    ConfigFile* cf;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv);

  ErraticNode en;
  Message* msg;

  // Start up the robot
  if(en.start() != 0)
    exit(-1);

  // Enable the motors
  if(en.setMotorState(1) < 0)
    puts("failed to enable motors");

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  while(en.ok())
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
      en.odom.pos.x = pdata->pos.px;
      en.odom.pos.y = pdata->pos.py;
      en.odom.pos.th = pdata->pos.pa;
      en.odom.vel.x = pdata->vel.px;
      en.odom.vel.y = pdata->vel.py;
      en.odom.vel.th = pdata->vel.pa;
      en.odom.stall = pdata->stall;

      en.odom.header.frame_id = 2;

      // Publish the new data
      en.publish("odom", en.odom);

      //printf("Published new odom: (%.3f,%.3f,%.3f)\n", 
             //en.odom.pos.x, en.odom.pos.y, en.odom.pos.th);
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
  en.stop();

  // To quote Morgan, Hooray!
  return(0);
}
