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

/*template<class N>
class DummyDriver : public Driver
{
  private:
    int (N::*cb)(QueuePointer&, player_msghdr*, void*);
  public:
    DummyDriver(N* node, int (N::*fp)(QueuePointer&, player_msghdr*, void*),
                player_devaddr_t* addrs, size_t len) :
            Driver(NULL, -1, true, PLAYER_QUEUE_LEN)
    {
      this->cb = fp;
      for(int i=0;i<len;i++)
      {
        if(this->AddInterface(addrs[i]))
        {
          this->SetError(-1);
          return;
        }
      }
    }
    int Setup() { return(0); }
    int Shutdown() { return(0); }
    int ProcessMessage(QueuePointer& q, player_msghdr* h, void* d)
    { 
      return((this->cb)(q,h,d));
    }
};
*/

class AmclNode: public ros::node
{
  public:
    QueuePointer q;

    player_devaddr_t position2d_addr;
    player_devaddr_t laser_addr;
    Device* position2d_dev;
    Device* laser_dev;

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
      const char* player_saddr = "position2d:0";

      // TODO: automatically convert between string and player_devaddr_t
      // representations
      const char* position2d_saddr = "odometery:::position2d:1";
      const char* laser_saddr = "laser:0";
      this->position2d_addr.host = 0;
      this->position2d_addr.robot = 0;
      this->position2d_addr.interf = PLAYER_POSITION2D_CODE;
      this->position2d_addr.index = 1;
      this->laser_addr.host = 0;
      this->laser_addr.robot = 0;
      this->laser_addr.interf = PLAYER_LASER_CODE;
      this->laser_addr.index = 0;

      this->position2d_dev = deviceTable->AddDevice(this->position2d_addr, 
                                                    NULL, false);
      assert(this->position2d_dev);
      this->laser_dev = deviceTable->AddDevice(this->laser_addr, 
                                               NULL, false);
      assert(this->laser_dev);

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();

      // Insert (name,value) pairs into the ConfigFile object.  These would
      // presumably come from the param server
      this->cf->InsertFieldValue(0,"provides",player_saddr);
      this->cf->InsertFieldValue(0,"port","/dev/ttyUSB0");

      this->cf->InsertFieldValue(0,"requires",position2d_saddr);
      this->cf->InsertFieldValue(0,"requires",laser_saddr);

      // Create an instance of the driver, passing it the ConfigFile object.
      // The -1 tells it to look into the "global" section of the ConfigFile,
      // which is where ConfigFile::InsertFieldValue() put the parameters.
      assert((this->driver = AdaptiveMCL_Init(cf, -1)));

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->device = deviceTable->GetDevice(player_saddr,false)));

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
main(int argc, char** argv)
{
  ros::init(argc, argv);

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
      an.publish("odom", an.odom);

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
