#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/ros_slave.h>
// I'm using a LaserScan flow
#include <common_flows/FlowLaserScan.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* URGLaserDriver_Init(ConfigFile* cf, int section);

class HokuyoNode: public ROS_Slave
{
  public:
    QueuePointer q;

    FlowLaserScan* fl;

    HokuyoNode() : ROS_Slave()
    {
      register_source(fl = new FlowLaserScan("scans"));


      // libplayercore boiler plate
      player_globals_init();
      itable_init();
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      // The Player address that will be assigned to this device.  The format
      // is interface:index.  The interface must match what the driver is
      // expecting to provide.  The value of the index doesn't really matter, 
      // but 0 is most common.
      const char* player_addr = "laser:0";

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();

      // Insert (name,value) pairs into the ConfigFile object.  These would
      // presumably come from the param server
      this->cf->InsertFieldValue(0,"provides",player_addr);
      this->cf->InsertFieldValue(0,"unit_angle","degrees");
      this->cf->InsertFieldValue(0,"min_angle","-90");
      this->cf->InsertFieldValue(0,"max_angle","90");

      string port;
      if (!get_string_param(".port", port))
	port = "/dev/ttyACM0";

      printf("Setting port to: %s\n",port.c_str());

      this->cf->InsertFieldValue(0,"port",port.c_str());

      // Create an instance of the driver, passing it the ConfigFile object.
      // The -1 tells it to look into the "global" section of the ConfigFile,
      // which is where ConfigFile::InsertFieldValue() put the parameters.
      assert((this->driver = URGLaserDriver_Init(cf, -1)));

      // Print out warnings about parameters that were set, but which the
      // driver never looked at.
      cf->WarnUnused();

      // Grab from the global deviceTable a pointer to the Device that was 
      // created as part of the driver's initialization.
      assert((this->device = deviceTable->GetDevice(player_addr,false)));

      // Create a message queue
      this->q = QueuePointer(false,PLAYER_QUEUE_LEN);

    }

    ~HokuyoNode()
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
  HokuyoNode hn;

  // Start up the laser
  if(hn.start() != 0)
    exit(-1);

  /////////////////////////////////////////////////////////////////
  // Main loop; grab messages off our queue and republish them via ROS
  for(;;)
  {
    // Block until there's a message on the queue
    hn.q->Wait();

    // Pop off one message (we own the resulting memory)
    Message* msg;
    assert((msg = hn.q->Pop()));

    // Is the message a laser scan?
    player_msghdr_t* hdr = msg->GetHeader();
    if((hdr->type == PLAYER_MSGTYPE_DATA) && 
       (hdr->subtype == PLAYER_LASER_DATA_SCAN))
    {
      // Cast the message payload appropriately 
      player_laser_data_t* pdata = (player_laser_data_t*)msg->GetPayload();
      
      // Translate from Player data to ROS data
      hn.fl->px = hn.fl->py = hn.fl->pyaw = 0.0;
      hn.fl->angle_min = pdata->min_angle;
      hn.fl->angle_max = pdata->max_angle;
      hn.fl->angle_increment = pdata->resolution;
      hn.fl->set_ranges_size(pdata->ranges_count);
      for(unsigned int i=0;i<pdata->ranges_count;i++)
        hn.fl->ranges[i] = pdata->ranges[i];
      hn.fl->set_intensities_size(pdata->intensity_count);
      for(unsigned int i=0;i<pdata->intensity_count;i++)
        hn.fl->intensities[i] = pdata->intensity[i];

      // Publish the new data
      hn.fl->publish();

      printf("Published a scan with %d ranges\n", pdata->ranges_count);
    }

    // We're done with the message now
    delete msg;
  }
  /////////////////////////////////////////////////////////////////

  // Stop the laser
  hn.stop();

  // To quote Morgan, Hooray!
  return(0);
}
