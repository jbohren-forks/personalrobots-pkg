/*
 * erratic_player
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

/**

@mainpage

@htmlinclude manifest.html

@b erratic_player is a driver for the Erratic mobile robot, available from
<a href="http://www.videredesign.com">Videre Design</a>.

This node wraps up the Player @b erratic driver.  For detailed documentation,
consult <a href="http://playerstage.sourceforge.net/doc/Player-cvs/player/group__driver__erratic.html">Player erratic documentation</a>.

<hr>

@section usage Usage
@verbatim
$ erratic_player [standard ROS args]
@endverbatim

@par Example

@verbatim
$ erratic_player
@endverbatim

<hr>

@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/BaseVel : velocity commands to differentially drive the robot.

Publishes to (name / type):
- @b "odom"/RobotBase2DOdom : odometry data from the robot.

<hr>

@section parameters ROS parameters

- None

@todo Expose the various erratic parameters via ROS.

 **/

#include <assert.h>

// For core Player stuff (message queues, config file objects, etc.)
#include <libplayercore/playercore.h>
// TODO: remove XDR dependency
#include <libplayerxdr/playerxdr.h>

// roscpp
#include <ros/node.h>
//rosTF
#include "tf/transform_broadcaster.h"
// Messages that I need
#include <std_msgs/RobotBase2DOdom.h>
//#include <std_msgs/RobotBase2DCmdVel.h>
#include <std_msgs/BaseVel.h>

#define PLAYER_QUEUE_LEN 32

// Must prototype this function here.  It's implemented inside
// libplayerdrivers.
Driver* Erratic_Init(ConfigFile* cf, int section);

class ErraticNode: public ros::Node
{
  public:
    QueuePointer q;

    std_msgs::RobotBase2DOdom odom;
    //RobotBase2DCmdVel cmdvel;
    std_msgs::BaseVel cmdvel;

  tf::TransformBroadcaster tf;
  
  ErraticNode() : ros::Node("erratic_player"),
		  tf(*this)
    {
      // libplayercore boiler plate
      player_globals_init();
      puts("init");
      itable_init();
      
      // TODO: remove XDR dependency
      playerxdr_ftable_init();

      advertise<std_msgs::RobotBase2DOdom>("odom", 1);

      // The Player address that will be assigned to this device.  The format
      // is interface:index.  The interface must match what the driver is
      // expecting to provide.  The value of the index doesn't really matter, 
      // but 0 is most common.
      const char* player_addr_pos = "position2d:0";
      const char* player_addr_power = "power:0";

      // Create a ConfigFile object, into which we'll stuff parameters.
      // Drivers assume that this object will persist throughout execution
      // (e.g., they store pointers to data inside it).  So it must NOT be
      // deleted until after the driver is shut down.
      this->cf = new ConfigFile();

      // Insert (name,value) pairs into the ConfigFile object.  These would
      // presumably come from the param server
      this->cf->InsertFieldValue(0,"provides",player_addr_pos);
      this->cf->InsertFieldValue(1,"provides",player_addr_power);
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
      assert((this->pos_device = deviceTable->GetDevice(player_addr_pos,false)));
      assert((this->power_device = deviceTable->GetDevice(player_addr_power,false)));

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
      if((this->pos_device->Subscribe(this->q) != 0) || 
         (this->power_device->Subscribe(this->q) != 0))
      {
        puts("Failed to subscribe the driver");
        return(-1);
      }
      else
      {
        subscribe("cmd_vel", cmdvel, &ErraticNode::cmdvelReceived, 1);
        return(0);
      }
    }

    int stop()
    {
      int ret=0;
      // Unsubscribe from the device, which causes it to shutdown
      if(this->pos_device->Unsubscribe(this->q) != 0)
      {
        puts("Failed to stop the driver");
        ret=-1;
      }
      if(this->power_device->Unsubscribe(this->q) != 0)
      {
        puts("Failed to stop the driver");
        ret=-1;
      }

      // Give the driver a chance to shutdown.  Wish there were a way to
      // detect when that happened.
      usleep(1000000);
      return(ret);
    }

    int setMotorState(uint8_t state)
    {
      Message* msg;
      // Enable the motors
      player_position2d_power_config_t motorconfig;
      motorconfig.state = state;
      if(!(msg = this->pos_device->Request(this->q,
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


      this->pos_device->PutMsg(this->q,
                               PLAYER_MSGTYPE_CMD,
                               PLAYER_POSITION2D_CMD_VEL,
                               (void*)&cmd,sizeof(cmd),NULL);
    }

  private:
    Driver* driver;
    Device* pos_device;
    Device* power_device;
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
       (hdr->subtype == PLAYER_POSITION2D_DATA_STATE) &&
       (hdr->addr.interf == PLAYER_POSITION2D_CODE))
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

      en.odom.header.frame_id = "odom";
      
      en.odom.header.stamp.sec = (long long unsigned int)floor(hdr->timestamp);
      en.odom.header.stamp.nsec = (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL);
      

      // Publish the new data
      en.publish("odom", en.odom);

      en.tf.sendTransform(tf::Transform(tf::Quaternion(pdata->pos.pa,
                                                    0.0,
                                                    0.0),
                                     tf::Point(pdata->pos.px,
                                               pdata->pos.py,
                                               0.0)
                                     ).inverse(),
                       ros::Time((long long unsigned int)floor(hdr->timestamp),
                                 (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL)),
                       "odom",
                       "base");
                       
        
                       


      /*
      en.tf.sendInverseEuler("odom",
                             "base",
                             pdata->pos.px,
                             pdata->pos.py,
                             0.0,
                             pdata->pos.pa,
                             0.0,
                             0.0,
                             ros::Time((long long unsigned int)floor(hdr->timestamp),
                                       (long long unsigned int)((hdr->timestamp - floor(hdr->timestamp)) * 1000000000ULL)));
      */
      //std::cout <<"Sent 32" <<std::endl;


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

  ros::fini();

  // To quote Morgan, Hooray!
  return(0);
}
