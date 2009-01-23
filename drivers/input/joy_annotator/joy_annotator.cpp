#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"
#include <sstream>
#include <string>
#include <std_msgs/String.h>

#include <map>
#include <iostream>
#include <fstream>

// This should really go in the .msg
#define MAX_BUTTONS 8

using namespace std;
using namespace ros;
using namespace joy;

class JoyNode : public Node
{
   public:
      joy::Joy joy_msg;
      pthread_t joy_thread;
      std_msgs::String joy_string;
      std::stringstream cur_string;

      int reset_button;
      int send_button;

      bool map_available_;
      std::map<std::string,std::string>cmd_map_;
      std::map<std::string,std::string>::const_iterator find_iter_;

      std::string publish_topic_name_; 

      JoyNode() : Node("joy_annotator")
      {
         param<int>("joy_annotator/reset_button",reset_button,6);
         param<int>("joy_annotator/send_button",send_button,8);

         param<string>("joy_annotator/publish_topic",publish_topic_name_,"~annotation_msg");

         advertise<std_msgs::String>(publish_topic_name_,1);

         subscribe("joy",joy_msg,&JoyNode::joyMsgReceived,this,1);
         reset_button -= 1;
         send_button -= 1;
         map_available_ = false;
      }
      void start()
      {
      }
      void stop()
      {
         unadvertise(publish_topic_name_);
         unsubscribe("joy");
      }
      void readMap(char *filename)
      {
         std::ifstream myfile;
         myfile.open(filename);
         string s1,s2;
         if(!myfile)
         {
            fprintf(stderr,"Unable to open file.\n");
            return;
         }
         else
         {
            fprintf(stderr,"Opened file\n");
            map_available_ = true;
            while(myfile >> s1 >> s2)
            {
               cmd_map_[s1] = s2;
               cout << "Mapped button sequence " << s1 << " to " << s2 << endl;
            }
            myfile.close();
         }
      }
      void joyMsgReceived()
      {
         if(joy_msg.buttons[send_button])
         {
            joy_string.data = cur_string.str();
            if(map_available_)
            {
               find_iter_ = cmd_map_.find(cur_string.str());
               if(find_iter_ != cmd_map_.end())
               {
                  joy_string.data = find_iter_->second;
                  cout << "Found string message match" << endl;
               }
            }
            cout << "Message sent: " << joy_string.data << endl;
            publish(publish_topic_name_,joy_string);
            cur_string.str("");
         }
         else if(joy_msg.buttons[reset_button])
         {
            cur_string.str("");
            cout << "Message reset" << endl;
         }
         else
         {
            for(int i=0 ; i<MAX_BUTTONS; i++)
            {
               if(i == send_button || i == reset_button)
                  continue;
               if(joy_msg.buttons[i])
               {
                  cur_string << (i+1);
                  cout << "Current message string: " << cur_string.str() << endl;
                  break;
               }
            }
         }
      };
};

int main(int argc, char **argv)
{
   ros::init(argc, argv);
   JoyNode joy;
   if(argc == 2)
   {
      joy.readMap(argv[1]);
   }
   joy.start();
   joy.spin();
   joy.stop();
   ros::fini();
   return 0;
}

