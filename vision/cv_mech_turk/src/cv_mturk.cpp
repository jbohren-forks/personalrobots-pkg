// Mechanical Turk labeler node
// Gary Bradski, Dec 3, 2008.  Modified from Jeremey's minimal camera node and connecting to Alexander Sorokin's Mech Turk stuff
// 
// TO DO:
//Maybe make a parameter <path_to_location of submit_img.py>

#include <cstdio>
#include <vector>
#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "ros/node.h"
#include "image_msgs/Image.h"
#include "image_msgs/CvBridge.h"
#include <stdlib.h>

using namespace std;
using namespace ros;

class CvMTurk : public Node
{
public:
  image_msgs::Image image_msg;
  image_msgs::CvBridge cv_bridge;
  //Parameters
  char cmd[1024];   		//Will hold commands to run submit_img.py
  char object_name[256]; 	//Will hold the object's base name (do not append endings) DEFAULT: Default0000
  int object_count; 		//Version of image being stored
 

  CvMTurk() : Node("cv_mturk"), object_count(0)
  { 
    cvNamedWindow("cv_mturk", CV_WINDOW_AUTOSIZE);
    subscribe("image", image_msg, &CvMTurk::image_cb, 0);
    sprintf(object_name,"Default");
  }


  void help()
  {
	printf("\nUsage:\n  ./cv_mturk  image:=dcam/image [or whatever your camera name is]\n\n"
	"   Takes keyboard input:\n"
	"\tESQ,q,Q:   Quit\n"
	"\th,H:       Print this help\n"
	"\tm,M:       Submit to Mechanical Turk\n\n" 
	"\tWhen you type \"m\", cv_mturk will invoke: submit_img.py which in turn depends on:\n"
	"\t\t* id_rsa_SIU  [These are access keys in the same dir as submit_img.py]\n"
	"\t\t* id_rsa_SIU.pub\n"
	"\t\t* Needs an image storage subdirectory below submit_img.phy called: \"/images\"\n\n"
	);
  }

/*
std::string cvGetString(std::string prompt, std::string init)
{
	std::string str=init;
	printf("%s\n%s",prompt.c_str(),str.c_str());
	fflush(stdout);

	 int c = 0, slen;
	 //COLLECT USER INPUT, IGNORE WHITESPACE, ESC OUT
	 while (1) 
	 {
		c = cvWaitKey(0) & 0xFF;
		if(c == 27) {str.erase(); break;}
		if((c == 0)||(c > 128)) continue;
		if((c == 13)||(c == 10)) break; //Carriage return and/or line feed => accept this label
		if((c == ' ')||(c == '\t')) continue; //Ignore white space
		slen = str.length();
		printf("\r");
		for(int u=0; u<slen; ++u)
			printf(" ");
		printf("\r");
			
		if(c == 8) //backspace
		{
			//OVERLAY MASK ONTO IMAGE
			printf("\r");
			for(int u=0; u<slen; ++u)
				printf(" ");
			printf("\r");

			if(slen)
				str.erase(slen - 1);
		}
		else
		{
			str.append(1, (char)c);
		}
		printf("%s\r",str.c_str());
		fflush(stdout);
	} //End while
	//CLEAN UP AND OUT
	printf("string %s\n",str.c_str());
	printf("\n");

	return str;
}
*/



  void image_cb()
  {
    IplImage *cv_img_to_label;

    cv_bridge.fromImage(image_msg);
    cv_img_to_label = cv_bridge.toIpl();

    if (cv_img_to_label)
    {
      //VIEW THE IMAGE
      cvShowImage("cv_mturk", cv_img_to_label);

      //HANDLE KEYBOARD INPUT
      int c = cvWaitKey(3)&0xFF;
      switch(c)
      { 
      	case 27:  //ESQ -- exit
	case 'q': // or quit
        case 'Q':    
		printf("Bye bye\n");
		cvReleaseImage(&cv_img_to_label);
		shutdown();
		break;
	case 'h': //Help
	case 'H':
		help();
		break;
	case 'm': //Invoke Mechanical Turk submission
	case 'M':
		//Save the image first
//		sprintf(cmd,"../src/images/%s%.4d.jpg",object_name,object_count);
		sprintf(cmd,"./src/images/%s%.4d.jpg",object_name,object_count);
		cvSaveImage(cmd,cv_img_to_label);
		printf("Saved %s\n",cmd);
		sprintf(cmd,"cd src && ./submit_img.py ./images/%s%.4d.jpg;\n",object_name,object_count);
//		sprintf(cmd,"python submit_img.py images/%s%.4d.jpg\n",object_name,object_count);
		printf("Issuing command: %s\n\n",cmd);
		system(cmd);
		object_count+= 1;		
		break;
      }

    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  CvMTurk view;
  view.spin();  //infinite loop in node which calls back to image_cb() "image call back".  exit by calling shutdown()
  ros::fini();
  return 0;
}

