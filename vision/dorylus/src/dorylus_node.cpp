#include <std_msgs/SpinImage.h>
#include <std_msgs/PointCloudFloat32.h>
#include <ros/node.h>
#include <iostream>
#include <smartScan.h>

using namespace std;

class dorylus : public ros::node
{
public:
  dorylus() : ros::node("dorylus")
  {
    //Set up spin image.
    support = .1;
    pixelsPerMeter = 250;
    height = (int)(2*support*pixelsPerMeter);
    width = (int)(support*pixelsPerMeter);
    advertise<std_msgs::SpinImage>("SpinImage");

    //Set up message.
    simsg.set_data_size(height*width);
    simsg.height = height;
    simsg.width = width;
    simsg.support = support;
    simsg.pixelsPerMeter = pixelsPerMeter;

    //Listen for new data.
    subscribe<std_msgs::PointCloudFloat32>("full_cloud", cloud, &dorylus::callback_cloud);
  }
  

  void loadSpinImageMsg(const scan_utils::Grid2D &si, float x, float y, float z) {
    simsg.x = x;
    simsg.y = y;
    simsg.z = z;

    for(int c=0; c<width; c++) {
      for(int r=0; r<height; r++) {	
	simsg.data[c*height+r] = si.getElement(r,c);
      }
    }
  }

private:
  std_msgs::PointCloudFloat32 cloud;
  std_msgs::SpinImage simsg;
  SmartScan cloudss;

  float support, pixelsPerMeter;
  int height, width;

  void callback_cloud()  {
    static int numRecvd_cloud=0;
    numRecvd_cloud++;
    cout << "Got cloud " << numRecvd_cloud << endl;
    
    //Put it into a SmartScan.
    cloudss = SmartScan();
    cloudss.setFromRosCloud(cloud);

    //Compute a spin image.
    scan_utils::Grid2D* psi = new scan_utils::Grid2D(height, width);
    srand(time(NULL));
    int randId = rand() % cloudss.size();
    std_msgs::Point3DFloat32 pt = cloudss.getPoint(randId);
    cloudss.computeSpinImageFixedOrientation(*psi, pt.x, pt.y, pt.z, support, pixelsPerMeter);

    //Send it to the Dorylus viewer.
    loadSpinImageMsg(*psi, pt.x, pt.y, pt.z);
//     cout << "Printing data" << endl;
//     for(unsigned int i=0; i < simsg.get_data_size(); i++) {
//       cout << simsg.data[i] << " ";
//     }
//     cout <<"done" << endl;
    publish("SpinImage", simsg);


    //Clean up.
    delete psi;    
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv);
  dorylus d;
  while(d.ok())
    usleep(1000);

  //  d.self_destruct();
  ros::fini();
  return 0;
}


