#include <ransac_ground_plane_extraction/ransac_ground_plane_extraction_node.h>


int main(int argc, char** argv)
{
  ros::init(argc,argv); 

  ransac_ground_plane_extraction::RansacGroundPlaneExtractionNode node("ransac_ground_plane");

  try {
    node.spin();
  }
  catch(char const* e){
    std::cout << e << std::endl;
  }
  ros::fini();
  return(0);
}
