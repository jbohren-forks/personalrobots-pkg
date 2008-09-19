#include <MoveBase.hh>

int main(int argc, char** argv)
{
  /*
  if(argc != 2){
    std::cout << "Usage: ./";
    return -1;
  }
  */
  ros::init(argc,argv);

  // Extract parameters
  //const std::string param = argv[1];

  MoveBase node(10, 1, 255, 100.0);
  node.run();
  ros::fini();


  return(0);
}
