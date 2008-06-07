#include <iostream>
#include <assert.h>

#include <ros/node.h>

#include <gridfastslam/gridslamprocessor.h>

class GMappingNode : public ros::node
{
  public:
    GMappingNode() : ros::node("gmapping") 
    { 
      this->gsp = new GMapping::GridSlamProcessor(std::cerr);
      assert(this->gsp);
    }
    ~GMappingNode()
    {
      delete this->gsp;
    }
    
  private:
    GMapping::GridSlamProcessor* gsp;
};

int
main(void)
{
  GMappingNode gn;

  return(0);
}
