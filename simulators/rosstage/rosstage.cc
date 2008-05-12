#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// libstage
#include <stage.hh>

// roscpp
#include <ros/node.h>
#include <std_msgs/MsgLaserScan.h>
#include <std_msgs/MsgRobotBase2DOdom.h>
#include <std_msgs/MsgBaseVel.h>

#define USAGE "rosstage <worldfile>"

class StageNode : public ros::node
{
  private:
    MsgBaseVel velMsg;
    MsgLaserScan laserMsg;
    MsgRobotBase2DOdom odomMsg;
    ros::thread::mutex lock;

  public:
    StageNode(int argc, char** argv, const char* fname);
    ~StageNode();
    int SubscribeModels();
    void cmdvelReceived();
    void Update();

    static void ghfunc(gpointer key, Stg::StgModel* mod, StageNode* node);

    Stg::StgWorldGui* world;
    Stg::StgModelLaser* lasermodel;
    Stg::StgModelPosition* positionmodel;
};

void
StageNode::ghfunc(gpointer key,
                  Stg::StgModel* mod,
                  StageNode* node)
{
  if(!(node->lasermodel) &&
     (node->lasermodel = dynamic_cast<Stg::StgModelLaser*>(mod)))
  {
    puts("found laser");
  }
  if(!(node->positionmodel) &&
     (node->positionmodel = dynamic_cast<Stg::StgModelPosition*>(mod)))
  {
    puts("found position");
  }
}

void
StageNode::cmdvelReceived()
{
  this->lock.lock();

  printf("received cmd: %.3f %.3f\n",
         this->velMsg.vx, this->velMsg.vw);

  this->positionmodel->SetSpeed(this->velMsg.vx, 0.0, this->velMsg.vw);
  this->lock.unlock();
}

StageNode::StageNode(int argc, char** argv, const char* fname) :
        ros::node("rosstage")
{
  this->lasermodel = NULL;
  this->positionmodel = NULL;

  // initialize libstage
  Stg::Init( &argc, &argv );
  //StgWorld world;
  this->world = new Stg::StgWorldGui(800, 700, "Stage (ROS)");

  this->world->Load(fname);

  this->world->ForEachModel((GHFunc)ghfunc, this);
}

int
StageNode::SubscribeModels()
{
  if(this->lasermodel)
    this->lasermodel->Subscribe();
  else
  {
    puts("no laser");
    return(-1);
  }
  if(this->positionmodel)
    this->positionmodel->Subscribe();
  else
  {
    puts("no position");
    return(-1);
  }

  advertise<MsgLaserScan>("scan");
  advertise<MsgRobotBase2DOdom>("odom");
  subscribe("cmd_vel", velMsg, &StageNode::cmdvelReceived);
  return(0);
}

StageNode::~StageNode()
{
}

void
StageNode::Update()
{
  this->world->PauseUntilNextUpdateTime();
  this->lock.lock();
  this->world->Update();

  Stg::stg_laser_sample_t* samples = this->lasermodel->GetSamples();
  if(samples)
  {
    Stg::stg_laser_cfg_t cfg = this->lasermodel->GetConfig();
    this->laserMsg.angle_min = -cfg.fov/2.0;
    this->laserMsg.angle_max = +cfg.fov/2.0;
    this->laserMsg.angle_increment = cfg.fov / (double)(cfg.sample_count-1);
    this->laserMsg.range_max = cfg.range_bounds.max;
    this->laserMsg.set_ranges_size(cfg.sample_count);
    this->laserMsg.set_intensities_size(cfg.sample_count);
    for(unsigned int i=0;i<cfg.sample_count;i++)
    {
      this->laserMsg.ranges[i] = samples[i].range;
      this->laserMsg.intensities[i] = (uint8_t)samples[i].reflectance;
    }

    publish("scan",this->laserMsg);
  }

  this->odomMsg.pos.x = this->positionmodel->est_pose.x;
  this->odomMsg.pos.y = this->positionmodel->est_pose.y;
  this->odomMsg.pos.th = this->positionmodel->est_pose.a;

  Stg::stg_velocity_t v = this->positionmodel->GetVelocity();

  this->odomMsg.vel.x = v.x;
  this->odomMsg.vel.y = v.y;
  this->odomMsg.vel.th = v.a;

  this->odomMsg.stall = this->positionmodel->Stall();

  publish("odom",this->odomMsg);
  puts("published");

  this->lock.unlock();
}

int 
main(int argc, char** argv)
{ 
  if( argc < 2 )
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc,argv);

  StageNode sn(argc,argv,argv[1]);

  if(sn.SubscribeModels() != 0)
    exit(-1);

  while(sn.ok() && !sn.world->TestQuit())
  {
    sn.Update();
  }
  
  // have to call this explicitly for some reason.  probably interference
  // from signal handling in Stage?
  ros::msg_destruct();

  exit(0);
}
