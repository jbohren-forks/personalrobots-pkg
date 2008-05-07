#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <sys/time.h>

#include <libstandalone_drivers/plan.h>

#include <ros/node.h>
#include <std_msgs/MsgPlanner2DState.h>
#include <std_msgs/MsgPlanner2DGoal.h>
#include <rosTF/rosTF.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
//void draw_cspace(plan_t* plan, const char* fname);
//void draw_path(plan_t* plan, double lx, double ly, const char* fname);
#include <gdk-pixbuf/gdk-pixbuf.h>
double get_time(void);
int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
       			const char* fname, int negate);

class WavefrontNode: public ros::node
{
  private:
    // Plan object
    plan_t* plan;
    // Do we have a goal?
    bool have_goal;
    // Have we reached our goal?
    bool reached_goal;
    // Are we enabled?
    bool enable;
    // Current goal
    double goal[3];
    // Current pose
    double pose[3];

    // Planning parameters
    double robot_radius;
    double safety_dist;
    double max_radius;
    double dist_penalty;
    double plan_halfwidth;

    // Goal message
    MsgPlanner2DGoal goalMsg;

    // Message callback
    void goalReceived();

  public:
    // Transform client
    rosTFClient* tf;

    WavefrontNode(char* fname, double res);
    ~WavefrontNode();
    
    // Execute a planning cycle
    void doOneCycle();
    // Sleep for the remaining time of the cycle
    void sleep();

};

#define USAGE "USAGE: wavefront_player <map.png> <res>"

int
main(int argc, char** argv)
{
  if(argc < 3)
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc,argv);

  WavefrontNode wn(argv[1],atof(argv[2]));
  wn.tf = new rosTFClient(wn);

  while(wn.ok())
  {
    wn.doOneCycle();
    wn.sleep();
  }
  return(0);
}

WavefrontNode::WavefrontNode(char* fname, double res) : 
        ros::node("wavfront_player"),
        have_goal(false),
        reached_goal(false),
        enable(true),
        robot_radius(0.16),
        safety_dist(0.05),
        max_radius(0.25),
        dist_penalty(1.0),
        plan_halfwidth(5.0),
        tf(NULL)
{
  advertise<MsgPlanner2DState>("state");
  subscribe("goal", goalMsg, &WavefrontNode::goalReceived);

  // TODO: get map via RPC
  char* mapdata;
  int sx, sy;
  assert(read_map_from_image(&sx, &sy, &mapdata, 
                             fname, 0) == 0);

  // TODO: get robot geometry (via RPC?) and plan params from
  // param_server
  assert((this->plan = plan_alloc(this->robot_radius+this->safety_dist,
                                  this->robot_radius+this->safety_dist,
                                  this->max_radius,
                                  this->dist_penalty,0.5)));
  // allocate space for map cells
  assert(this->plan->cells == NULL);
  assert((this->plan->cells = 
          (plan_cell_t*)realloc(this->plan->cells,
                                (sx * sy * sizeof(plan_cell_t)))));

  // Copy over obstacle information from the image data that we read
  for(int j=0;j<sy;j++)
  {
    for(int i=0;i<sx;i++)
    {
      this->plan->cells[i+j*sx].occ_state = 
              mapdata[MAP_IDX(sx,i,j)];
    }
  }
  free(mapdata);

  this->plan->scale = res;
  this->plan->size_x = sx;
  this->plan->size_y = sy;
  this->plan->origin_x = 0.0;
  this->plan->origin_y = 0.0;

  // Do initialization
  plan_init(this->plan);

  // Compute cspace over static map
  plan_compute_cspace(this->plan);
}

WavefrontNode::~WavefrontNode()
{
  plan_free(this->plan);
  if(this->tf)
    delete this->tf;
}

void 
WavefrontNode::goalReceived()
{
  // Got a new goal message; handle it
  this->enable = goalMsg.enable;
  if(this->enable)
  {
    this->goal[0] = goalMsg.goal.x;
    this->goal[1] = goalMsg.goal.y;
    this->goal[2] = goalMsg.goal.th;
    this->have_goal = true;
  }
}

// Execute a planning cycle
void 
WavefrontNode::doOneCycle()
{
  // Try a local plan
  /*
  if(plan_do_local(this->plan, this->localize_x, 
                   this->localize_y, this->scan_maxrange) < 0)
  {
    // Fallback on global plan
    if(plan_do_global(this->plan, this->localize_x, this->localize_y, 
                      this->target_x, this->target_y) < 0)
    {
      if(!printed_warning)
      {
        puts("Wavefront: global plan failed");
        printed_warning = true;
      }
    }
  }
  */
}

// Sleep for the remaining time of the cycle
void 
WavefrontNode::sleep()
{
}

int
read_map_from_image(int* size_x, int* size_y, char** mapdata, 
                    const char* fname, int negate)
{
  GdkPixbuf* pixbuf;
  guchar* pixels;
  guchar* p;
  int rowstride, n_channels, bps;
  GError* error = NULL;
  int i,j,k;
  double occ;
  int color_sum;
  double color_avg;

  // Initialize glib
  g_type_init();

  printf("MapFile loading image file: %s...", fname);
  fflush(stdout);

  // Read the image
  if(!(pixbuf = gdk_pixbuf_new_from_file(fname, &error)))
  {
    printf("failed to open image file %s", fname);
    return(-1);
  }

  *size_x = gdk_pixbuf_get_width(pixbuf);
  *size_y = gdk_pixbuf_get_height(pixbuf);

  assert(*mapdata = (char*)malloc(sizeof(char) * (*size_x) * (*size_y)));

  rowstride = gdk_pixbuf_get_rowstride(pixbuf);
  bps = gdk_pixbuf_get_bits_per_sample(pixbuf)/8;
  n_channels = gdk_pixbuf_get_n_channels(pixbuf);
  //if(gdk_pixbuf_get_has_alpha(pixbuf))
    //n_channels++;

  // Read data
  pixels = gdk_pixbuf_get_pixels(pixbuf);
  for(j = 0; j < *size_y; j++)
  {
    for (i = 0; i < *size_x; i++)
    {
      p = pixels + j*rowstride + i*n_channels*bps;
      color_sum = 0;
      for(k=0;k<n_channels;k++)
        color_sum += *(p + (k * bps));
      color_avg = color_sum / (double)n_channels;

      if(negate)
        occ = color_avg / 255.0;
      else
        occ = (255 - color_avg) / 255.0;
      if(occ > 0.95)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = +1;
      else if(occ < 0.1)
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = -1;
      else
        (*mapdata)[MAP_IDX(*size_x,i,*size_y - j - 1)] = 0;
    }
  }

  gdk_pixbuf_unref(pixbuf);

  puts("Done.");
  printf("MapFile read a %d X %d map\n", *size_x, *size_y);
  return(0);
}

double 
get_time(void)
{
  struct timeval curr;
  gettimeofday(&curr,NULL);
  return(curr.tv_sec + curr.tv_usec / 1e6);
}
