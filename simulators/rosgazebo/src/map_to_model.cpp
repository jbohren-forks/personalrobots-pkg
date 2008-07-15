#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include <SDL/SDL_image.h>

#define USAGE "USAGE: heightmap-simplifier <img_in> <res>"

/** specify a rectangular size */
typedef struct 
{
  int x, y;
} rect_size_t;

/** Specify a 2d position */
typedef struct
{
  int x, y;
} rect_pose_t;

/** defines a rectangle of [size] located at [pose] */
typedef struct
{
  rect_pose_t pose;
  rect_size_t size;
} rect_t; // axis-aligned rectangle

static unsigned char* get_pixel(SDL_Surface* img, int x, int y );
static bool pixel_is_set( SDL_Surface* img, int x, int y, int threshold );
static void set_rect( SDL_Surface* pb, int x, int y, int width, int height, unsigned char val );
SDL_Surface* rects_from_image_file( const char* filename, 
                                    rect_t** rects, 
                                    unsigned int* rect_count,
                                    unsigned int* widthp, 
                                    unsigned int* heightp );

int save_rects_to_image_file(const char* outfile, 
                             SDL_Surface* pixbuf,
                             rect_t* rects, 
                             unsigned int rect_count, 
                             unsigned int width, 
                             unsigned int height);

int create_gazebo_obstacles(int width, int height,
                            rect_t* rects, 
                            unsigned int rect_count,
                            double res);

int
main(int argc, char** argv)
{
  if(argc < 3)
  {
    puts(USAGE);
    exit(-1);
  }

  const char* infile = argv[1];
  //const char* outfile = argv[2];
  double res = atof(argv[2]);

  rect_t* rects;
  unsigned int rect_count;
  unsigned int width, height;

  SDL_Surface* pixbuf; 
  pixbuf = rects_from_image_file(infile, &rects, &rect_count, 
                                        &width, &height);
  assert(pixbuf);

  //save_rects_to_image_file("foo.png", pixbuf, rects, rect_count, width, height);

  create_gazebo_obstacles(width, height,
                          rects, 
                          rect_count,
                          res);

  fprintf(stderr, "found %d rects\n", rect_count);

  return(0);
}


int
create_gazebo_obstacles(int width,
                        int height,
                        rect_t* rects, 
                        unsigned int rect_count,
                        double res)
{
  printf("<?xml version=\"1.0\"?>\n"
         "<model:physical name=\"willow-walls\"\n"
         "xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"\n"
         "xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"\n"
         "xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"\n"
         "xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"\n"
         "xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"\n"
         "xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n"
         "xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"\n"
         ">\n"
         "    <xyz>  0.0 0.0 0.0</xyz>\n"
         "    <rpy>   0.0    0.0    0.0</rpy>\n"
         "    <static>true</static>\n");
  for(unsigned int i=0;i<rect_count;i++)
  {
    printf("\n\n  <body:box name=\"wall_%d_body\">\n"
           "    <xyz>  %.3f   %.3f  1.0</xyz>\n"
           "    <rpy>   0.0    0.0    0.0</rpy>\n"
           "    <static>true</static>\n"
           "      <geom:box name=\"wall_%d_geom\">\n"
           "        <mesh>default</mesh>\n"
           "        <size> %.3f   %.3f  2.0</size>\n"
           "        <visual>\n"
           "          <size> %.3f   %.3f  2.0</size>\n"
           "          <material>Gazebo/PioneerBody</material>\n"
           "          <mesh>unit_box</mesh>\n"
           "        </visual>\n"
           "      </geom:box>\n"
           "     </body:box>\n",
           i,
           (rects[i].pose.x+rects[i].size.x/2.0-width/2.0)*res, 
           (rects[i].pose.y+rects[i].size.y/2.0-height/2.0)*res,
           i,
           rects[i].size.x*res, rects[i].size.y*res,
           rects[i].size.x*res, rects[i].size.y*res);

  }

  // Also add the ground
  printf("<model:physical name=\"gplane\">\n"
          "<xyz>%.3f %.3f 0</xyz>    \n"
          "<rpy>0 0 0</rpy>\n"
          "<static>true</static>\n"
          "<body:plane name=\"plane\">\n"
          "<geom:plane name=\"plane\">\n"
          "<kp>1000000.0</kp>\n"
          "<kd>1.0</kd>\n"
          "<normal>0 0 1</normal>\n"
          "<size>%.3f %.3f</size>\n"
          "<material>PR2/floor_texture</material>\n"
          "</geom:plane>\n"
          "</body:plane>\n"
          "</model:physical>\n",
         0.0,
         0.0,
         width*res, height*res);
  printf("</model:physical>\n");
           
  return(0);
}

static unsigned char* get_pixel( SDL_Surface* img, int x, int y )
{
  //unsigned char* pixels = (unsigned char*)(img->data()[0]);
  unsigned char* pixels = (unsigned char*)(img->pixels);
  int rs = img->pitch;
  int ch = img->format->BytesPerPixel;

  unsigned int index = (y * rs) + (x * ch);
  return( pixels + index );
}

static bool pixel_is_set( SDL_Surface* img, int x, int y, int threshold )
{
  unsigned char* pixel = get_pixel( img,x,y );
  return( pixel[0] < threshold );
}

// set all the pixels in a rectangle 
static void set_rect( SDL_Surface* pb, int x, int y, int width, int height, unsigned char val )
{
  int a;
  for( a = y; a < y+height; a++ )
  {
    unsigned char* pix = get_pixel( pb, x, a );
    memset(pix, val, pb->format->BytesPerPixel * (int)width);
  }
}  

SDL_Surface* rects_from_image_file( const char* filename, 
                                         rect_t** rects, 
                                         unsigned int* rect_count,
                                         unsigned int* widthp, 
                                         unsigned int* heightp )
{
  // TODO: make this a parameter
  const int threshold = 1;

  SDL_Surface* img;

  fprintf(stderr,"opening %s\n", filename);
  if(!(img = IMG_Load(filename)))
  {
    fprintf(stderr,"failed to open image file %s", filename);
    return(NULL);
  }

  fprintf(stderr, "loaded image %s %d X %d\n", filename, img->w, img->h);

  *rect_count = 0;
  size_t allocation_unit = 1000;
  size_t rects_allocated = allocation_unit;
  *rects = (rect_t*)malloc(sizeof(rect_t)*rects_allocated);
  assert(*rects);

  int img_width = img->w;
  int img_height = img->h;

  // if the caller wanted to know the dimensions
  if( widthp ) *widthp = img_width;
  if( heightp ) *heightp = img_height;


  int y, x;
  for(y = 0; y < img_height; y++)
  {
    for(x = 0; x < img_width; x++)
    {
      // skip blank (black) pixels
      if(pixel_is_set( img,x,y, threshold))
        continue;

      // a rectangle starts from this point
      int startx = x;
      int starty = y;
      int height = img_height; // assume full height for starters

      // grow the width - scan along the line until we hit an empty (white) pixel
      int yy;
      for( ; x < img_width &&  ! pixel_is_set(img,x,y,threshold); x++ )
      {
        // look down to see how large a rectangle below we can make
        yy  = y;
        while( ! pixel_is_set(img,x,yy,threshold) && (yy < img_height-1) )
        { 
          yy++;
        } 	      

        // now yy is the depth of a line of non-zero pixels
        // downward we store the smallest depth - that'll be the
        // height of the rectangle
        if( yy-y < height ) height = yy-y; // shrink the height to fit
      }

      // blacken the pixels we have used in this rect
      set_rect(img, startx, starty, x-startx, height, 0x00);

      // add this rectangle to the array
      (*rect_count)++;

      if( (*rect_count) > rects_allocated )
      {
        rects_allocated = (*rect_count) + allocation_unit;

        *rects = (rect_t*)
                realloc( *rects, rects_allocated * sizeof(rect_t) );
      }

      rect_t *latest = (*rects)+(*rect_count)-1;
      latest->pose.x = startx;
      latest->pose.y = img_height - (starty+height);
      latest->size.x = x-startx;
      latest->size.y = height;

      //fprintf(stderr, "rect %d (%d %d %d %d) \n", 
        //*rect_count, 
        //latest->pose.x, latest->pose.y, latest->size.x, latest->size.y);

    }
  }

  return img;
}
