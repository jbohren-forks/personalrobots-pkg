#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

// We use gdk-pixbuf to load the image from disk
// I would use SDL_image, but it doesn't provide the capability to save an
// image file (doh!)
#include <gdk-pixbuf/gdk-pixbuf.h>

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

static unsigned char* get_pixel(GdkPixbuf* img, int x, int y );
static bool pixel_is_set( GdkPixbuf* img, int x, int y, int threshold );
static void set_rect( GdkPixbuf* pb, int x, int y, int width, int height, unsigned char val );
GdkPixbuf* rects_from_image_file( const char* filename, 
                                  rect_t** rects, 
                                  unsigned int* rect_count,
                                  unsigned int* widthp, 
                                  unsigned int* heightp );

int save_rects_to_image_file(const char* outfile, 
                             GdkPixbuf* pixbuf,
                             rect_t* rects, 
                             unsigned int rect_count, 
                             unsigned int width, 
                             unsigned int height);

int create_gazebo_obstacles(rect_t* rects, 
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

  GdkPixbuf* pixbuf; 
  pixbuf = rects_from_image_file(infile, &rects, &rect_count, 
                                        &width, &height);
  assert(pixbuf);

  //save_rects_to_image_file(outfile, pixbuf, rects, rect_count, width, height);

  create_gazebo_obstacles(rects, 
                          rect_count,
                          res);

  fprintf(stderr, "found %d rects\n", rect_count);

  return(0);
}


int 
save_rects_to_image_file(const char* outfile, 
                         GdkPixbuf* pixbuf,
                         rect_t* rects, 
                         unsigned int rect_count, 
                         unsigned int width, 
                         unsigned int height)
{
  GdkPixbuf* newpixbuf;
  GError* error = NULL;

  newpixbuf = gdk_pixbuf_new(gdk_pixbuf_get_colorspace(pixbuf),
                          gdk_pixbuf_get_has_alpha(pixbuf),
                          gdk_pixbuf_get_bits_per_sample(pixbuf),
                          gdk_pixbuf_get_width(pixbuf),
                          gdk_pixbuf_get_height(pixbuf));
  assert(newpixbuf);

  int bytes_per_sample = gdk_pixbuf_get_bits_per_sample (newpixbuf) / 8;
  int num_samples = gdk_pixbuf_get_n_channels(newpixbuf);

  // pixbuf starts out empty (all black), so just add the rectangles
  for(int i=0;i<(int)rect_count;i++)
  {
    fprintf(stderr,"x: %d y: %d w: %d h: %d\n",
           rects[i].pose.x,
           rects[i].pose.y,
           rects[i].size.x,
           rects[i].size.y);

    // Assuming axis-aligned rectangles
    int x0, y0;
    x0 = rects[i].pose.x;
    y0 = rects[i].pose.y;
    for(int y=y0;(y-y0)<rects[i].size.y;y++)
    {
      unsigned char* pix = get_pixel(newpixbuf, x0, y);
      memset(pix, 255, rects[i].size.x * num_samples * bytes_per_sample);
    }
  }

  if(!gdk_pixbuf_save(newpixbuf,
                      outfile,
                      "png",
                      &error, NULL))
  {
    fprintf(stderr,"failed to save output image to %s\n", outfile);
  }
  

  return(0);
}

int
create_gazebo_obstacles(rect_t* rects, 
                        unsigned int rect_count,
                        double res)
{
  for(unsigned int i=0;i<rect_count;i++)
  {
    printf("\n\n  <model:physical name=\"wall_%d_model\">\n"
           "    <xyz>  %.3f   %.3f  1.0</xyz>\n"
           "    <rpy>   0.0    0.0    0.0</rpy>\n"
           "    <static>true</static>\n"
           "    <body:box name=\"wall_%d_body\">\n"
           "      <geom:box name=\"wall_%d_geom\">\n"
           "        <mesh>default</mesh>\n"
           "        <size> %.3f   %.3f  2.0</size>\n"
           "        <visual>\n"
           "          <size> %.3f   %.3f  2.0</size>\n"
           "          <material>Gazebo/PioneerBody</material>\n"
           "          <mesh>unit_box</mesh>\n"
           "        </visual>\n"
           "      </geom:box>\n"
           "     </body:box>\n"
           "  </model:physical>\n",

           i,
           rects[i].pose.x*res, rects[i].pose.y*res,
           i,
           i,
           rects[i].size.x*res, rects[i].size.y*res,
           rects[i].size.x*res, rects[i].size.y*res);
  }
  return(0);
}

static unsigned char* get_pixel( GdkPixbuf* img, int x, int y )
{
  //unsigned char* pixels = (unsigned char*)(img->data()[0]);
  unsigned char* pixels = (unsigned char*)(gdk_pixbuf_get_pixels(img));
  int rs = gdk_pixbuf_get_rowstride(img);
  int ch = gdk_pixbuf_get_n_channels(img);

  unsigned int index = (y * rs) + (x * ch);
  return( pixels + index );
}

static bool pixel_is_set( GdkPixbuf* img, int x, int y, int threshold )
{
  unsigned char* pixel = get_pixel( img,x,y );
  return( pixel[0] < threshold );
}

// set all the pixels in a rectangle 
static void set_rect( GdkPixbuf* pb, int x, int y, int width, int height, unsigned char val )
{
  int bytes_per_sample = gdk_pixbuf_get_bits_per_sample (pb) / 8;
  int num_samples = gdk_pixbuf_get_n_channels(pb);

  int a;
  for( a = y; a < y+height; a++ )
  {
    unsigned char* pix = get_pixel( pb, x, a );
    memset(pix, val, num_samples * bytes_per_sample * (int)width);
  }
}  

GdkPixbuf* rects_from_image_file( const char* filename, 
                                         rect_t** rects, 
                                         unsigned int* rect_count,
                                         unsigned int* widthp, 
                                         unsigned int* heightp )
{
  // TODO: make this a parameter
  const int threshold = 1;

  GdkPixbuf* img;
  GError* error = NULL;

  // Initialize glib
  g_type_init();

  fprintf(stderr,"opening %s\n", filename);
  if(!(img = gdk_pixbuf_new_from_file(filename, &error)))
  {
    fprintf(stderr,"failed to open image file %s", filename);
    return(NULL);
  }

  fprintf(stderr, "loaded image %s w %d h %d rs %d n_chan %d\n",
    filename, gdk_pixbuf_get_width(img), gdk_pixbuf_get_height(img),
    gdk_pixbuf_get_rowstride(img), gdk_pixbuf_get_n_channels(img));

  *rect_count = 0;
  size_t allocation_unit = 1000;
  size_t rects_allocated = allocation_unit;
  *rects = (rect_t*)malloc(sizeof(rect_t)*rects_allocated);
  assert(*rects);

  int img_width = gdk_pixbuf_get_width(img);
  int img_height = gdk_pixbuf_get_height(img);

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

      fprintf(stderr, "rect %d (%d %d %d %d) \n", 
        *rect_count, 
        latest->pose.x, latest->pose.y, latest->size.x, latest->size.y);

    }
  }

  return img;
}
