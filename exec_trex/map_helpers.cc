#include <assert.h>
#include <stdlib.h>
#include <iostream>

#include "map_helpers.h"


int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
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
      if(occ > 0.5)
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

void draw_map(plan_t* plan, double lx, double ly, const char* fname)
{
  GdkPixbuf* pixbuf;
  GError* error = NULL;
  guchar* pixels;
  int p;
  int paddr;
  int i, j;
  //plan_cell_t* cell;

  pixels = (guchar*)malloc(sizeof(guchar)*plan->size_x*plan->size_y*3);

  p=0;
  for(j=plan->size_y-1;j>=0;j--)
  {
    for(i=0;i<plan->size_x;i++,p++)
    {
      paddr = p * 3;
      if(plan->cells[PLAN_INDEX(plan,i,j)].occ_state == 1)
      {
        pixels[paddr] = 255;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 0;
      } else if(plan->cells[PLAN_INDEX(plan,i,j)].occ_state_dyn == 1) {
	pixels[paddr] = 255;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 0;
      }else if(plan->cells[PLAN_INDEX(plan,i,j)].occ_dist < plan->max_radius)
      {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
      }
      else
      {
        pixels[paddr] = 255;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 255;
      }
      /*
         if((7*plan->cells[PLAN_INDEX(plan,i,j)].plan_cost) > 255)
         {
         pixels[paddr] = 0;
         pixels[paddr+1] = 0;
         pixels[paddr+2] = 255;
         }
         else
         {
         pixels[paddr] = 255 - 7*plan->cells[PLAN_INDEX(plan,i,j)].plan_cost;
         pixels[paddr+1] = 0;
         pixels[paddr+2] = 0;
         }
       */
    }
  }

  int li = PLAN_GXWX(plan, lx);
  int lj = PLAN_GYWY(plan, ly);

  for(i=li-3;i<=li+3;i++) {
    for(j = lj-3;j <= lj+3; j++) {
      paddr = 3*PLAN_INDEX(plan,i,plan->size_y - j - 1);
      pixels[paddr] = 0;
      pixels[paddr+1] = 255;
      pixels[paddr+2] = 0;
    }
  }

 //  for(i=0;i<plan->path_count;i++)
//   {
//     cell = plan->path[i];
    
//     paddr = 3*PLAN_INDEX(plan,cell->ci,plan->size_y - cell->cj - 1);
//     pixels[paddr] = 0;
//     pixels[paddr+1] = 255;
//     pixels[paddr+2] = 0;
//   }

//   for(i=0;i<plan->lpath_count;i++)
//   {
//     cell = plan->lpath[i];
    
//     paddr = 3*PLAN_INDEX(plan,cell->ci,plan->size_y - cell->cj - 1);
//     pixels[paddr] = 255;
//     pixels[paddr+1] = 0;
//     pixels[paddr+2] = 255;
//   }

  /*
  for(p=0;p<plan->waypoint_count;p++)
  {
    cell = plan->waypoints[p];
    for(j=-3;j<=3;j++)
    {
      cj = cell->cj + j;
      for(i=-3;i<=3;i++)
      {
        ci = cell->ci + i;
        paddr = 3*PLAN_INDEX(plan,ci,plan->size_y - cj - 1);
        pixels[paddr] = 255;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
      }
    }
  }
  */

  pixbuf = gdk_pixbuf_new_from_data(pixels, 
                                    GDK_COLORSPACE_RGB,
                                    0,8,
                                    plan->size_x,
                                    plan->size_y,
                                    plan->size_x * 3,
                                    NULL, NULL);
  
  gdk_pixbuf_save(pixbuf,fname,"png",&error,NULL);
  gdk_pixbuf_unref(pixbuf);
  free(pixels);
}

void draw_path(plan_t* plan, double lx, double ly, const char* fname) {
  GdkPixbuf* pixbuf;
  GError* error = NULL;
  guchar* pixels;
  int p;
  int paddr;
  int i, j;
  plan_cell_t* cell;

  pixels = (guchar*)malloc(sizeof(guchar)*plan->size_x*plan->size_y*3);

  p=0;
  for(j=plan->size_y-1;j>=0;j--)
  {
    for(i=0;i<plan->size_x;i++,p++)
    {
      paddr = p * 3;
      if(plan->cells[PLAN_INDEX(plan,i,j)].occ_state == 1)
      {
        pixels[paddr] = 255;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 0;
      }
      else if(plan->cells[PLAN_INDEX(plan,i,j)].occ_dist < plan->max_radius)
      {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
      }
      else
      {
        pixels[paddr] = 255;
        pixels[paddr+1] = 255;
        pixels[paddr+2] = 255;
      }
      /*
         if((7*plan->cells[PLAN_INDEX(plan,i,j)].plan_cost) > 255)
         {
         pixels[paddr] = 0;
         pixels[paddr+1] = 0;
         pixels[paddr+2] = 255;
         }
         else
         {
         pixels[paddr] = 255 - 7*plan->cells[PLAN_INDEX(plan,i,j)].plan_cost;
         pixels[paddr+1] = 0;
         pixels[paddr+2] = 0;
         }
       */
    }
  }

  for(i=0;i<plan->path_count;i++)
  {
    cell = plan->path[i];
    
    paddr = 3*PLAN_INDEX(plan,cell->ci,plan->size_y - cell->cj - 1);
    pixels[paddr] = 0;
    pixels[paddr+1] = 255;
    pixels[paddr+2] = 0;
  }

  for(i=0;i<plan->lpath_count;i++)
  {
    cell = plan->lpath[i];
    
    paddr = 3*PLAN_INDEX(plan,cell->ci,plan->size_y - cell->cj - 1);
    pixels[paddr] = 255;
    pixels[paddr+1] = 0;
    pixels[paddr+2] = 255;
  }

  /*
  for(p=0;p<plan->waypoint_count;p++)
  {
    cell = plan->waypoints[p];
    for(j=-3;j<=3;j++)
    {
      cj = cell->cj + j;
      for(i=-3;i<=3;i++)
      {
        ci = cell->ci + i;
        paddr = 3*PLAN_INDEX(plan,ci,plan->size_y - cj - 1);
        pixels[paddr] = 255;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
      }
    }
  }
  */

  pixbuf = gdk_pixbuf_new_from_data(pixels, 
                                    GDK_COLORSPACE_RGB,
                                    0,8,
                                    plan->size_x,
                                    plan->size_y,
                                    plan->size_x * 3,
                                    NULL, NULL);
  
  gdk_pixbuf_save(pixbuf,fname,"png",&error,NULL);
  gdk_pixbuf_unref(pixbuf);
  free(pixels);
}

void draw_costmap(plan_t* plan, const char* fname) {
  
  double maxCost = -1.0;
  double minCost = 1000000.0;

  for(int j = plan->size_y-1; j>=0; j--) {
    for(int i = 0; i<plan->size_x; i++) {
      
      //std::cout << "Cost " << j << " " << i << " " 
      //		<< plan->cells[PLAN_INDEX(plan,i,j)].plan_cost << std::endl;

      if(plan->cells[PLAN_INDEX(plan,i,j)].plan_cost > maxCost && 
	 plan->cells[PLAN_INDEX(plan,i,j)].plan_cost < PLAN_MAX_COST) {
	//std::cout << "Updating maxCost " << j << " " << i << " " 
	//	  << plan->cells[PLAN_INDEX(plan,i,j)].plan_cost << std::endl;
	maxCost = plan->cells[PLAN_INDEX(plan,i,j)].plan_cost;
      }
      if(plan->cells[PLAN_INDEX(plan,i,j)].plan_cost < minCost) {
	//std::cout << "Updating minCost " << j << " " << i << " " 
	//	  << plan->cells[PLAN_INDEX(plan,i,j)].plan_cost << std::endl;
	minCost = plan->cells[PLAN_INDEX(plan,i,j)].plan_cost;
      }
    }
  }

  std::cout << "MaxCost " << maxCost << " MinCost " << minCost << std::endl;

  GdkPixbuf* pixbuf;
  GError* error = NULL;
  guchar* pixels;
  
  pixels = (guchar*)malloc(sizeof(guchar)*plan->size_x*plan->size_y*3);

  double range = 255.0/(maxCost-minCost);

  int p = 0;
  int paddr;
  for(int j = plan->size_y-1; j>=0; j--) {
    for(int i = 0; i<plan->size_x; i++, p++) {
      paddr = p * 3;
      if(plan->cells[PLAN_INDEX(plan,i,j)].occ_state == 1) {
        pixels[paddr] = 255;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 0;
      } else if(plan->cells[PLAN_INDEX(plan,i,j)].occ_dist < plan->max_radius) {
        pixels[paddr] = 0;
        pixels[paddr+1] = 0;
        pixels[paddr+2] = 255;
      } else{
	double r = (plan->cells[PLAN_INDEX(plan,i,j)].plan_cost-minCost)*range;
	
        pixels[paddr] = 255-(guchar)r;
        pixels[paddr+1] = 255-(guchar)r;
        pixels[paddr+2] = 255-(guchar)r;

	//std::cout << "Pixel " << j << " " << i << " " <<  (int)pixels[paddr] << std::endl;
      }
    }
  }

  pixbuf = gdk_pixbuf_new_from_data(pixels, 
                                    GDK_COLORSPACE_RGB,
                                    0,8,
                                    plan->size_x,
                                    plan->size_y,
                                    plan->size_x * 3,
                                    NULL, NULL);
  
  gdk_pixbuf_save(pixbuf,fname,"png",&error,NULL);
  gdk_pixbuf_unref(pixbuf);
  free(pixels);
  
}

plan_t* copy_plan_t(const plan_t* source) {
  plan_t* dest = plan_alloc(source->abs_min_radius,
			    source->des_min_radius,
			    source->max_radius,
			    source->dist_penalty,
			    source->hysteresis_factor);
  
  assert(dest->cells == NULL);
  assert((dest->cells = 
	  (plan_cell_t*)realloc(dest->cells,
                                (source->size_x * source->size_y * sizeof(plan_cell_t))))); 
  
  memcpy(dest->cells, source->cells, (source->size_x * source->size_y * sizeof(plan_cell_t)));

  dest->scale = source->scale;
  dest->size_x = source->size_x;
  dest->size_y = source->size_y;
  dest->origin_x = source->origin_x;
  dest->origin_y = source->origin_y;

  dest->dist_kernel_width = source->dist_kernel_width;
  dest->dist_kernel = (float*)realloc(source->dist_kernel,
				      sizeof(float) * 
				      source->dist_kernel_width *
				      source->dist_kernel_width);
  memcpy(dest->dist_kernel, source->dist_kernel,
	 sizeof(float) * 
	 source->dist_kernel_width *
	 source->dist_kernel_width);

  memcpy(dest->dist_kernel_3x3, source->dist_kernel_3x3,
	 sizeof(float)*9);

  dest->waypoint_count = 0;
  plan_set_bounds(dest,0,0,dest->size_x-1,dest->size_y-1);

  plan_reset(dest);

  return dest;
  
}
