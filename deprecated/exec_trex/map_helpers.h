#ifndef _MAP_HELPERS_H_
#define _MAP_HELPERS_H_

#include <libstandalone_drivers/plan.h>
#include <gdk-pixbuf/gdk-pixbuf.h>
// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

// computes the signed minimum difference between the two angles.
int read_map_from_image(int* size_x, int* size_y, char** mapdata, 
       			const char* fname, int negate);

void draw_map(plan_t* plan, double lx, double ly, const char* fname);

void draw_path(plan_t* plan, double lx, double ly, const char* fname);

void draw_costmap(plan_t* plan, const char* fname);

//dest should have uninitialize pointer
plan_t* copy_plan_t(const plan_t* source);

#endif
