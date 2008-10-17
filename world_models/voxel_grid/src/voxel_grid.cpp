#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define sign(x) ((x) > 0 ? 1 : -1)

class VoxelGrid{
public:
  VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);
  ~VoxelGrid();

  void markVoxel(unsigned int x, unsigned int y, unsigned int z);
  void clearVoxel(unsigned int x, unsigned int y, unsigned int z);
 
  void markVoxelLine(unsigned int x0, unsigned y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1);
  void clearVoxelLine(unsigned int x0, unsigned int y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1);
  void clearVoxelLineInMap(unsigned int x0, unsigned int y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1, unsigned char *map_2d);

  bool getVoxel(unsigned int x, unsigned int y, unsigned int z);
  bool getVoxelColumn(unsigned int x, unsigned int y); //Are there any obstacles at that (x, y) location in the grid?

  void printVoxelGrid();
  void printColumnGrid();

private:
  int size_x, size_y, size_z;
  int *data;
};

VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
{
  this->size_x = size_x; 
  this->size_y = size_y; 
  this->size_z = size_z; 

  if(size_z > 32)
    printf("Error, this implementation can only support up to 32 z values"); 
 
  data = new int[size_x * size_y];
  bzero(data, sizeof(int) * size_x * size_y);
}

VoxelGrid::~VoxelGrid()
{
  delete(data);
}

void VoxelGrid::markVoxel(unsigned int x, unsigned int y, unsigned int z)
{
  if(x >= size_x || y >= size_y || z >= size_z){
    printf("Error, voxel out of bounds.\n");
    return;
  }
  data[y * size_x + x] |= 1<<z;
}

void VoxelGrid::clearVoxel(unsigned int x, unsigned int y, unsigned int z)
{
  if(x >= size_x || y >= size_y || z >= size_z){
    printf("Error, voxel out of bounds.\n");
    return;
  }
  data[y * size_x + x] &= ~(1<<z);
}

#define bresenham_3d_macro(abs_da, abs_db, abs_dc, error_b, error_c, inc_a, inc_b, inc_c, mark_voxel) \
for(int i = 0; i < abs_da; i++){ \
 mark_voxel; \
 inc_a; \
 error_b += abs_db; \
 error_c += abs_dc; \
 if(error_b >= abs_da){ \
  inc_b; \
  error_b -= abs_da; \
 } \
 if(error_c >= abs_da){ \
  inc_c; \
  error_c -= abs_da; \
 } \
} \
mark_voxel; \

void VoxelGrid::markVoxelLine(unsigned int x0, unsigned y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1){
  if(x0 >= size_x || y0 >= size_y || z0 >= size_z || x1>=size_x || y1>=size_y || z1>=size_z){
    printf("Error, line endpoint out of bounds.\n");
    return;
  }
  int dx = x1 - x0;
  int dy = y1 - y0;
  int dz = z1 - z0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);
  unsigned int abs_dz = abs(dz);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * size_x;
  int shift_dz = sign(dz);

  unsigned int z_mask = 1 << z0;
  unsigned int offset = y0 * size_x + x0;

  if(abs_dx >= abs_dy && abs_dx >= abs_dz){
    int error_y = abs_dx/2, error_z = abs_dx/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask <<= 1, data[offset] |= z_mask);
    }
    else{
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask >>= 1, data[offset] |= z_mask);
    }
    return;
  }
  if(abs_dy >= abs_dz){
    int error_x = abs_dy / 2, error_z = abs_dy/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask <<= 1, data[offset] |= z_mask);
    }
    else{
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask >>= 1, data[offset] |= z_mask);
    }
    return;
  }
  else{
    int error_x = abs_dz / 2, error_y = abs_dz / 2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, data[offset] |= z_mask);
    }
    else{
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, data[offset] |= z_mask);
    }
    return;
  }
}

void VoxelGrid::clearVoxelLine(unsigned int x0, unsigned y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1){
  if(x0 >= size_x || y0 >= size_y || z0 >= size_z || x1>=size_x || y1>=size_y || z1>=size_z){
    printf("Error, line endpoint out of bounds.\n");
    return;
  }
  int dx = x1 - x0;
  int dy = y1 - y0;
  int dz = z1 - z0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);
  unsigned int abs_dz = abs(dz);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * size_x;
  int shift_dz = sign(dz);

  unsigned int z_mask = 1 << z0;
  unsigned int offset = y0 * size_x + x0;

  if(abs_dx >= abs_dy && abs_dx >= abs_dz){
    int error_y = abs_dx/2, error_z = abs_dx/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask <<= 1, data[offset] &= ~z_mask);
    }
    else{
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask >>= 1, data[offset] &= ~z_mask);
    }
    return;
  }
  if(abs_dy >= abs_dz){
    int error_x = abs_dy / 2, error_z = abs_dy/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask <<= 1, data[offset] &= ~z_mask);
    }
    else{
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask >>= 1, data[offset] &= ~z_mask);
    }
    return;
  }
  else{
    int error_x = abs_dz / 2, error_y = abs_dz / 2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, data[offset] &= ~z_mask);
    }
    else{
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, data[offset] &= ~z_mask);
    }
    return;
  }
}

void VoxelGrid::clearVoxelLineInMap(unsigned int x0, unsigned y0, unsigned int z0, unsigned int x1, unsigned int y1, unsigned int z1, unsigned char *map_2d){
  if(map_2d == 0){
    clearVoxelLine(x0, y0, z0, x1, y1, z1);
    return;
  }
  if(x0 >= size_x || y0 >= size_y || z0 >= size_z || x1>=size_x || y1>=size_y || z1>=size_z){
    printf("Error, line endpoint out of bounds.\n");
    return;
  }
  int dx = x1 - x0;
  int dy = y1 - y0;
  int dz = z1 - z0;

  unsigned int abs_dx = abs(dx);
  unsigned int abs_dy = abs(dy);
  unsigned int abs_dz = abs(dz);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy) * size_x;
  int shift_dz = sign(dz);

  unsigned int z_mask = 1 << z0;
  unsigned int offset = y0 * size_x + x0;

  if(abs_dx >= abs_dy && abs_dx >= abs_dz){
    int error_y = abs_dx/2, error_z = abs_dx/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask <<= 1, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    else{
      bresenham_3d_macro(abs_dx, abs_dy, abs_dz, error_y, error_z, offset += offset_dx, offset += offset_dy, z_mask >>= 1, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    return;
  }
  if(abs_dy >= abs_dz){
    int error_x = abs_dy / 2, error_z = abs_dy/2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask <<= 1, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    else{
      bresenham_3d_macro(abs_dy, abs_dx, abs_dz, error_x, error_z, offset += offset_dy, offset += offset_dx, z_mask >>= 1, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    return;
  }
  else{
    int error_x = abs_dz / 2, error_y = abs_dz / 2;
    if(shift_dz > 0){
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    else{
      bresenham_3d_macro(abs_dz, abs_dx, abs_dy, error_x, error_y, z_mask <<= 1, offset += offset_dx, offset += offset_dy, {data[offset] &= ~z_mask; if(data[offset]==0) map_2d[offset] = 0;});
    }
    return;
  }
}


bool VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
{
  if(x >= size_x || y >= size_y || z >= size_z){
    printf("Error, voxel out of bounds.\n");
    return false;
  }
  return data[y * size_x + x] & (1<<z); 
}

bool VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y)
{
  if(x >= size_x || y >= size_y){
    printf("Error, voxel out of bounds.\n");
    return false;
  }
  return data[y * size_x + x] > 0;
}

void VoxelGrid::printVoxelGrid(){
  for(int z = 0; z < size_z; z++){
    printf("Layer z = %d:\n",z);
    for(int y = 0; y < size_y; y++){
      for(int x = 0 ; x < size_x; x++){
        printf((getVoxel(x, y, z))? "#" : " ");
      }
      printf("|\n");
    } 
  }
}

void VoxelGrid::printColumnGrid(){
  printf("Column view:\n");
  for(int y = 0; y < size_y; y++){
    for(int x = 0 ; x < size_x; x++){
      printf((getVoxelColumn(x, y))? "#" : " ");
    }
    printf("|\n");
  } 
}

int main(int argc, char *argv[]){
  printf("Initializing voxel grid.\n");
  int size_x = 50, size_y = 10, size_z = 5;
  VoxelGrid *v = new VoxelGrid(size_x, size_y, size_z);

  unsigned char *costMap = new unsigned char[size_x * size_y]; //initialize cost map
  for(int x = 0; x < size_x; x++){
    for(int y = 0; y < size_y; y++){
      costMap[y * size_x + x] = 128;
    }
  }


  //Put a "tabletop" into the scene.  A flat rectangle of set voxels at z = 12.
  int table_z = 1;
  int table_x_min = 5, table_x_max = 15;
  int table_y_min = 0, table_y_max = 3;
  for(int x = table_x_min; x <= table_x_max; x++){
    v->markVoxelLine(x, table_y_min, table_z, x, table_y_max, table_z);
  }

  //Add a few synthetic obstacles (diagonal line on the floor) just to demonstrate line drawing
  v->markVoxelLine(size_x - 1, size_y - 1, 0, 0, 0, 0);

  //clear a scan next to the table (will clear obstacles out)
  v->clearVoxelLineInMap(table_x_max + 1, 0, table_z, table_x_max + 1, size_y - 1, table_z, costMap);

  //clear a scan over the table (will not clear obstacles out until it passes the edge of the table)
  v->clearVoxelLineInMap(table_x_min + 1, 0, table_z + 1, table_x_min + 1, size_y - 1, table_z, costMap);

  //clear a scan through the table (will only clear obstacles where it passes through the table
  v->clearVoxelLineInMap(table_x_min, table_y_min, 0, table_x_max, table_y_max, size_z - 1, costMap);

  //clear a scan that clears out a whole line of the table, to show that it doesnt clear out the diagonal line underneath it
  v->clearVoxelLineInMap(table_x_max - 2, 0, table_z, table_x_max - 2, size_y - 1, table_z, costMap);

  //Visualize the output
  v->printVoxelGrid();
  v->printColumnGrid();

  printf("CostMap:\n===========\n");
  for(int y = 0; y < size_y; y++){
    for(int x = 0; x < size_x; x++){
      printf((costMap[y * size_x + x] > 0 ? "#" : " "));
    }printf("|\n");
  }
}
