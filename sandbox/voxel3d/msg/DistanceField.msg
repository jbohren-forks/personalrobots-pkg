# A message that contains a 3-D voxel grid with "distance field" values,
# i.e. a value corresponding to the distance to the closest obstacle

# Header with timestamp and frame information
Header header

# resolution of the grid
float64 resolution

# number of cells along each axis (x, y, z)
uint32[] size

# origin of the distance field
robot_msgs/Vector3 origin

# array of voxels with the associated distance field information
Voxel[] voxels 

# maximum obstacle expansion distance (distance field information is available
# only up to this distance, any voxels that have a larger distance are not sent
# in the voxels array
float64 max_expansion
