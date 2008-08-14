#ifndef _grasp_learner_types_h_
#define _grasp_learner_types_h_

/*! \file Just some definitions of constants that are used throughout
  the grasp_learner.
 */

namespace grasp_learner {

const char VOXEL_EMPTY = 2;
const char VOXEL_OCCUPIED = 1;

/*!
Specifies whether the voxel grid stored as an octree is either:

- BINARY: just has occupied / unseen voxels
- CARVED: has occupied / known empty / unseen voxels
- FREE: grasp point is at 0,0,0, but the orientation of the gripper is unknown
- ALIGNED: voxel grid is aligned with the orientation of the gripper for the grasp point
*/
enum OctreeType{BINARY_FREE, BINARY_ALIGNED, CARVED_FREE, CARVED_ALIGNED};

} //namespace grasp_learner

#endif
