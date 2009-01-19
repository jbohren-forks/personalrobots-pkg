#A score map cell

#distance to planner's path
float32 path_dist

#distance to goal
float32 goal_dist

#grown obstacles
float32 occ_dist

#occupancy state of the cell
# -1 = free, 0 = unkown, 1 = occupied
int32 occ_state
