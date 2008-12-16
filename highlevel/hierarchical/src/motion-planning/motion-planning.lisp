(defpackage motion-planning
  (:documentation "Package motion-planning (mplan)

Basic operations related to motion planning and configuration spaces.

Types
-----
<simple-cspace>

Cspace Operations
-----------------
get-path
distance
min-conf-distance
get-cspace
directly-reachable-confs
all-confs
free-space-sampler
edge-biased-sampler

Collision
---------
is-free
path-collides
*default-path-collision-resolution*

Geometric motion planning
-------------------------
2d-polygonal-visibility-graph
connect-using-visibility-graph

Roadmap
-------
construct-simple-roadmap
connect-using-roadmap
visibility-roadmap-paths
confs->path

Simple cspace
-------------
robot
obstacles

Other
-----
move

")
  (:export
   <simple-cspace>
   
   get-path
   distance
   min-conf-distance
   get-cspace
   directly-reachable-confs
   all-confs
   free-space-sampler
   edge-biased-sampler
   
   is-free
   path-collides
   *default-path-collision-resolution*

   2d-polygonal-visibility-graph
   connect-using-visibility-graph
   
   construct-simple-roadmap
   connect-using-roadmap
   visibility-roadmap-paths
   confs->path
   
   robot
   obstacles)
 (:use
   cl
   queue
   geometry
   graph
   set
   utils
   lin-alg
   prob)
  (:nicknames mplan))

(in-package mplan)

