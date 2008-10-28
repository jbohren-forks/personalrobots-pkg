(defpackage stereo_blob_tracker
  (:use cl
        roslisp)
  (:export
   "<RECT2D>"
   "<RECT2DSTAMPED>"
  ))

(roslisp:load-if-necessary "/u/jdchen/workspace/ros-pkg/vision/stereo_blob_tracker/msg/lisp/stereo_blob_tracker/_package_Rect2D.lisp")
(roslisp:load-if-necessary "/u/jdchen/workspace/ros-pkg/vision/stereo_blob_tracker/msg/lisp/stereo_blob_tracker/_package_Rect2DStamped.lisp")
