Index: stereocam.cpp
===================================================================
--- stereocam.cpp	(revision 6000)
+++ stereocam.cpp	(working copy)
@@ -54,6 +54,9 @@
 StereoCam::StereoCam()
 {
   stIm = new StereoData();
+  buf = NULL;
+  flim = NULL;
+  frim = NULL;
 }
 
 StereoCam::~StereoCam()
