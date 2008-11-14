#include <cstdio>
#include <iterator>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <cvwimage.h>
#include "TrainBase.h"
#include "star_detector/detector.h"
#include "star_detector/keypoint.h"
#include "boost/foreach.hpp"
#include "mcutils/mcutils.h"
#include <fstream>
#include <climits>

void TrainBase::view(std::string img_url, int num_pts)
{
   // load image
   IplImage *img = cvLoadImage(img_url.c_str(), 0);
   IplImage *img_color = cvLoadImage(img_url.c_str(), 1);
   if (!img) STDOUT_ERROR("Cannot open file: " + img_url);
   printf("w=%d, h=%d; cw=%d, ch=%d\n", img->width, img->height,img_color->width, img_color->height);
   // detect keypoints
   std::vector<Keypoint> det_kpts;
   StarDetector det(cvSize(img->width, img->height),8,10);
   std::insert_iterator< std::vector<Keypoint> > inserter(det_kpts,det_kpts.begin());
printf("Detecting points\n");
   det.DetectPoints(img, inserter);
   printf("[OK] %i keypoints detected.\n", det_kpts.size());
  
   // sort descending on response magnitude and pick num_pts 
   std::sort(det_kpts.begin(), det_kpts.end());
   
   // show detected points
   std::vector<features::BaseKeypoint> base_kpts;
   features::BaseKeypoint helper(0,0,img);
   int count = 0;
   BOOST_FOREACH(Keypoint k, det_kpts) {
      cvCircle(img_color, cvPoint(k.x,k.y), 3+(1<<k.s), CV_RGB(0,255,0));
      helper.x = k.x; helper.y = k.y;
      base_kpts.push_back(helper);
      if (++count >= num_pts) break; 
   }
   if (count == num_pts)
      printf("[OK] %i strongest keypoints selected.\n", count);
   else
      printf("[NOTE] Have only %i instead of %i keypoints in base.\n", count, num_pts);
   const char *wnd_name = "keypoints to train with (q to abort)";
   cvNamedWindow(wnd_name);
   cvShowImage(wnd_name, img_color);
   if (cvWaitKey(60000) == 'q') return;
   cvDestroyWindow(wnd_name);
}

float TrainBase::train(std::string img_url, int num_trees, 
                       int depth, int views, int base_sz,
                       float phi_minmax, float theta_minmax, float lambda_plusminus)
{
   // load image
   IplImage *img = cvLoadImage(img_url.c_str(), 0);
   IplImage *img_color = cvLoadImage(img_url.c_str(), 1);
   if (!img) STDOUT_ERROR("Cannot open file: " + img_url);
   
   // detect keypoints
   std::vector<Keypoint> det_kpts;
   StarDetector det(cvSize(img->width, img->height));
   std::insert_iterator< std::vector<Keypoint> > inserter(det_kpts,det_kpts.begin());
 printf("Detecting points\n");
   det.DetectPoints(img, inserter);
   printf("[OK] %i keypoints detected.\n", det_kpts.size());
  
   // sort descending on response magnitude and pick base_sz 
   std::sort(det_kpts.begin(), det_kpts.end());
   
   // show detected points
   std::vector<features::BaseKeypoint> base_kpts;
   features::BaseKeypoint helper(0,0,img);
   int count = 0;
   BOOST_FOREACH(Keypoint k, det_kpts) {
      cvCircle(img_color, cvPoint(k.x,k.y), 3+(1<<k.s), CV_RGB(0,255,0));
      helper.x = k.x; helper.y = k.y;
      base_kpts.push_back(helper);
      if (++count >= base_sz) break; 
   }
   if (count == base_sz)
      printf("[OK] %i strongest keypoints selected.\n", count);
   else
      printf("[NOTE] Have only %i instead of %i keypoints in base.\n", count, base_sz);
   const char *wnd_name = "keypoints to train with (q to abort)";
   cvNamedWindow(wnd_name);
   cvShowImage(wnd_name, img_color);
   if (cvWaitKey(60000) == 'q') return 0;
   cvDestroyWindow(wnd_name);
   
   // train a randomized tree classifier
   features::RTreeClassifier rtc;
   features::Rng rng;
   features::PatchGenerator patch_gen(img, rng);
   patch_gen.setThetaBounds(-phi_minmax, phi_minmax);
   patch_gen.setPhiBounds(-phi_minmax, phi_minmax);   
   patch_gen.setLambdaBounds(1-lambda_plusminus, 1+lambda_plusminus);
   rtc.train(base_kpts, rng, patch_gen, num_trees, depth, views, base_kpts.size()/3);
   char rtc_url_buf[1000];
   sprintf(rtc_url_buf, "%s_t%id%iv%ib%i", img_url.c_str(), num_trees, depth, views, count);
   std::string rtc_url = findNonexistingFilename(rtc_url_buf, "rtc");
   rtc.write(rtc_url.c_str());
   
   // do naive evaluation
   cv::WImageView1_b img_view = cv::WImageView1_b(img);
   cv::WImageView1_b patch_view;
   int ok = 0;
   for (size_t i=0; i<base_kpts.size(); i++) {
      patch_view = extractPatch(img_view, base_kpts[i]);
      features::DenseSignature sig = rtc.getSignature(patch_view.Ipl());
      
      // find maximum
      int max_ind = std::distance(sig.begin(), std::max_element(sig.begin(), sig.end()));
      if (max_ind == (int)i) ok++;
   }
   float rec_rate = 1.f*ok/base_kpts.size()*100;
   printf("[OK] training completed.\n     naive recogntion rate (= quality indicator) is %.2f%%\n", rec_rate);
   
   // write info file giving details on training
   std::ofstream ofs((rtc_url + ".info").c_str());
   ofs << "Info on trained classifier. " << std::endl
       << "---" << std::endl
       << "URL............. " << rtc_url << ".rtc" << std::endl
       << "image URL....... " << img_url << std::endl
       << "size on disk.... " << (mcu::file_size(img_url + ".rtc")>>20) << " MB (uncompressed)" << std::endl
       << "# trees......... " << num_trees << std::endl
       << "tree depth...... " << depth << std::endl
       << "# views......... " << views << std::endl
       << "base size....... " << count << std::endl
       << "phi range....... " << "[" << -phi_minmax << "," << phi_minmax << "] rad" << std::endl
       << "theta range..... " << "[" << -theta_minmax << "," << theta_minmax << "] rad" << std::endl
       << "lambda range.... " << "[" << 1-lambda_plusminus << "," << 1+lambda_plusminus << "]" << std::endl
       << "naive rec rate.. " << rec_rate << "%" << std::endl;
   ofs.close();

   // clean up 
   cvReleaseImage(&img);
   cvReleaseImage(&img_color);
   
   return rec_rate;
}


std::string TrainBase::findNonexistingFilename(std::string base, std::string ext)
{
   char *cand = new char[base.size()+ext.size()+20];
   sprintf(cand, "%s.%s", base.c_str(), ext.c_str());
   int i = 1;
   
   while (mcu::file_exists(cand) && i<=INT_MAX)
      sprintf(cand, "%s(%i).%s", base.c_str(), i, ext.c_str());
   
   if (i == INT_MAX) STDOUT_ERROR("Cannot find good file name.");
   return cand;
}


