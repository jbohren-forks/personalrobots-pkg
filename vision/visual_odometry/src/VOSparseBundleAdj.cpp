/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * VOSparseBundleAdj.cpp
 *
 *  Created on: Sep 17, 2008
 *      Author: jdchen
 */

#include "VOSparseBundleAdj.h"
#include "CvMatUtils.h"
#include "CvTestTimer.h"
#include "LevMarqSparseBundleAdj.h"

using namespace cv::willow;

// boost
#include <boost/foreach.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/framework/features.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
using namespace boost::accumulators;

// opencv
#include <opencv/cv.h>

#include <queue>

#define DISPLAY 1
#define DEBUG 1

VOSparseBundleAdj::VOSparseBundleAdj(const CvSize& imageSize,
    int num_fixed_frames, int num_free_frames):
  PathRecon(imageSize),
  full_free_window_size_(num_free_frames),
  full_fixed_window_size_(num_fixed_frames),
  mNumIteration(DefaultNumIteration),
  mTrackId(0),
  levmarq_sba_(NULL)
{
  mPoseEstimator.setInlierErrorThreshold(4.0);
}

VOSparseBundleAdj::~VOSparseBundleAdj() {
  delete levmarq_sba_;
}

void VOSparseBundleAdj::updateSlideWindow() {
  while(mActiveKeyFrames.size() > (unsigned int)this->full_free_window_size_ ||
      // make sure there is at least one fixed frame.
      (mActiveKeyFrames.size()>1 && mFramePoses.size() == mActiveKeyFrames.size())
      ){
    PoseEstFrameEntry* frame = mActiveKeyFrames.front();
    mActiveKeyFrames.pop_front();
    delete frame;
  }

#if DEBUG==1
  cout << "Current slide window: [";
  BOOST_FOREACH(const PoseEstFrameEntry* frame, mActiveKeyFrames) {
    cout << frame->mFrameIndex << ",";
  }
  cout << "]" << endl;
#endif

  int numWinSize = mActiveKeyFrames.size();
  assert(numWinSize<=this->full_free_window_size_);
  int numFixedFrames = std::min(this->full_fixed_window_size_, numWinSize-full_fixed_window_size_);
  numFixedFrames = std::max(1, numFixedFrames);
#if DEBUG==1
  cout << "free window size: "<< numWinSize << ", # fixed win: " << numFixedFrames << endl;
#endif

}

bool VOSparseBundleAdj::track(queue<StereoFrame>& inputImageQueue) {
  bool status = false;
  vector<FramePose*> free_frames;
  vector<FramePose*> fixed_frames;

  while (inputImageQueue.size()>0) {
    bool newKeyFrame = trackOneFrame(inputImageQueue, mFrameSeq);
    if (newKeyFrame == true) {
      // update the sliding window
      // slide the window by getting rid of old windows
      updateSlideWindow();

      // update the tracks
      status = updateTracks(mActiveKeyFrames, mTracks);

#if 1 // 1: use sba;  0: no sba
      fillFrames(&mFramePoses, mActiveKeyFrames.front()->mFrameIndex,
          mActiveKeyFrames.back()->mFrameIndex, (int)mActiveKeyFrames.size(),
          full_fixed_window_size_, &mTracks, &free_frames, &fixed_frames);

      LevMarqSparseBundleAdj::ErrorCode err_code = levmarq_sba_->optimize(&fixed_frames, &free_frames, &mTracks);
      if (err_code == LevMarqSparseBundleAdj::InputError) {
        cout << "Input error: either fixed win or free win is empty" << endl;
      }
      cout << "initial cost: " << levmarq_sba_->initial_cost_ << " final cost: " << levmarq_sba_->accepted_cost_ << endl;
      cout << "# good updates: " << levmarq_sba_->num_good_updates_ << " # retractions: " << levmarq_sba_->num_retractions_ << endl;

      // update the current key frame
      if (err_code == LevMarqSparseBundleAdj::Normal ||
          err_code == LevMarqSparseBundleAdj::NotImproved) {
    	  int fi = mActiveKeyFrames.back()->mFrameIndex;
    	  FramePose* fp = map_index_to_FramePose_[fi];
    	  cvCopy(&fp->transf_local_to_global_,
    			  &(mActiveKeyFrames.back()->transf_local_to_global_) );
    	  cvCopy(&fp->transf_local_to_global_, &(this->mTransform));
    	  status = true;
      } else {
        status = false;
      }
#endif

      updateStat2();

      if (mVisualizer) {
        PoseEstFrameEntry* pee = mActiveKeyFrames.back();
        PoseEstFrameEntry& peef = *pee;
        cout << pee->mFrameIndex << endl;
        SBAVisualizer* vis = (SBAVisualizer*)mVisualizer;
        vis->recordFrameIds(&fixed_frames, &free_frames);
        vis->drawTrack(peef);
      }
    }
    visualize();

  }
  return status;
}

void VOSparseBundleAdj::purgeTracks(int oldtestFrameIndex){
  mTracks.purge(oldtestFrameIndex);
}

bool VOSparseBundleAdj::updateTracks(deque<PoseEstFrameEntry*> & frames,
    PointTracks & tracks)
{
  bool status = true;
  int lastFrame = tracks.current_frame_index_;

  // loop thru all the frames in the current sliding window
  BOOST_FOREACH( PoseEstFrameEntry* frame, frames) {
    assert(frame != NULL);
    // skip over the frames that are not newer than the last frame
    // the tracks have been updated against.
    if (frame->mFrameIndex <= lastFrame) {
      continue;
    }
    // loop thru all the inlier pairs of this frame
    for (int i=0; i<frame->mNumInliers; i++) {
      // a pair of inliers will end up being either an extension to
      // existing track, or the start of a new track.

      // loop thru all the existing tracks to
      // - extending old tracks,
      // - adding new tracks, and
      // - remove old tracks that will not be used anymore
      if (extendTrack(tracks, *frame, i) == false) {
        // no track extended. Add the new pair as a new track
        addTrack(tracks, *frame, i);
      }
    }
  }
  if (frames.size()>0)
    tracks.current_frame_index_ = frames.back()->mFrameIndex;

  // loop thru all tracks and get purge the tracks that
  // fall out of the free window totally
  purgeTracks(frames.front()->mFrameIndex);
  return status;
}

bool VOSparseBundleAdj::extendTrack(PointTracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  BOOST_FOREACH( PointTrack* track, tracks.tracks_ ) {
    if (// The last frame of this track is not the same as
        // the last frame of this inlier pair. Skip it.
        track->lastFrameIndex() == frame.mLastKeyFrameIndex ) {
      PointTrackObserv* observ = track->back();
      if (// keypoint index needs to match skip to next track
          observ->keypoint_index_ == p.second) {
        // found a matching track. Extend it.
        CvPoint3D64f coord1 = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
        CvPoint3D64f coord0 = CvMatUtils::rowToPoint(*frame.mInliers0, inlierIndex);
        PointTrackObserv* obsv = new PointTrackObserv(frame.mFrameIndex, coord1, p.first);

        track->extend(obsv);
        status = true;
#if DEBUG==1
        printf("extend track %3d => f=%3d, pi=%3d:[%3d, %3d]->f=%3d, pi=%3d:[%3d, %3d]\n",
            track->id_,
            frame.mLastKeyFrameIndex, p.second,
            (int)(coord0.x+.5), (int)(coord0.y+.5),
            frame.mFrameIndex,        p.first,
            (int)(coord1.x+.5), (int)(coord1.y+.5));
#endif
        break;
      }
    }
  }
  return status;
}
bool VOSparseBundleAdj::addTrack(PointTracks& tracks, PoseEstFrameEntry& frame,
    int inlierIndex){
  bool status = false;
  int inlier = frame.mInlierIndices[inlierIndex];
  std::pair<int, int>& p = frame.mTrackableIndexPairs->at(inlier);
  CvPoint3D64f dispCoord0 = CvMatUtils::rowToPoint(*frame.mInliers0, inlierIndex);
  CvPoint3D64f dispCoord1 = CvMatUtils::rowToPoint(*frame.mInliers1, inlierIndex);
  PointTrackObserv* obsv0 = new PointTrackObserv(frame.mLastKeyFrameIndex, dispCoord0, p.second);
  PointTrackObserv* obsv1 = new PointTrackObserv(frame.mFrameIndex,        dispCoord1, p.first);
  // initial estimate of the position of the 3d point in Cartesian coord.
  CvMat disp1;
  cvGetRow(frame.mInliers1, &disp1, inlierIndex);
  CvPoint3D64f cartCoord1; //< Estimated global Cartesian coordinate.
  CvMat _cartCoord1 = cvMat(1, 3, CV_64FC1, &cartCoord1);
  dispToGlobal(disp1, frame.transf_local_to_global_, _cartCoord1);
  PointTrack* newtrack = new PointTrack(obsv0, obsv1, cartCoord1, mTrackId++);
  tracks.tracks_.push_back(newtrack);

#if DEBUG==1
  int trackId = mTrackId-1;
  printf("add new track %3d => %3d, %3d:[%8.2f, %8.2f, %8.2f]\n", trackId, inlierIndex, inlier,
      cartCoord1.x, cartCoord1.y, cartCoord1.z);
  printf("f=%3d, pi=%3d: [%3d, %3d], f=%3d, pi=%3d: [%3d, %3d]\n",
      frame.mLastKeyFrameIndex, p.second,  (int)(dispCoord0.x+.5), (int)(dispCoord0.y+.5),
      frame.mFrameIndex,        p.first,   (int)(dispCoord1.x+.5), (int)(dispCoord1.y+.5));
#endif
  return status;
}

void VOSparseBundleAdj::fillFrames(const vector<FramePose*>* frames,
    int lowest_free_frame_index,
    int highest_free_frame_index,
    int free_window_size,
    int max_fixed_window_size,
    const PointTracks* tracks,
    vector<FramePose*>* free_frames,
    vector<FramePose*>* fixed_frames) {
  free_frames->clear();
  fixed_frames->clear();

  vector<FramePose*> rfree_frames;
  vector<FramePose*> rfixed_frames;

  int oldest_frame_index_in_tracks = tracks->oldest_frame_index_in_tracks_;
  int free_win_count = 0;
  int fixed_win_size = 0;

  BOOST_REVERSE_FOREACH(FramePose* fp, *frames) {
    assert(fp->mIndex<=highest_free_frame_index);
    if (fp->mIndex >= lowest_free_frame_index) {
      free_win_count++;
      rfree_frames.push_back(fp);
    } else if (fp->mIndex >= oldest_frame_index_in_tracks ) {
      fixed_win_size++;
      rfixed_frames.push_back(fp);
      if (fixed_win_size>=max_fixed_window_size) {
        // done. fixed window is full
        break;
      }
    } else {
      // done. fixed window is not full but no more window
      // referenced in the tracks.
      break;
    }
  }
  assert (free_win_count == free_window_size);
  // now reverse vectors of free frames and fixed frames
  BOOST_REVERSE_FOREACH(FramePose* fp, rfree_frames) {
    free_frames->push_back(fp);
  }
  BOOST_REVERSE_FOREACH(FramePose* fp, rfixed_frames) {
    fixed_frames->push_back(fp);
  }
}


void SBAVisualizer::drawTrackingCanvas(
    const PoseEstFrameEntry& lastFrame,
    const PoseEstFrameEntry& frame
){
  Parent::drawTrackingCanvas(lastFrame, frame);

  drawTrack(frame);
}

void SBAVisualizer::drawTrack(const PoseEstFrameEntry& frame){
  drawTrackTrajectories(frame.mFrameIndex);
}

void SBAVisualizer::drawTrackTrajectories(int frame_index) {
  // draw all the tracks on canvasTracking
  CvMat* mat0 = cvCreateMat(4, 4, CV_64FC1);
  CvMat* mat1 = cvCreateMat(4, 4, CV_64FC1);

  if (tracks && map_index_to_FramePose_ && framePoses ) {
    BOOST_FOREACH( const PointTrack* track, this->tracks->tracks_ ){
      const PointTrackObserv* lastObsv = track->back();
      const CvScalar colorFixedFrame = CvMatUtils::blue;
      const CvScalar colorEstimated  = CvMatUtils::magenta;
      CvScalar colorFreeFrame;

      // drawing it green if the last observation is over the current frame,
      // namely a track that is still extending.
      // drawing it yellow otherwise. Namely a track that is phasing out.
      if (lastObsv->frame_index_ < frame_index) {
        colorFreeFrame = CvMatUtils::yellow;
      } else {
        colorFreeFrame  = CvMatUtils::green;
      }

      /// @todo compute the estimated trajectory here. Not the right place.
#if 1
      BOOST_FOREACH(PointTrackObserv* obsv, *track) {
        boost::unordered_map<int, FramePose*>::const_iterator it;
        it = map_index_to_FramePose_->find(obsv->frame_index_);
        if (it == map_index_to_FramePose_->end()) {
#if DEBUG==1
          printf("frame %d is missing in frame lists but referenced by track %d\n",
              obsv->frame_index_, track->id_);
#endif
          continue;
        }
        const FramePose* fp = it->second;
        assert(obsv->frame_index_ == fp->mIndex);

        CvMat* mat_global_to_disp;

        /// @todo jdc debugging - turn off the following branch
        if (false && fp->transf_global_to_disp_) {
          mat_global_to_disp = fp->transf_global_to_disp_;
        } else {
          // compute it here.
          mat_global_to_disp = mat1;
          CvMatUtils::invertRigidTransform(&fp->transf_local_to_global_,
              mat0);
          cvMatMul(threeDToDisparity_, mat0, mat_global_to_disp);

        }

        CvMat mat_coord = cvMat(1, 1, CV_64FC3, (double *)&track->coordinates_);
        CvPoint3D64f disp_coord_est;
        //      CvMat mat_disp_coord_est = cvMat(1, 1, CV_64FC3, &obsv->disp_coord_est_);
        CvMat mat_disp_coord_est = cvMat(1, 1, CV_64FC3, &disp_coord_est);

        cvPerspectiveTransform(&mat_coord, &mat_disp_coord_est, mat_global_to_disp);
        obsv->disp_res_.x = disp_coord_est.x - obsv->disp_coord_.x;
        obsv->disp_res_.y = disp_coord_est.y - obsv->disp_coord_.y;
        obsv->disp_res_.z = disp_coord_est.z - obsv->disp_coord_.z;
      }
#endif

      int thickness = 1;
      deque<PointTrackObserv*>::const_iterator iObsv = track->begin();
      PointTrackObserv* obsv = *iObsv;
      CvPoint pt0     = CvStereoCamModel::dispToLeftCam(obsv->disp_coord_);
      CvPoint3D64f disp_coord_est;
      disp_coord_est.x = obsv->disp_coord_.x + obsv->disp_res_.x;
      disp_coord_est.y = obsv->disp_coord_.y + obsv->disp_res_.y;
      disp_coord_est.z = obsv->disp_coord_.z + obsv->disp_res_.z;

      CvPoint est_pt0 = CvStereoCamModel::dispToLeftCam(disp_coord_est);
      //    CvPoint est_pt0 = CvStereoCamModel::dispToLeftCam((*iObsv)->disp_coord_est_);
#if DEBUG==1
      int i=0;
      printf("track %3d, len=%3d, [%7.2f, %7.2f, %7.2f]\n", track->id_, track->size(),
          track->coordinates_.x, track->coordinates_.y, track->coordinates_.z);
      printf("%3d: fi=%d, [%3d, %3d] <=> est: ", i++, (*iObsv)->frame_index_, pt0.x, pt0.y);
      printf("[%3d, %3d] \n", est_pt0.x, est_pt0.y);
#endif
      for (iObsv++; iObsv != track->end(); iObsv++) {
        obsv = *iObsv;
        CvScalar color;
        if (free_frame_id_set_.find(obsv->frame_index_) != free_frame_id_set_.end()) {
          color = colorFreeFrame;
        } else
        if (fixed_frame_id_set_.find(obsv->frame_index_) != fixed_frame_id_set_.end()) {
          color = colorFixedFrame;
        } else {
          // probably a frame that are ignored
          color = colorFixedFrame;
        }
        CvPoint pt1     = CvStereoCamModel::dispToLeftCam(obsv->disp_coord_);
        disp_coord_est.x = obsv->disp_coord_.x + obsv->disp_res_.x;
        disp_coord_est.y = obsv->disp_coord_.y + obsv->disp_res_.y;
        disp_coord_est.z = obsv->disp_coord_.z + obsv->disp_res_.z;

        CvPoint est_pt1 = CvStereoCamModel::dispToLeftCam(disp_coord_est);
        //      CvPoint est_pt1 = CvStereoCamModel::dispToLeftCam((*iObsv)->disp_coord_est_);

        // draw the line between estimations, re-projected
        cvLine(canvasTracking.Ipl(), est_pt0, est_pt1, colorEstimated, thickness, CV_AA);

        // draw the line between observations
        cvLine(canvasTracking.Ipl(), pt0, pt1, color, thickness, CV_AA);

        // setting up for next iteration
        pt0     = pt1;
        est_pt0 = est_pt1;

#if DEBUG==1
        printf("%3d: fi=%d, [%3d, %3d] <=> est: ", i++, obsv->frame_index_, pt0.x, pt0.y);
        printf("[%3d, %3d] \n", est_pt0.x, est_pt0.y);
#endif
      }
    }
  }
  canvasTrackingRedrawn = true;
  cvReleaseMat(&mat0);
  cvReleaseMat(&mat1);
}

void SBAVisualizer::recordFrameIds(const vector<FramePose*>* fixed_frames,
    const vector<FramePose*>* free_frames) {
  fixed_frame_id_set_.clear();
  BOOST_FOREACH(const FramePose* fp, *fixed_frames) {
    fixed_frame_id_set_.insert(fp->mIndex);
  }
  free_frame_id_set_.clear();
  BOOST_FOREACH(const FramePose* fp, *free_frames) {
    free_frame_id_set_.insert(fp->mIndex);
  }
}

void SBAVisualizer::show(IplImage* im, const vector<FramePose*>& fixed_frames,
    const vector<FramePose*>& free_frames, const PointTracks& tracks) {
  reset();
  // set frame poses, point tracks and maps from index to frame poses
  vector<FramePose* > frame_poses;
#if DEBUG==1
  cout << ("Inserting fixed frames [");
#endif
  BOOST_FOREACH(FramePose *fp, fixed_frames) {
    frame_poses.push_back(fp);
    map_index_to_FramePose_->insert(make_pair(fp->mIndex, fp));
#if DEBUG==1
    printf("%d ", fp->mIndex);
#endif
  }
#if DEBUG==1
  cout << endl;
#endif

#if DEBUG==1
  cout << ("Inserting free frames [");
#endif
  BOOST_FOREACH(FramePose *fp, free_frames) {
    frame_poses.push_back(fp);
    map_index_to_FramePose_->insert(make_pair(fp->mIndex,fp));
#if DEBUG==1
    printf("%d ", fp->mIndex);
#endif
  }
#if DEBUG==1
  cout << endl;
#endif
  this->framePoses = &frame_poses;
  this->tracks = &tracks;
  FramePose* fp = free_frames.back();
  int current_frame_index = fp->mIndex;

  if (im) {
    canvasTracking.SetIpl(im);
  } else {
    // make sure the image buffers is allocated to the right sizes
    canvasTracking.Allocate(640, 480);
    // clear the image
    cvSet(canvasTracking.Ipl(), cvScalar(0,0,0));
  }
  { // annotation on the canvas
    char info[256];
    CvPoint org = cvPoint(0, 475);
    CvFont font;
    cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, .5, .4);
    double x = -cvmGet(&fp->transf_local_to_global_, 0, 3);
    double y = -cvmGet(&fp->transf_local_to_global_, 1, 3);
    double z = -cvmGet(&fp->transf_local_to_global_, 2, 3);
    cout << "last frame " << fp->mIndex << " "<<x<<" "<<y<<" "<<z<<endl;
    CvMatUtils::printMat(&fp->transf_local_to_global_);


    sprintf(info, "%04d, [%5.2f,%5.2f,%5.2f], nTrcks=%d",
        current_frame_index, x,y,z, tracks.tracks_.size());

    cvPutText(canvasTracking.Ipl(), info, org, &font, CvMatUtils::yellow);
  }
  cout << "All The Tracks"<<endl;
  tracks.print();
  sprintf(poseEstFilename,  "%s/poseEst-%04d.png", outputDirname.c_str(),
      current_frame_index);
  recordFrameIds(&fixed_frames, &free_frames);
  drawTrackTrajectories(current_frame_index);
  this->show();
  save();
  reset();
}

void VOSparseBundleAdj::Stat2::print() {
  CvMat mat_num_tracks = cvMat(1, numTracks.size(), CV_32SC1, &(numTracks[0]));
  CvScalar average = cvAvg(&mat_num_tracks);
  double   min_val, max_val;
  cvMinMaxLoc(&mat_num_tracks, &min_val, &max_val);
  printf("min numTracks    = %d, ", (int)min_val);
  printf("max numTracks    = %d, ", (int)max_val);
  printf("avg numTracks    = %f, ", average.val[0]);
  printf("\n");

  CvMat mat_minTrackLens = cvMat(1, minTrackLens.size(), CV_32SC1, &(minTrackLens[0]));
  average = cvAvg(&mat_minTrackLens);
  cvMinMaxLoc(&mat_minTrackLens, &min_val, &max_val);
  printf("min minTrackLens = %d, ", (int)min_val);
  printf("max minTrackLens = %d, ", (int)max_val);
  printf("avg minTrackLens = %f, ", average.val[0]);
  printf("\n");
  CvMat mat_maxTrackLens = cvMat(1, maxTrackLens.size(), CV_32SC1, &(maxTrackLens[0]));
  average = cvAvg(&mat_maxTrackLens);
  cvMinMaxLoc(&mat_maxTrackLens, &min_val, &max_val);
  printf("min maxTrackLens = %d, ", (int)min_val);
  printf("max maxTrackLens = %d, ", (int)max_val);
  printf("avg maxTrackLens = %f, ", average.val[0]);
  printf("\n");

#if 0 // I would prefer using the following if I can figure out how to
  // avoid the excessive compilation warnings.
  // The accumulator set which will calculate the properties for us:
  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc;

  // Use std::for_each to accumulate the statistical properties:
  acc = std::for_each( numTracks.begin(), numTracks.end(), acc );
  printf("min numTracks = %d, ", extract::min( acc ));
  printf("max numTracks = %d, ", extract::max( acc ));
  printf("avg numTracks = %f, ", extract::mean( acc ));
  printf("\n");

  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc2;
  acc2 = std::for_each( minTrackLens.begin(), minTrackLens.end(), acc2 );
  printf("min mintracklen = %d, ", extract::min( acc2 ));
  printf("max mintracklen = %d, ", extract::max( acc2 ));
  printf("avg mintracklen = %f, ", extract::mean( acc2 ));
  printf("\n");
  accumulator_set< int, stats<tag::min, tag::mean, tag::max> > acc3;
  acc3 = std::for_each( maxTrackLens.begin(), maxTrackLens.end(), acc3 );
  printf("min maxtracklen = %d, ", extract::min( acc3 ));
  printf("max maxtracklen = %d, ", extract::max( acc3 ));
  printf("avg maxtracklen = %f, ", extract::mean( acc3 ));
  printf("\n");

  accumulator_set< double, stats<tag::min, tag::mean, tag::max> > acc1;
  acc1 = std::for_each( avgTrackLens.begin(), avgTrackLens.end(), acc1 );
  printf("min avgTracklen = %f, ", extract::min( acc1 ));
  printf("max avgTracklen = %f, ", extract::max( acc1 ));
  printf("mean avgTracklen = %f, ", extract::mean( acc1 ));
  printf("\n");
#endif

  printf("Histogram of track lengths  [len, count, avg count]:\n");
  int len = 0;
  int numKeyFrames = minTrackLens.size();
  BOOST_FOREACH( const int count, trackLenHisto) {
    printf("%2d  %5d, %7.2f\n", len, count, (double)count/(double)numKeyFrames);
    len++;
  }
}

void VOSparseBundleAdj::updateStat2() {
  mStat2.update(mTracks);
}

void VOSparseBundleAdj::Stat2::update(const PointTracks& point_track) {
  int nTracks, maxLen, minLen;
  double avgLen;
  vector<int> lenHisto;
  point_track.stats(&nTracks, &maxLen, &minLen, &avgLen, &lenHisto);
  if (nTracks>0) {
    numTracks.push_back(nTracks);
    maxTrackLens.push_back(maxLen);
    minTrackLens.push_back(minLen);
    avgTrackLens.push_back(avgLen);

    if (lenHisto.size()>trackLenHisto.size()) {
      trackLenHisto.resize(lenHisto.size());
    }
    int len=0;
    BOOST_FOREACH( const int count, lenHisto ) {
      trackLenHisto.at(len) += count;
      len++;
    }
  }
}
void VOSparseBundleAdj::Stat2::clear() {
  numTracks.clear();
  maxTrackLens.clear();
  minTrackLens.clear();
  avgTrackLens.clear();
  trackLenHisto.clear();
}

void VOSparseBundleAdj::setCameraParams(double Fx, double Fy, double Tx,
    double Clx, double Crx, double Cy, double dispUnitScale) {
  Parent::setCameraParams(Fx, Fy, Tx, Clx, Crx, Cy, dispUnitScale);
  CvMat cartToDisp;
  CvMat dispToCart;
  mPoseEstimator.getProjectionMatrices(&cartToDisp, &dispToCart);
  delete levmarq_sba_;
  int num_updates = 10;
  CvTermCriteria term_criteria =
    cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,num_updates,DBL_EPSILON);

  levmarq_sba_ = new LevMarqSparseBundleAdj(&dispToCart, &cartToDisp,
      full_fixed_window_size_, full_free_window_size_, term_criteria);
}

