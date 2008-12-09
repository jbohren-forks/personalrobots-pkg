/*
 * PointTracks.cpp
 *
 *  Created on: Nov 5, 2008
 *      Author: jdchen
 */

#include <limits>

#include <PointTracks.h>
#include <boost/foreach.hpp>

namespace cv {
namespace willow {
PointTracks::~PointTracks(){
  /// @todo need to delete PointTrack's when it owns them
#if 0
  BOOST_FOREACH(PointTrack* pt, tracks_) {
    delete pt;
  }
#endif
}

void PointTracks::purge(int oldestFrameIndex) {
  if (tracks_.size()==0) {
    return;
  }
  int oldest_frame_index_in_tracks = oldestFrameIndex;
  for (list<PointTrack*>::iterator iTrack = tracks_.begin();
       iTrack != tracks_.end() && tracks_.size() != 0;
  ) {
    PointTrack* track = *iTrack;
    if (track->lastFrameIndex() < oldestFrameIndex) {
      //  remove the entire track, as it is totally outside of the window
#if DEBUG==1
      printf("erase track %d, len=%d\n", track->id_, track->size());
      track->print();
#endif
      iTrack = tracks_.erase(iTrack);
      delete track;
    } else {
      if (track->firstFrameIndex() < oldestFrameIndex){
        // part of this track is in fixed frames/cameras.

        // keep track of the oldest frame index in the existing tracks.
        if (track->firstFrameIndex() < oldest_frame_index_in_tracks) {
          oldest_frame_index_in_tracks = track->firstFrameIndex();
        }
      }
      iTrack++;
    }
  }
  oldest_frame_index_in_tracks_  = oldest_frame_index_in_tracks;
#if DEBUG==1
  printf("oldest_frame_index_in_tracks_ set to %d\n", oldest_frame_index_in_tracks_);
#endif
}

void PointTrack::print() const {
  printf("track %d (%7.4f, %7.4f, %7.4f) of size %2d: ", id_, coordinates_.x,
      coordinates_.y, this->coordinates_.z,  size());
  BOOST_FOREACH( const PointTrackObserv* obsv, *this) {
    printf("(%d, %d, [%5.1f,%5.1f,%5.1f]), ",
        obsv->frame_index_, obsv->keypoint_index_,
        obsv->disp_coord_.x, obsv->disp_coord_.y, obsv->disp_coord_.z);
  }
  printf("\n");
}

PointTrack::~PointTrack(){
  BOOST_FOREACH( PointTrackObserv* obsv, *this) {
    delete obsv;
  }
}

void PointTracks::print() const {
  // print the stats
  int numTracks, maxLen, minLen;
  double avgLen;
  vector<int> lenHisto;
  stats(&numTracks, &maxLen, &minLen, &avgLen, &lenHisto);
  printf("printing %d tracks", tracks_.size());
  printf("Stat of the tracks: [num, maxLen, minLen, avgLen]=[%d,%d,%d,%f]\n",
      numTracks, maxLen, minLen, avgLen);
  printf("Histogram of track lengths  [len, count]:\n");
  int len = 0;
  BOOST_FOREACH( const int count, lenHisto) {
    printf("%d  %d\n", len, count);
    len++;
  }

  BOOST_FOREACH( const PointTrack* track, tracks_) {
    track->print();
  }
}

void PointTracks::stats(int *numTracks, int *maxLen, int* minLen, double *avgLen,
    vector<int>* lenHisto) const {
  // note, we consider tracks that have at least two observations (detected in two frames)
  int nTracks = 0;
  int min = numeric_limits<int>::max();
  int max = 0;
  int sum = 0;
  BOOST_FOREACH( const PointTrack* track, tracks_ ) {
    int sz = track->size();
    if (sz>1) {
      if (min > sz ) min = sz;
      if (max < sz ) max = sz;
      sum += sz;
      nTracks++;
    }
  }

  if (numTracks) *numTracks = tracks_.size();
  if (maxLen) *maxLen = max;
  if (minLen) *minLen = min;
  if (avgLen) *avgLen = (double)sum/(double)nTracks;

  if (lenHisto) {
    lenHisto->resize(max+1);
    BOOST_FOREACH( const PointTrack* track, tracks_ ) {
      int& count = lenHisto->at(track->size());
      count++;
    }
  }
}

/// \brief Save the tracks to files.
/// One file each track.
/// first row is the track id followed by Cartesian coordinates
/// from second row and on is the observations in frames, each of
/// these rows are frame id, followed by u, v, d (disparity).
void PointTracks::save(string& dir) const  {
  BOOST_FOREACH( const PointTrack* p, tracks_ ) {
    CvMat* tracks_mat = cvCreateMat(p->size()+1, 4, CV_64FC1);
    // fill in the first row.
    cvmSet(tracks_mat, 0, 0, p->id_);
    cvmSet(tracks_mat, 0, 1, p->coordinates_.x);
    cvmSet(tracks_mat, 0, 2, p->coordinates_.y);
    cvmSet(tracks_mat, 0, 3, p->coordinates_.z);
    int row=1;
    BOOST_FOREACH( const PointTrackObserv* obsv, *p ){
      cvmSet(tracks_mat, row, 0, obsv->disp_coord_.x);
      cvmSet(tracks_mat, row, 1, obsv->disp_coord_.y);
      cvmSet(tracks_mat, row, 2, obsv->disp_coord_.z);
    }
    // save it to a file
    char point_track_index_str[16];
    char point_track_filename_str[32];
    sprintf(point_track_index_str, "track-%05d", p->id_);
    sprintf(point_track_filename_str, "/%s.xml", point_track_index_str);
    string file_path(dir);
    file_path.append(point_track_filename_str);
    cvSave(file_path.c_str(), tracks_mat, "PointTrack", point_track_index_str);
    cvReleaseMat(&tracks_mat);
  }
}

PointTracks* PointTracks::load(string& dir, int start, int end) {
  PointTracks* tracks;
  for (int i=start; i<=end; i++) {
    char point_track_index_str[16];
    char point_track_filename_str[32];
    sprintf(point_track_index_str, "track-%05d", i);
    sprintf(point_track_filename_str, "%s.xml", point_track_index_str);
    string file_path(dir);
    file_path.append(point_track_filename_str);
    CvMat* tracks_mat = (CvMat*)cvLoad(file_path.c_str());
    if (tracks_mat->rows<2) {
      // bad data
      printf("track data file %s is not valid: length less than 2\n", file_path.c_str());
    } else {
      // index of the point
      int ipt = cvmGet(tracks_mat, 0, 0);
      CvPoint3D64f coord;
      coord.x = cvmGet(tracks_mat, 0, 1);
      coord.y = cvmGet(tracks_mat, 0, 2);
      coord.z = cvmGet(tracks_mat, 0, 3);
      int frame_index0 = cvmGet(tracks_mat, 1, 0);
      CvPoint3D64f disp_coord0;
      disp_coord0.x = cvmGet(tracks_mat, 1, 1);
      disp_coord0.y = cvmGet(tracks_mat, 1, 2);
      disp_coord0.z = cvmGet(tracks_mat, 1, 3);
      int frame_index1 = cvmGet(tracks_mat, 2, 0);
      CvPoint3D64f disp_coord1;
      disp_coord0.x = cvmGet(tracks_mat, 2, 1);
      disp_coord0.y = cvmGet(tracks_mat, 2, 2);
      disp_coord0.z = cvmGet(tracks_mat, 2, 3);
      PointTrackObserv* obsv0 = new PointTrackObserv(frame_index0, disp_coord0, ipt);
      PointTrackObserv* obsv1 = new PointTrackObserv(frame_index1, disp_coord1, ipt);

      PointTrack* point =  new PointTrack(obsv0, obsv1, coord, ipt);
      for (int k=3; k<tracks_mat->rows; k++) {
        int frame_index = cvmGet(tracks_mat, k, 0);
        CvPoint3D64f disp_coord;
        disp_coord.x = cvmGet(tracks_mat, k, 1);
        disp_coord.y = cvmGet(tracks_mat, k, 2);
        disp_coord.z = cvmGet(tracks_mat, k, 3);
        PointTrackObserv* obsv = new PointTrackObserv(frame_index, disp_coord, ipt);
        point->extend(obsv);
      }
      tracks->tracks_.push_back(point);
    }
    cvReleaseMat(&tracks_mat);
  }
  return tracks;
}
}
}

