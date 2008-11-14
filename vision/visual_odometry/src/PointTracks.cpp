/*
 * PointTracks.cpp
 *
 *  Created on: Nov 5, 2008
 *      Author: jdchen
 */

#include <PointTracks.h>
#include <boost/foreach.hpp>


void PointTracks::purge(int oldestFrameIndex) {
  if (tracks_.size()==0) {
    return;
  }
  int oldest_frame_index_in_tracks = oldestFrameIndex;
  for (list<PointTrack>::iterator iTrack = tracks_.begin();
       iTrack != tracks_.end() && tracks_.size() != 0;
  ) {
    if (iTrack->lastFrameIndex() < oldestFrameIndex) {
      //  remove the entire track, as it is totally outside of the window
#if DEBUG==1
      printf("erase track %d, len=%d\n", iTrack->id_, iTrack->size());
      iTrack->print();
#endif
      iTrack = tracks_.erase(iTrack);
    } else {
      if (iTrack->firstFrameIndex() < oldestFrameIndex){
        // part of this track is in fixed frames/cameras.

        // keep track of the oldest frame index in the existing tracks.
        if (iTrack->firstFrameIndex() < oldest_frame_index_in_tracks) {
          oldest_frame_index_in_tracks = iTrack->firstFrameIndex();
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
  printf("track %d of size %d: ", this->id_, size());
  BOOST_FOREACH( const PointTrackObserv& obsv, *this) {
    printf("(%d, %d, [%5.1f,%5.1f,%5.1f]), ",
        obsv.frame_index_, obsv.keypoint_index_,
        obsv.disp_coord_.x, obsv.disp_coord_.y, obsv.disp_coord_.z);
  }
  printf("\n");
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

  BOOST_FOREACH( const PointTrack& track, tracks_) {
    track.print();
  }
}

void PointTracks::stats(int *numTracks, int *maxLen, int* minLen, double *avgLen,
    vector<int>* lenHisto) const {
  // note, we consider tracks that have at least two observations (detected in two frames)
  int nTracks = 0;
  int min = INT_MAX;
  int max = 0;
  int sum = 0;
  BOOST_FOREACH( const PointTrack& track, tracks_ ) {
    int sz = track.size();
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
    BOOST_FOREACH( const PointTrack& track, tracks_ ) {
      int& count = lenHisto->at(track.size());
      count++;
    }
  }
}


