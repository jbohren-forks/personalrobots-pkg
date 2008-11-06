/*
 * PointTracks.cpp
 *
 *  Created on: Nov 5, 2008
 *      Author: jdchen
 */

#include <PointTracks.h>
#include <boost/foreach.hpp>


void PointTracks::purge(int oldestFrameIndex) {
  for (deque<PointTrack>::iterator iTrack = mTracks.begin();
    iTrack != mTracks.end();
    iTrack++) {
    if (iTrack->lastFrameIndex() < oldestFrameIndex) {
      //  remove the entire track, as it is totally outside of the window
#if DEBUG==1
      printf("erase track %d, len=%d\n", iTrack->mId, iTrack->size());
#endif
      mTracks.erase(iTrack);
    } else if (iTrack->firstFrameIndex() < oldestFrameIndex){
#if 0 // Do not do this as those older frames are used as fixed frames
      // get rid of the entries that fall out of the slide window
      iTrack->purge(oldestFrameIndex);
      // remove the track if its length is reduced to 1 or less.
      if (iTrack->size() < 2) {
        mTracks.erase(iTrack);
      }
#endif
    }
  }
}

void PointTrack::print() const {
  printf("track of size %d: ", size());
  BOOST_FOREACH( const PointTrackObserv& obsv, *this) {
    printf("(%d, %d), ", obsv.mFrameIndex, obsv.mKeypointIndex);
  }
  printf("\n");
}

void PointTracks::print() const {
  // print the stats
  int numTracks, maxLen, minLen;
  double avgLen;
  vector<int> lenHisto;
  stats(&numTracks, &maxLen, &minLen, &avgLen, &lenHisto);
  printf("printing %d tracks", mTracks.size());
  printf("Stat of the tracks: [num, maxLen, minLen, avgLen]=[%d,%d,%d,%f]\n",
      numTracks, maxLen, minLen, avgLen);
  printf("Histogram of track lengths  [len, count]:\n");
  int len = 0;
  BOOST_FOREACH( const int count, lenHisto) {
    printf("%d  %d\n", len, count);
    len++;
  }

  BOOST_FOREACH( const PointTrack& track, mTracks) {
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
  BOOST_FOREACH( const PointTrack& track, mTracks ) {
    int sz = track.size();
    if (sz>1) {
      if (min > sz ) min = sz;
      if (max < sz ) max = sz;
      sum += sz;
      nTracks++;
    }
  }

  if (numTracks) *numTracks = mTracks.size();
  if (maxLen) *maxLen = max;
  if (minLen) *minLen = min;
  if (avgLen) *avgLen = (double)sum/(double)nTracks;

  if (lenHisto) {
    lenHisto->resize(max+1);
    BOOST_FOREACH( const PointTrack& track, mTracks ) {
      int& count = lenHisto->at(track.size());
      count++;
    }
  }
}


