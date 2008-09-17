/*
 * CvRandomTripletSetGenerator.cpp
 *
 *  Created on: Sep 15, 2008
 *      Author: jdchen
 */

#include <iostream>
#include <exception>
using namespace std;

#include "CvRandomTripletSetGenerator.h"

CvRandomTripletSetGenerator::CvRandomTripletSetGenerator(
    const int64 seed,
    const int lower,
    const int upper,
    const bool noDuplicate):
      mRandomNumberGenerator(cvRNG(seed)),
      mLower(lower),
      mRange(upper-lower+1),
      mNoDuplicate(noDuplicate),
      mMaxNumSets((mRange)*(mRange-1)*(mRange-2)/6)
{
      if (lower>=upper) {
        std::cerr << __PRETTY_FUNCTION__ << "invalid parameters: lower no less than upper"<<std::endl;
        throw std::exception();
      }
      if (noDuplicate == true && mRange > (int)std::pow(2.f,21)) {
        std::cerr << __PRETTY_FUNCTION__ << "invalid parameters: when noDuplicate is true, range shall be less than 2^21"<<std::endl;
        throw std::exception();
      }
}

CvRandomTripletSetGenerator::~CvRandomTripletSetGenerator() {
}

bool operator == (CvTripletSet const& a, CvTripletSet const& b) {
  return a == b;
}

CvTripletSet CvRandomTripletSetGenerator::newSetWithDuplicate(){
  int a = cvRandInt(&mRandomNumberGenerator) % mRange;
  int b, c;
  do { b = cvRandInt(&mRandomNumberGenerator) % mRange; } while ( b == a);
  do { c = cvRandInt(&mRandomNumberGenerator) % mRange; } while ( c == a || c == b);
  return CvTripletSet(a+mLower, b+mLower, c+mLower);
}
void CvRandomTripletSetGenerator::_sort(CvTripletSet& s) {
  int a = s[0];
  int b = s[1];
  int c = s[2];

  // the following shall be faster than calling standard sort routine
  if (a<b) {
    if (b<c) {
      // a < b < c
      // great already sorted
    } else if (a < c) {
      // a < c < b
      // swap b and c
      s[1] = c;
      s[2] = b;
    } else {
      // c < a < b
      // shift a, b, c to right
      s[0] = c;
      s[1] = a;
      s[2] = b;
    }
  } else { // b < a
    if (c<b) {
      // c < b < a
      // swap a and c
      s[0] = c;
      s[2] = a;
    } else if (c<a) {
      // b < c < a
      // shift a, b, c to left
      s[0] = b;
      s[1] = c;
      s[2] = a;
    } else {
      // b < a < c
      // swap b and a
      s[0] = b;
      s[1] = a;
    }
  }
}

CvTripletSet CvRandomTripletSetGenerator::newSet()
{

  if (mNoDuplicate == false) {
    return newSetWithDuplicate();
  } else {
    for (CvTripletSet triplet = newSetWithDuplicate();
      mSetOfSets.size() < mMaxNumSets;
      triplet = newSetWithDuplicate()) {
      // sort the triplet
      _sort(triplet);
      boost::long_long_type encoded = mEncodeSetInLong(triplet);
      if (mSetOfSets.find(encoded) == mSetOfSets.end() ) {
        // new set that we have not seen before
        mSetOfSets.insert(encoded);
        return triplet;
      }
    }
    cerr << "reach max num: "<<mMaxNumSets<<endl;
    throw std::exception();
  }
}

uint64 CvRandomTripletSetGenerator::mEncodeSetInLong(CvTripletSet const & _triplet){
  CvTripletSet triplet = _triplet;
  uint64 t0 = triplet[0] - mLower;
  uint64 t1 = triplet[1] - mLower;
  uint64 t2 = triplet[2] - mLower;
  uint64 encoded = ((t0 << 21)+t1) << 21 + t2;

#if 0
  boost::long_long_type encoded =  triplet[0] - mLower;
  encoded = encoded*mRange + (triplet[1] - mLower);
  encoded = encoded*mRange + (triplet[2] - mLower);
#endif
  return encoded;
}


