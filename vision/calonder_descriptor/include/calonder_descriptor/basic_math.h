#ifndef FEATURES_BASIC_MATH_H
#define FEATURES_BASIC_MATH_H

#include <cstdlib>
#include <cmath>
#include <stdint.h>

namespace features {

inline void add(int size, const float* src1, const float* src2, float* dst)
{
  while(--size >= 0) {
    *dst = *src1 + *src2;
    ++dst; ++src1; ++src2;
  }
}

inline void add(int size, const uint16_t* src1, const uchar* src2, uint16_t* dst)
{
  while(--size >= 0) {
    *dst = *src1 + *src2;
    ++dst; ++src1; ++src2;
  }
}

// Squared Euclidean distance
inline float squaredDistance(int size, const float* a, const float* b)
{
  float result = 0;
  while (--size >= 0) {
    float diff = *a - *b;
    ++a; ++b;
    result += diff*diff;
  }
  return result;
}

// L1 distance
inline float L1Distance(int size, const float* a, const float* b)
{
  float result = 0;
  while (--size >= 0) {
    result += fabs(*a - *b);
    ++a; ++b;
  }
  return result;
}

inline int L1Distance(int size, const uint8_t* a, const uint8_t* b)
{
  int result = 0;
  while (--size >= 0) {
    result += abs(*a - *b);
    ++a; ++b;
  }
  return result;
}

inline float L2Distance(int size, const float* a, const float* b)
{
   return squaredDistance(size, a, b);
}

// infinity norm
inline float LInfDistance(int size, const float* a, const float* b)
{
  float result = 0.f, diff;
  while (--size >= 0) {
    diff = fabs(*a - *b);
    if (diff > result) result = diff;
    ++a; ++b;
  }
  return result;
}


} // namespace features

#endif
