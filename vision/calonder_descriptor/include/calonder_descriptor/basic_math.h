#ifndef FEATURES_BASIC_MATH_H
#define FEATURES_BASIC_MATH_H

#include <cstdlib>
#include <cmath>
#include <stdint.h>
#include <emmintrin.h>

namespace features {

// Metaprogram to determine accumulation data type (float->float, uint8_t->int)
template< typename T >
struct Promote {};
template<> struct Promote<float> { typedef float type; };
template<> struct Promote<uint8_t> { typedef int type; };

inline void add(int size, const float* src1, const float* src2, float* dst)
{
  while(--size >= 0) {
    *dst = *src1 + *src2;
    ++dst; ++src1; ++src2;
  }
}

inline void add(int size, const uint16_t* src1, const uint8_t* src2, uint16_t* dst)
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

inline int L1Distance_176(const uint8_t *s1, const uint8_t *s2)
{
#ifdef __SSE2__
  __m128i acc, *acc1, *acc2;
  acc1 = (__m128i *)s1;
  acc2 = (__m128i *)s2;
  
  // abs diff of columns
  
  acc = _mm_sad_epu8(*acc1,*acc2);
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+1),*(acc2+1)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+2),*(acc2+2)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+3),*(acc2+3)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+4),*(acc2+4)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+5),*(acc2+5)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+6),*(acc2+6)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+7),*(acc2+7)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+8),*(acc2+8)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+9),*(acc2+9)));
  acc = _mm_add_epi16(acc,_mm_sad_epu8(*(acc1+10),*(acc2+10)));

  acc = _mm_add_epi16(acc,_mm_srli_si128(acc,4)); // add both halves
  return _mm_cvtsi128_si32(acc);
#else
  #error "just to let you know: using unoptimized L1 distance! now uncomment this line"
  return L1Distance(176, s1, s2);
#endif
}

// L1 distance functor specialized for float or uint8_t
template< typename Elem >
struct L1DistanceFunc {};

template<> struct L1DistanceFunc<float>
{
  int dim;
  L1DistanceFunc(int dimension) : dim(dimension) {}
  float operator()(const float* a, const float* b) const {
    return L1Distance(dim, a, b);
  }
};

// TODO: currently assumes dimension == 176
template<> struct L1DistanceFunc<uint8_t>
{
  L1DistanceFunc(int = 0) {}
  int operator()(const uint8_t* a, const uint8_t* b) const {
    return L1Distance_176(a, b);
  }
};

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
