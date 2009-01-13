#ifndef FEATURES_BASIC_MATH_H
#define FEATURES_BASIC_MATH_H

#include <cstdlib>
#include <cmath>
#include <stdint.h>
#include <emmintrin.h>
#include <cstdio> // TODO: remove, debug only

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
  acc = _mm_sad_epu8(acc1[0],acc2[0]);
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[1], acc2[1]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[2], acc2[2]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[3], acc2[3]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[4], acc2[4]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[5], acc2[5]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[6], acc2[6]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[7], acc2[7]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[8], acc2[8]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[9], acc2[9]));
  acc = _mm_add_epi16(acc, _mm_sad_epu8(acc1[10], acc2[10]));

  acc = _mm_add_epi16(acc, _mm_srli_si128(acc, 8)); // add both halves
  return _mm_cvtsi128_si32(acc);
#else
  #error "just to let you know: using unoptimized L1 distance! now uncomment this line"
  return L1Distance(176, s1, s2);
#endif
}

// sum up 50 byte vectors of length 176
// assume 4 bits max for input vector values
// final shift is 2 bits right
// temp buffer should be twice as long as signature
// sig and buffer need not be initialized
inline void sum_50t_176c(uint8_t **pp, uint8_t *sig, uint16_t *temp)
{
#ifdef __SSE2__
  __m128i acc, *acc1, *acc2, *acc3, *acc4, tzero;
  __m128i *ssig, *ttemp;
  
  ssig = (__m128i *)sig;
  ttemp = (__m128i *)temp;

  // empty ttemp[]
  tzero = _mm_xor_si128(tzero,tzero);
  for (int i=0; i<22; i++)
    ttemp[i] = tzero;

  for (int j=0; j<48; j+=16)
    {
      // empty ssig[]
      tzero = _mm_xor_si128(tzero,tzero);
      for (int i=0; i<11; i++)
	ssig[i] = tzero;

      for (int i=j; i<j+16; i+=4) // 4 columns at a time, to 16
	{
	  acc1 = (__m128i *)pp[i];
	  acc2 = (__m128i *)pp[i+1];
	  acc3 = (__m128i *)pp[i+2];
	  acc4 = (__m128i *)pp[i+3];

	  // add next four columns
	  acc = _mm_adds_epu8(acc1[0],acc2[0]);
	  acc = _mm_adds_epu8(acc,acc3[0]);
	  acc = _mm_adds_epu8(acc,acc4[1]);
	  ssig[0] = _mm_adds_epu8(acc,ssig[0]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[1],acc2[1]);
	  acc = _mm_adds_epu8(acc,acc3[1]);
	  acc = _mm_adds_epu8(acc,acc4[1]);
	  ssig[1] = _mm_adds_epu8(acc,ssig[1]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[2],acc2[2]);
	  acc = _mm_adds_epu8(acc,acc3[2]);
	  acc = _mm_adds_epu8(acc,acc4[2]);
	  ssig[2] = _mm_adds_epu8(acc,ssig[2]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[3],acc2[3]);
	  acc = _mm_adds_epu8(acc,acc3[3]);
	  acc = _mm_adds_epu8(acc,acc4[3]);
	  ssig[3] = _mm_adds_epu8(acc,ssig[3]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[4],acc2[4]);
	  acc = _mm_adds_epu8(acc,acc3[4]);
	  acc = _mm_adds_epu8(acc,acc4[4]);
	  ssig[4] = _mm_adds_epu8(acc,ssig[4]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[5],acc2[5]);
	  acc = _mm_adds_epu8(acc,acc3[5]);
	  acc = _mm_adds_epu8(acc,acc4[5]);
	  ssig[5] = _mm_adds_epu8(acc,ssig[5]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[6],acc2[6]);
	  acc = _mm_adds_epu8(acc,acc3[6]);
	  acc = _mm_adds_epu8(acc,acc4[6]);
	  ssig[6] = _mm_adds_epu8(acc,ssig[6]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[7],acc2[7]);
	  acc = _mm_adds_epu8(acc,acc3[7]);
	  acc = _mm_adds_epu8(acc,acc4[7]);
	  ssig[7] = _mm_adds_epu8(acc,ssig[7]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[8],acc2[8]);
	  acc = _mm_adds_epu8(acc,acc3[8]);
	  acc = _mm_adds_epu8(acc,acc4[8]);
	  ssig[8] = _mm_adds_epu8(acc,ssig[8]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[9],acc2[9]);
	  acc = _mm_adds_epu8(acc,acc3[9]);
	  acc = _mm_adds_epu8(acc,acc4[9]);
	  ssig[9] = _mm_adds_epu8(acc,ssig[9]);
	  // add four columns
	  acc = _mm_adds_epu8(acc1[10],acc2[10]);
	  acc = _mm_adds_epu8(acc,acc3[10]);
	  acc = _mm_adds_epu8(acc,acc4[10]);
	  ssig[10] = _mm_adds_epu8(acc,ssig[10]);
	}

      // unpack to ttemp buffer and add
      tzero = _mm_xor_si128(tzero,tzero);
      ttemp[0] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[0],tzero),ttemp[0]);
      ttemp[1] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[0],tzero),ttemp[1]);
      ttemp[2] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[1],tzero),ttemp[2]);
      ttemp[3] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[1],tzero),ttemp[3]);
      ttemp[4] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[2],tzero),ttemp[4]);
      ttemp[5] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[2],tzero),ttemp[5]);
      ttemp[6] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[3],tzero),ttemp[6]);
      ttemp[7] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[3],tzero),ttemp[7]);
      ttemp[8] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[4],tzero),ttemp[8]);
      ttemp[9] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[4],tzero),ttemp[9]);
      ttemp[10] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[5],tzero),ttemp[10]);
      ttemp[11] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[5],tzero),ttemp[11]);
      ttemp[12] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[6],tzero),ttemp[12]);
      ttemp[13] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[6],tzero),ttemp[13]);
      ttemp[14] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[7],tzero),ttemp[14]);
      ttemp[15] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[7],tzero),ttemp[15]);
      ttemp[16] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[8],tzero),ttemp[16]);
      ttemp[17] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[8],tzero),ttemp[17]);
      ttemp[18] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[9],tzero),ttemp[18]);
      ttemp[19] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[9],tzero),ttemp[19]);
      ttemp[20] = _mm_add_epi16(_mm_unpacklo_epi8(ssig[10],tzero),ttemp[20]);
      ttemp[21] = _mm_add_epi16(_mm_unpackhi_epi8(ssig[10],tzero),ttemp[21]);
    }

  // create ssignature from 16-bit result
  ssig[0] =_mm_packus_epi16(_mm_srai_epi16(ttemp[0],2),_mm_srai_epi16(ttemp[1],2));
  ssig[1] =_mm_packus_epi16(_mm_srai_epi16(ttemp[2],2),_mm_srai_epi16(ttemp[3],2));
  ssig[2] =_mm_packus_epi16(_mm_srai_epi16(ttemp[4],2),_mm_srai_epi16(ttemp[5],2));
  ssig[3] =_mm_packus_epi16(_mm_srai_epi16(ttemp[6],2),_mm_srai_epi16(ttemp[7],2));
  ssig[4] =_mm_packus_epi16(_mm_srai_epi16(ttemp[8],2),_mm_srai_epi16(ttemp[9],2));
  ssig[5] =_mm_packus_epi16(_mm_srai_epi16(ttemp[10],2),_mm_srai_epi16(ttemp[11],2));
  ssig[6] =_mm_packus_epi16(_mm_srai_epi16(ttemp[12],2),_mm_srai_epi16(ttemp[13],2));
  ssig[7] =_mm_packus_epi16(_mm_srai_epi16(ttemp[14],2),_mm_srai_epi16(ttemp[15],2));
  ssig[8] =_mm_packus_epi16(_mm_srai_epi16(ttemp[16],2),_mm_srai_epi16(ttemp[17],2));
  ssig[9] =_mm_packus_epi16(_mm_srai_epi16(ttemp[18],2),_mm_srai_epi16(ttemp[19],2));
  ssig[10] =_mm_packus_epi16(_mm_srai_epi16(ttemp[20],2),_mm_srai_epi16(ttemp[21],2));
#else
  #error "just to let you know: using unoptimized byte additions for signature! now uncomment this line"
#  return L1Distance(176, s1, s2);
#endif
}


// L1 distance functor specialized for float or uint8_t
template< typename Elem >
struct L1DistanceFunc {};

template<> struct L1DistanceFunc<float>
{
  int dim;
  L1DistanceFunc(int dimension) : dim(dimension) {}
  inline float operator()(const float* a, const float* b) const {
    return L1Distance(dim, a, b);
  }
};

// TODO: currently assumes dimension == 176
template<> struct L1DistanceFunc<uint8_t>
{
  L1DistanceFunc(int = 0) {}
  inline int operator()(const uint8_t* a, const uint8_t* b) const {
    return L1Distance_176(a, b);
  }
};

inline float L2Distance(int size, const float* a, const float* b)
{
   return squaredDistance(size, a, b);
}

inline int L2Distance(int size, const uint8_t* a, const uint8_t* b)
{
  int result = 0;
  while (--size >= 0) {
    int diff = *a - *b;
    ++a; ++b;
    result += diff*diff;
  }
  return result;
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
