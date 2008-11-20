#ifndef FEATURES_BASIC_MATH_H
#define FEATURES_BASIC_MATH_H

namespace features {

inline void add(int size, const float* src1, const float* src2, float* dst)
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
/*printf("a = "); for (int i=0; i<10; i++) printf(" %.2e ", a[i]); printf("\n");
printf("b = ");
for (int i=0; i<10; i++)
   printf(" %.2e ", b[i]);
printf("\n");*/
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
    result += *a - *b;
    ++a; ++b;
  }
  return result;
}

} // namespace features

#endif
