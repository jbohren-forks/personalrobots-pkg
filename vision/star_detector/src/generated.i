// [1, 2, 3]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen3() {
__m128i w1,w2,w3,w4,w6 ;
for (int y = 12; y < m_H - 12; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 12);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 12);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 12);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 12);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 12);
for (int x = 12; x < m_W - 12; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
__m128 r1, r2, r3 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen4() {
__m128i w1,w2,w3,w4,w6,w8 ;
for (int y = 12; y < m_H - 12; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 12);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 12);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 12);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 12);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 12);
for (int x = 12; x < m_W - 12; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
__m128 r1, r2, r3, r4 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen5() {
__m128i w1,w2,w3,w4,w6,w8,w12 ;
for (int y = 20; y < m_H - 20; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 20);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 20);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 20);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 20);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 20);
for (int x = 20; x < m_W - 20; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
__m128 r1, r2, r3, r4, r6 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen6() {
__m128i w1,w2,w3,w4,w6,w8,w12,w16 ;
for (int y = 24; y < m_H - 24; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 24);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 24);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 24);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 24);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 24);
for (int x = 24; x < m_W - 24; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
__m128 r1, r2, r3, r4, r6, r8 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen7() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22 ;
for (int y = 36; y < m_H - 36; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 36);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 36);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 36);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 36);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 36);
for (int x = 36; x < m_W - 36; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
__m128 r1, r2, r3, r4, r6, r8, r11 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11, 16]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen8() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22,w32 ;
for (int y = 48; y < m_H - 48; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 48);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 48);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 48);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 48);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 48);
for (int x = 48; x < m_W - 48; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
  w32 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 58674), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -56831)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -56896), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 58609))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 87074), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -48)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -85295), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 49))));
__m128 r1, r2, r3, r4, r6, r8, r11, r16 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
// 2290 6640
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w16), _mm_set1_ps(1.0 / 2290));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w32, w16)), _mm_set1_ps(1.0 / 6640));
  r16 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
is_win = _mm_cmpgt_epi32(ABS(r16), ABS(mx));
mx_s = PRED(is_win, K(8), mx_s);
mx = PRED(is_win, r16, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11, 16, 23]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen9() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22,w23,w32,w46 ;
for (int y = 72; y < m_H - 72; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 72);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 72);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 72);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 72);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 72);
for (int x = 72; x < m_W - 72; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
  w23 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 42672), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -40847)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 42625))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 62196), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -34)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -60417), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 35))));
  w32 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 58674), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -56831)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -56896), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 58609))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 87074), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -48)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -85295), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 49))));
  w46 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 83566), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -81695)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 83473))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 124391), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -69)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -122612), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 70))));
__m128 r1, r2, r3, r4, r6, r8, r11, r16, r23 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
// 2290 6640
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w16), _mm_set1_ps(1.0 / 2290));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w32, w16)), _mm_set1_ps(1.0 / 6640));
  r16 = (_mm_sub_ps(a, b));
}
// 4590 13720
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w23), _mm_set1_ps(1.0 / 4590));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w46, w23)), _mm_set1_ps(1.0 / 13720));
  r23 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
is_win = _mm_cmpgt_epi32(ABS(r16), ABS(mx));
mx_s = PRED(is_win, K(8), mx_s);
mx = PRED(is_win, r16, mx);
is_win = _mm_cmpgt_epi32(ABS(r23), ABS(mx));
mx_s = PRED(is_win, K(9), mx_s);
mx = PRED(is_win, r23, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11, 16, 23, 32]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen10() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22,w23,w32,w46,w64 ;
for (int y = 96; y < m_H - 96; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 96);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 96);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 96);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 96);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 96);
for (int x = 96; x < m_W - 96; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
  w23 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 42672), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -40847)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 42625))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 62196), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -34)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -60417), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 35))));
  w32 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 58674), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -56831)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -56896), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 58609))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 87074), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -48)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -85295), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 49))));
  w46 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 83566), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -81695)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 83473))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 124391), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -69)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -122612), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 70))));
  w64 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 115570), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -113663)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -113792), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 115441))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 172370), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -96)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -170591), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 97))));
__m128 r1, r2, r3, r4, r6, r8, r11, r16, r23, r32 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
// 2290 6640
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w16), _mm_set1_ps(1.0 / 2290));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w32, w16)), _mm_set1_ps(1.0 / 6640));
  r16 = (_mm_sub_ps(a, b));
}
// 4590 13720
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w23), _mm_set1_ps(1.0 / 4590));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w46, w23)), _mm_set1_ps(1.0 / 13720));
  r23 = (_mm_sub_ps(a, b));
}
// 8930 26336
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w32), _mm_set1_ps(1.0 / 8930));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w64, w32)), _mm_set1_ps(1.0 / 26336));
  r32 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
is_win = _mm_cmpgt_epi32(ABS(r16), ABS(mx));
mx_s = PRED(is_win, K(8), mx_s);
mx = PRED(is_win, r16, mx);
is_win = _mm_cmpgt_epi32(ABS(r23), ABS(mx));
mx_s = PRED(is_win, K(9), mx_s);
mx = PRED(is_win, r23, mx);
is_win = _mm_cmpgt_epi32(ABS(r32), ABS(mx));
mx_s = PRED(is_win, K(10), mx_s);
mx = PRED(is_win, r32, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11, 16, 23, 32, 45]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen11() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22,w23,w32,w45,w46,w64,w90 ;
for (int y = 136; y < m_H - 136; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 136);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 136);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 136);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 136);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 136);
for (int x = 136; x < m_W - 136; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
  w23 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 42672), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -40847)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 42625))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 62196), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -34)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -60417), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 35))));
  w32 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 58674), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -56831)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -56896), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 58609))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 87074), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -48)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -85295), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 49))));
  w45 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -79919)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -80010), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 81697))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 120837), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -67)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -119058), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 68))));
  w46 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 83566), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -81695)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 83473))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 124391), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -69)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -122612), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 70))));
  w64 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 115570), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -113663)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -113792), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 115441))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 172370), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -96)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -170591), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 97))));
  w90 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 161798), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -159839)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -160020), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 161617))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 241673), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -135)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -239894), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 136))));
__m128 r1, r2, r3, r4, r6, r8, r11, r16, r23, r32, r45 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
// 2290 6640
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w16), _mm_set1_ps(1.0 / 2290));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w32, w16)), _mm_set1_ps(1.0 / 6640));
  r16 = (_mm_sub_ps(a, b));
}
// 4590 13720
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w23), _mm_set1_ps(1.0 / 4590));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w46, w23)), _mm_set1_ps(1.0 / 13720));
  r23 = (_mm_sub_ps(a, b));
}
// 8930 26336
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w32), _mm_set1_ps(1.0 / 8930));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w64, w32)), _mm_set1_ps(1.0 / 26336));
  r32 = (_mm_sub_ps(a, b));
}
// 17394 52088
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w45), _mm_set1_ps(1.0 / 17394));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w90, w45)), _mm_set1_ps(1.0 / 52088));
  r45 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
is_win = _mm_cmpgt_epi32(ABS(r16), ABS(mx));
mx_s = PRED(is_win, K(8), mx_s);
mx = PRED(is_win, r16, mx);
is_win = _mm_cmpgt_epi32(ABS(r23), ABS(mx));
mx_s = PRED(is_win, K(9), mx_s);
mx = PRED(is_win, r23, mx);
is_win = _mm_cmpgt_epi32(ABS(r32), ABS(mx));
mx_s = PRED(is_win, K(10), mx_s);
mx = PRED(is_win, r32, mx);
is_win = _mm_cmpgt_epi32(ABS(r45), ABS(mx));
mx_s = PRED(is_win, K(11), mx_s);
mx = PRED(is_win, r45, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
// [1, 2, 3, 4, 6, 8, 11, 16, 23, 32, 45, 64]
#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))
#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))
#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))
#define K(v)        _mm_set_epi32((v),(v),(v),(v))
#define ABS(x)      AND(x, K(0x7fffffff))
#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))
void StarDetector::FilterResponsesGen12() {
__m128i w1,w2,w3,w4,w6,w8,w11,w12,w16,w22,w23,w32,w45,w46,w64,w90,w128 ;
for (int y = 192; y < m_H - 192; ++y) { 
float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, 192);
uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, 192);
int *m_upright_p = &CV_IMAGE_ELEM(m_upright, int, y, 192);
int *m_tilted_p = &CV_IMAGE_ELEM(m_tilted, int, y, 192);
int *m_flat_p = &CV_IMAGE_ELEM(m_flat, int, y, 192);
for (int x = 192; x < m_W - 192; x += 4) {
  w1 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -1775)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -1778), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 3553))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 3555), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -1)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -1776), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 2))));
  w2 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -3551)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -3556), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 5329))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 7109), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -3)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -5330), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 4))));
  w3 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -5327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -5334), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 7105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 8886), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -4)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -7107), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 5))));
  w4 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 8890), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -7103)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -7112), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 8881))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 12440), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -6)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -10661), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 7))));
  w6 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 12446), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -10655)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -10668), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 12433))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 17771), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -9)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -15992), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 10))));
  w8 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 16002), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -14207)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -14224), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 15985))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 23102), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -12)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -21323), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 13))));
  w11 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -19535)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -19558), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 21313))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 30210), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -16)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -28431), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 17))));
  w12 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 23114), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -21311)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -21336), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 23089))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 33764), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -18)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -31985), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 19))));
  w16 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 30226), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -28415)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -28448), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 30193))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 44426), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -24)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -42647), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 25))));
  w22 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -39071)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -39116), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 40849))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 60419), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -33)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -58640), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 34))));
  w23 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 42672), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -40847)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -40894), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 42625))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 62196), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -34)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -60417), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 35))));
  w32 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 58674), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -56831)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -56896), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 58609))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 87074), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -48)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -85295), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 49))));
  w45 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -79919)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -80010), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 81697))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 120837), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -67)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -119058), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 68))));
  w46 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 83566), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -81695)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -81788), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 83473))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 124391), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -69)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -122612), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 70))));
  w64 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 115570), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -113663)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -113792), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 115441))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 172370), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -96)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -170591), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 97))));
  w90 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 161798), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -159839)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -160020), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 161617))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 241673), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -135)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -239894), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 136))));
  w128 = _mm_add_epi32(_mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + 229362), (__m128i)_mm_loadu_ps((const float *)m_upright_p + -227327)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_upright_p + -227584), (__m128i)_mm_loadu_ps((const float *)m_upright_p + 229105))), _mm_add_epi32(_mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + 342962), (__m128i)_mm_loadu_ps((const float *)m_flat_p + -192)), _mm_sub_epi32((__m128i)_mm_loadu_ps((const float *)m_tilted_p + -341183), (__m128i)_mm_loadu_ps((const float *)m_flat_p + 193))));
__m128 r1, r2, r3, r4, r6, r8, r11, r16, r23, r32, r45, r64 ;
// 14 36
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w1), _mm_set1_ps(1.0 / 14));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w2, w1)), _mm_set1_ps(1.0 / 36));
  r1 = (_mm_sub_ps(a, b));
}
// 50 116
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w2), _mm_set1_ps(1.0 / 50));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w4, w2)), _mm_set1_ps(1.0 / 116));
  r2 = (_mm_sub_ps(a, b));
}
// 90 260
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w3), _mm_set1_ps(1.0 / 90));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w6, w3)), _mm_set1_ps(1.0 / 260));
  r3 = (_mm_sub_ps(a, b));
}
// 166 436
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w4), _mm_set1_ps(1.0 / 166));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w8, w4)), _mm_set1_ps(1.0 / 436));
  r4 = (_mm_sub_ps(a, b));
}
// 350 960
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w6), _mm_set1_ps(1.0 / 350));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w12, w6)), _mm_set1_ps(1.0 / 960));
  r6 = (_mm_sub_ps(a, b));
}
// 602 1688
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w8), _mm_set1_ps(1.0 / 602));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w16, w8)), _mm_set1_ps(1.0 / 1688));
  r8 = (_mm_sub_ps(a, b));
}
// 1074 3196
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w11), _mm_set1_ps(1.0 / 1074));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w22, w11)), _mm_set1_ps(1.0 / 3196));
  r11 = (_mm_sub_ps(a, b));
}
// 2290 6640
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w16), _mm_set1_ps(1.0 / 2290));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w32, w16)), _mm_set1_ps(1.0 / 6640));
  r16 = (_mm_sub_ps(a, b));
}
// 4590 13720
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w23), _mm_set1_ps(1.0 / 4590));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w46, w23)), _mm_set1_ps(1.0 / 13720));
  r23 = (_mm_sub_ps(a, b));
}
// 8930 26336
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w32), _mm_set1_ps(1.0 / 8930));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w64, w32)), _mm_set1_ps(1.0 / 26336));
  r32 = (_mm_sub_ps(a, b));
}
// 17394 52088
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w45), _mm_set1_ps(1.0 / 17394));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w90, w45)), _mm_set1_ps(1.0 / 52088));
  r45 = (_mm_sub_ps(a, b));
}
// 35266 104896
{
  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w64), _mm_set1_ps(1.0 / 35266));
  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w128, w64)), _mm_set1_ps(1.0 / 104896));
  r64 = (_mm_sub_ps(a, b));
}
__m128i mx_s, mx, is_win;
mx_s = K(1);
mx = (__m128i)r1;
is_win = _mm_cmpgt_epi32(ABS(r2), ABS(mx));
mx_s = PRED(is_win, K(2), mx_s);
mx = PRED(is_win, r2, mx);
is_win = _mm_cmpgt_epi32(ABS(r3), ABS(mx));
mx_s = PRED(is_win, K(3), mx_s);
mx = PRED(is_win, r3, mx);
is_win = _mm_cmpgt_epi32(ABS(r4), ABS(mx));
mx_s = PRED(is_win, K(4), mx_s);
mx = PRED(is_win, r4, mx);
is_win = _mm_cmpgt_epi32(ABS(r6), ABS(mx));
mx_s = PRED(is_win, K(5), mx_s);
mx = PRED(is_win, r6, mx);
is_win = _mm_cmpgt_epi32(ABS(r8), ABS(mx));
mx_s = PRED(is_win, K(6), mx_s);
mx = PRED(is_win, r8, mx);
is_win = _mm_cmpgt_epi32(ABS(r11), ABS(mx));
mx_s = PRED(is_win, K(7), mx_s);
mx = PRED(is_win, r11, mx);
is_win = _mm_cmpgt_epi32(ABS(r16), ABS(mx));
mx_s = PRED(is_win, K(8), mx_s);
mx = PRED(is_win, r16, mx);
is_win = _mm_cmpgt_epi32(ABS(r23), ABS(mx));
mx_s = PRED(is_win, K(9), mx_s);
mx = PRED(is_win, r23, mx);
is_win = _mm_cmpgt_epi32(ABS(r32), ABS(mx));
mx_s = PRED(is_win, K(10), mx_s);
mx = PRED(is_win, r32, mx);
is_win = _mm_cmpgt_epi32(ABS(r45), ABS(mx));
mx_s = PRED(is_win, K(11), mx_s);
mx = PRED(is_win, r45, mx);
is_win = _mm_cmpgt_epi32(ABS(r64), ABS(mx));
mx_s = PRED(is_win, K(12), mx_s);
mx = PRED(is_win, r64, mx);
__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);
is_win = _mm_cmpgt_epi32(ABS(mx), thresh);
mx_s = PRED(is_win, mx_s, K(1));
*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));
_mm_storeu_ps(p_prj, (__m128)mx);
p_prj += 4;
p_scl += 4;
m_upright_p += 4;
m_tilted_p += 4;
m_flat_p += 4;
}}
}
