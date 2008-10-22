import math

TARGET_C = 0
SCALE_RATIO = math.sqrt(2.0)
sumwidth = 1777

for n in range(3,13):
  m_filter_sizes = [0] * n
  m_filter_sizes[0] = 1;
  cur_size = 1;
  scale = 1;
  while (scale < n):
      cur_size *= SCALE_RATIO;
      rounded_size = int(cur_size + 0.5);
      if (rounded_size == m_filter_sizes[scale - 1]):
          continue;
      m_filter_sizes[scale] = rounded_size;
      scale += 1

  print "//", m_filter_sizes

  def StarPixels(radius, offset):
      upright_pixels = (2*radius + 1)*(2*radius + 1);
      tilt_pixels = offset*offset + (offset + 1)*(offset + 1);
      return upright_pixels + tilt_pixels;

  m_filter_sizes_2 = [2*i for i in m_filter_sizes]
  border = m_filter_sizes_2[-1] + m_filter_sizes_2[-1] / 2
  border = (border+3) & ~3

  def genRead(im, yo, xo):
      #return "CV_IMAGE_ELEM(%s, int, y + %d, x + %d)" % (im, yo, xo)
      if TARGET_C:
          return '%s_p[%d]' % (im, xo + yo * sumwidth)
      else:
          #return '_mm_cvtsi32_si128(%s_p[%d])' % (im, xo + yo * 641)
          offset = xo + yo * sumwidth
          return '(__m128i)_mm_loadu_ps((const float *)%s_p + %d)' % (im, offset)

  class point:
      def __init__(self, x, y):
          self.x = x
          self.y = y

  def abcd(a,b,c,d):
      if TARGET_C:
          return "%s-%s-%s+%s" % (a,b,c,d)
      else:
          # compute (a-b)+(d-c)
          return "_mm_add_epi32(_mm_sub_epi32(%s, %s), _mm_sub_epi32(%s, %s))" % (a, b, d, c)

  def UprightAreaSum(tl, br):
      return abcd(
          genRead('m_upright', br.y + 1, br.x + 1),
          genRead('m_upright', tl.y, br.x + 1),
          genRead('m_upright', br.y + 1, tl.x),
          genRead('m_upright', tl.y, tl.x))

  def TiltedAreaSum(offset):
      return abcd(
          genRead('m_tilted', offset + 1, 1),
          genRead('m_flat', 0, -offset),
          genRead('m_flat', 0, offset + 1),
          genRead('m_tilted', -offset, 1))

  uniqs = sorted(list(set(m_filter_sizes) | set(m_filter_sizes_2)))

  print "#define OR(x,y)     ((__m128i)_mm_or_ps((__m128)(x), (__m128)(y)))"
  print "#define AND(x,y)    ((__m128i)_mm_and_ps((__m128)(x), (__m128)(y)))"
  print "#define ANDNOT(x,y) ((__m128i)_mm_andnot_ps((__m128)(x), (__m128)(y)))"
  print "#define K(v)        _mm_set_epi32((v),(v),(v),(v))"
  print "#define ABS(x)      AND(x, K(0x7fffffff))"
  print "#define PRED(m,a,b) OR(AND((m),(a)), ANDNOT((m),(b)))"

  print "void StarDetector::FilterResponsesGen%d() {" % n

  if TARGET_C:
      print "int", ",".join(["w%d" % r for r in uniqs]), ";"
  else:
      print "__m128i", ",".join(["w%d" % r for r in uniqs]), ";"
  print "for (int y = %d; y < m_H - %d; ++y) { " % (border,border)

  print "float *p_prj = &CV_IMAGE_ELEM(m_projected, float, y, %d);" % border
  print "uchar *p_scl = &CV_IMAGE_ELEM(m_scales, uchar, y, %d);" % border

  # Source pointers
  for b in ['m_upright', 'm_tilted', 'm_flat']:
      print "int *%s_p = &CV_IMAGE_ELEM(%s, int, y, %d);" % (b, b, border)
  if TARGET_C:
      xstride = 1
  else:
      xstride = 4

  print "for (int x = %d; x < m_W - %d; x += %d) {" % (border,border, xstride)

  # Compute pixel counts in w* variables
  for r in uniqs:
      o = r + r / 2
      #print r, o, StarPixels(r, o)
      if TARGET_C:
          print "  w%d" % r + " = ((" + UprightAreaSum(point(-r,-r), point(r,r)) + ") + (" + TiltedAreaSum(o) + "));"
      else:
          print "  w%d" % r + " = _mm_add_epi32(%s, %s);" % (UprightAreaSum(point(-r,-r), point(r,r)), TiltedAreaSum(o))

  # Compute scaled differences in r*.  These are the responses.
  if TARGET_C:
      print "float", ", ".join(["r%d" % r for r in m_filter_sizes]), ";"
  else:
      print "__m128", ", ".join(["r%d" % r for r in m_filter_sizes]), ";"

  for (dst, (i, o)) in enumerate(zip(m_filter_sizes, m_filter_sizes_2)):
      inner_pix = StarPixels(i, i + i/2)
      outer_pix = StarPixels(o, o + o/2) - inner_pix
      print "// %d %d" % (inner_pix, outer_pix)
      if TARGET_C:
          print "  r%d = %s*%e - %s*%e;" % (i, "w%d" % i, 1.0 / inner_pix, "(w%d - w%d)" % (o, i), 1.0 / outer_pix)
      else:
          print "{"
          print "  __m128 a = _mm_mul_ps(_mm_cvtepi32_ps(w%d), _mm_set1_ps(1.0 / %d));" % (i, inner_pix)
          print "  __m128 b = _mm_mul_ps(_mm_cvtepi32_ps(_mm_sub_epi32(w%d, w%d)), _mm_set1_ps(1.0 / %d));" % (o, i, outer_pix)
          print "  r%d = (_mm_sub_ps(a, b));" % i
          print "}"

  rs = ["r%d" % r for r in m_filter_sizes]

  if TARGET_C:
      print "int mx_s = 0;"
      print "float mx = %s, mx_abs = std::abs(%s);" % (rs[0], rs[0])
      for i in range(1, len(rs)):
        r = rs[i]
        print "if (std::abs(%s) > mx_abs) { mx = %s; mx_s = %d; mx_abs = std::abs(%s); }" % (r, r, i, r)

      print "*p_prj = mx;"
      print "*p_scl = mx_s + 1;"

  else:

      print "__m128i mx_s, mx, is_win;"
      print "mx_s = K(1);"
      print "mx = (__m128i)%s;" % rs[0]
      for i,r in list(enumerate(rs))[1:]:
        # Both operands are floating-point, but because they're both +ve,
        # it's safe to use the faster integer compare.
        print "is_win = _mm_cmpgt_epi32(ABS(%s), ABS(mx));" % r
        print "mx_s = PRED(is_win, K(%d), mx_s);" % (i + 1)
        print "mx = PRED(is_win, %s, mx);" % (r)

      if 1:
        print "__m128i thresh = (__m128i)_mm_set1_ps(m_response_threshold);"
        print "is_win = _mm_cmpgt_epi32(ABS(mx), thresh);"
        print "mx_s = PRED(is_win, mx_s, K(1));"

      print "*(int*)p_scl = _mm_cvtsi128_si32(_mm_packs_epi16(_mm_packs_epi32(mx_s, mx_s), mx_s));"
      print "_mm_storeu_ps(p_prj, (__m128)mx);"

  print "p_prj += %d;" % xstride
  print "p_scl += %d;" % xstride

  for b in ['m_upright', 'm_tilted', 'm_flat']:
      print "%s_p += %d;" % (b, xstride)

  print "}}"
  print "}"
