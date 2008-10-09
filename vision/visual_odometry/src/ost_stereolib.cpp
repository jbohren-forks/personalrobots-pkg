//
// C version of stereo algorithm
// Basic column-oriented library
//
// Copyright 2008 by Kurt Konolige
// Videre Design
// kurt@videredesign.com
//

// This file is copied over from stereo/ost. All the functions are
// therefore being put into a namespace of ost
// We may want to consolidate the multiple similar version of this header file
// jdchen

#include "ost_stereolib.h"

#include <iostream>

#define inline			// use this for Intel Compiler Debug mode

// algorithm requires larger buffers to be passed in
// using lib fns like "memset" can cause problems (?? not sure -
//   it was a stack alignment problem)
// -- but still roll our own

static inline void
memclr(void *buf, int n)
{
  int i;
  unsigned char *bb = (unsigned char *)buf;
  for (i=0; i<n; i++)
    *bb++ = 0;
}


//
// simple normalization prefilter
// XKERN x YKERN mean intensity
// normalize center pixels
// feature values are unsigned bytes, offset at <ftzero>
//
// image aligned at 16 bytes
// feature output aligned at 16 bytes
//

void
ost_do_prefilter_norm(
    const uint8_t *im,  // input image
    uint8_t *ftim,  // feature image output
    int xim, int yim, // size of image
    uint8_t ftzero, // feature offset from zero
    uint8_t *buf    // buffer storage
    )
{
  int i,j;
  const uint8_t *imp, *impp;
  uint16_t acc;
  const uint8_t *topp, *toppp, *botp, *botpp, *cenp;
  int32_t qval;
  uint32_t uqval;
  uint8_t *ftimp;

  // set up buffers
  uint16_t *accbuf, *accp;
  int ACCBUFSIZE = xim+64;
  accbuf = (uint16_t *)buf;

  // image ptrs
  imp = im;     // leading
  impp = im;      // lagging

  // offset output ptr
  ftim += xim*(YKERN/2+1) + XKERN/2+1;

  // clear acc buffer
  memclr(accbuf, ACCBUFSIZE*sizeof(int16_t));

  // loop over rows
  for (j=0; j<yim; j++, imp+=xim, ftim+=xim)
    {
      acc = 0;      // accumulator
      accp = accbuf;    // start at beginning of buf
      topp  = imp;    // leading integration ptr
      toppp = imp;    // lagging integration ptr
      botp  = impp;   // leading integration ptr
      botpp = impp;   // lagging integration ptr

      if (j<YKERN)    // initial row accumulation
  {
    for (i=0; i<XKERN; i++) // initial col accumulation
      acc += *topp++;
    for (; i<xim; i++) // incremental col acc
      {
        acc += *topp++ - *toppp++; // do line sum increment
        *accp++ += acc; // increment acc buf value
      }
  }
      else      // incremental accumulation
  {
    cenp = imp - (YKERN/2)*xim + XKERN/2 + 1; // center pixel
    ftimp = ftim - (YKERN/2)*xim + XKERN/2 + 1; // output ptr
    for (i=0; i<XKERN; i++) // initial col accumulation
      acc += *topp++ - *botp++;
    for (; i<xim; i++, accp++, cenp++) // incremental col acc
      {
        acc += *topp++ - *toppp++; // do line sum increment
        acc -= *botp++ - *botpp++; // subtract out previous vals
        *accp += acc; // increment acc buf value

        // now calculate diff from mean value and save
        qval = 4*(*cenp) + *(cenp-1) + *(cenp+1) + *(cenp-xim) + *(cenp+xim);
#if (XKERN==9)
        qval = (qval*10) - *accp; // difference with mean, norm is 81
        qval = qval>>4; // divide by 16 (cenp val divide by ~2)
#endif
#if (XKERN==7)
        qval = (qval*6) - *accp;  // difference with mean, norm is 49
        qval = qval>>3; // divide by 8 (cenp val divide by ~2)
#endif
        if (qval < -ftzero)
    uqval = 0;
        else if (qval > ftzero)
    uqval = ftzero+ftzero;
        else
    uqval = qval + ftzero;
        *ftimp++ = (uint8_t)uqval;
      }
    impp += xim;    // increment lagging image ptr
  }
    }
}


#ifdef __SSE2__
// algorithm requires larger buffers to be passed in
// using lib fns like "memset" can cause problems (?? not sure -
//   it was a stack alignment problem)
// -- but still roll our own

static inline void
memclr_si128(__m128i *buf, int n)
{
  int i;
  __m128i zz;
  zz = _mm_setzero_si128();
  n = n>>4;			// divide by 16
  for (i=0; i<n; i++, buf++)
    _mm_store_si128(buf,zz);
}

//
// fast SSE2 version
// NOTE: output buffer <ftim> must be aligned on 16-byte boundary
// NOTE: input image <im> must be aligned on 16-byte boundary
// NOTE: KSIZE (XKERN, YKERN) is fixed at 7
//

#define PXKERN 7
#define PYKERN 7

void
ost_do_prefilter_fast(
    const uint8_t *im,  // input image
    uint8_t *ftim,  // feature image output
    int xim, int yim, // size of image
    uint8_t ftzero, // feature offset from zero
    uint8_t *buf    // buffer storage
    )
{
  int i,j;

  // set up buffers, first align to 16 bytes
  uintptr_t bufp = (uintptr_t)buf;
  int16_t *accbuf, *accp;
  int16_t *intbuf, *intp; // intbuf is size xim
  const uint8_t *imp, *impp, *ftimp;
  __m128i acc, accs, acc1, acct, pxs, opxs, zeros;
  __m128i const_ftzero, const_ftzero_x48, const_ftzero_x2;

  int FACCBUFSIZE = xim+64;

  if (bufp & 0xF)
    bufp = (bufp+15) & ~(uintptr_t)0xF;
  buf = (uint8_t *)bufp;
  accbuf = (int16_t *)buf;

  intbuf = (int16_t *)&buf[FACCBUFSIZE*sizeof(int16_t)];  // integration buffer
  bufp = (uintptr_t)intbuf;
  if (bufp & 0xF)
    bufp = (bufp+15) & ~(uintptr_t)0xF;
  intbuf = (int16_t *)bufp;

  // clear buffers
  memclr_si128((__m128i *)accbuf, FACCBUFSIZE*sizeof(int16_t));
  memclr_si128((__m128i *)intbuf, 8*sizeof(uint16_t));
  impp = im;      // old row window pointer

  // constants
  zeros = _mm_setzero_si128();
  const_ftzero     = _mm_set1_epi16(ftzero);
  const_ftzero_x48 = _mm_set1_epi16(ftzero*48);
  const_ftzero_x2  = _mm_set1_epi16(ftzero*2);

  // loop over rows
  for (j=0; j<yim; j++, im+=xim, ftim+=xim)
    {
      accp = accbuf;    // start at beginning of buf
      intp = intbuf+8;
      acc = _mm_setzero_si128();
      imp = im;     // new row window ptr
      impp = im - PYKERN*xim;

      // initial row accumulation
      if (j<PYKERN)
  {
    for (i=0; i<xim; i+=16, imp+=16, intp+=16, accp+=16)
      {
        pxs = _mm_load_si128((__m128i *)imp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc = _mm_srli_si128(acc, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc1 = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)intp,acc1); // stored
        // next 8 pixels
        accs = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        acc = _mm_srli_si128(acc1, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc  = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)(intp+8),acc); // stored

        // update acc buffer, first 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc1,acct);
        accs = _mm_load_si128((__m128i *)accp); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)accp,acc1); // stored
        // update acc buffer, second 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7+8)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc,acct);
        accs = _mm_load_si128((__m128i *)(accp+8)); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)(accp+8),acc1); // stored

      }
  }


      else      // incremental accumulation
  {
    for (i=0; i<xim; i+=16, imp+=16, impp+=16, intp+=16, accp+=16)
      {
        pxs = _mm_load_si128((__m128i *)imp); // next 16 pixels
        opxs = _mm_load_si128((__m128i *)impp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc = _mm_srli_si128(acc, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        acct = _mm_unpacklo_epi8(opxs,zeros); // unpack first 8 into words
        accs = _mm_sub_epi16(accs, acct); // subtract it out

        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc1 = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)intp,acc1); // stored

        // next 8 pixels
        accs = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        acc = _mm_srli_si128(acc1, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        acct = _mm_unpackhi_epi8(opxs,zeros); // unpack first 8 into words
        accs = _mm_sub_epi16(accs, acct); // subtract it out

        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc  = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)(intp+8),acc); // stored

        // update acc buffer, first 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc1,acct);
        accs = _mm_load_si128((__m128i *)accp); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)accp,acc1); // stored
        // update acc buffer, second 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7+8)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc,acct);
        accs = _mm_load_si128((__m128i *)(accp+8)); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)(accp+8),acc1); // stored
      }

    // now do normalization and saving of results
    accp = accbuf+6;  // start at beginning of good values, off by 1 pixel
    imp = im - (PYKERN/2)*xim + PXKERN/2;
    ftimp = ftim - (PYKERN/2)*xim + PXKERN/2;
    for (i=0; i<xim-8; i+=16, imp+=16, accp+=16, ftimp+=16)
      {
        // sum up weighted pixels in a 4-square pattern
        pxs = _mm_loadu_si128((__m128i *)imp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc  = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_slli_epi16(accs,2); // multiply by 4
        acc  = _mm_slli_epi16(acc,2); // multiply by 4
        pxs  = _mm_loadu_si128((__m128i *)(imp+1)); // 16 pixels to the right
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp-1)); // 16 pixels to the left
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp+xim)); // 16 pixels below
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp-xim)); // 16 pixels above
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        // first 8 values
        // times weighted center by 6, giving 48x single pixel value
        acct = _mm_slli_epi16(accs,2); // multiply by 4
        accs = _mm_add_epi16(accs,accs); // double
        accs = _mm_add_epi16(accs,acct); // now x6
        // subtract from window sum
        acct = _mm_loadu_si128((__m128i *)accp);
        accs = _mm_sub_epi16(accs,acct); // subtract out norm
        // normalize to ftzero and saturate, divide by 8
        accs = _mm_srai_epi16(accs,3); // divide by 8
        accs = _mm_adds_epi16(accs,const_ftzero); // normalize to ftzero
        // saturate to [0, ftzero*2]
        accs = _mm_max_epi16(accs,zeros); // floor of 0
        accs = _mm_min_epi16(accs,const_ftzero_x2);

        // second 8 values
        // times weighted center by 6
        acct = _mm_slli_epi16(acc,2); // multiply by 4
        acc = _mm_add_epi16(acc,acc); // double
        acc = _mm_add_epi16(acc,acct); // now x6
        // subtract from window sum, get absolute value
        acct = _mm_loadu_si128((__m128i *)(accp+8));
        acc1 = _mm_sub_epi16(acc,acct);
        // normalize to ftzero and saturate, divide by 8
        acc1 = _mm_srai_epi16(acc1,3); // divide by 8
        acc1 = _mm_adds_epi16(acc1,const_ftzero); // normalize to ftzero
        // saturate to [0, ftzero*2]
        acc1 = _mm_max_epi16(acc1,zeros); // floor of 0
        acc1 = _mm_min_epi16(acc1,const_ftzero_x2);

        // pack both results
        accs = _mm_packus_epi16(accs,acc1);
        _mm_storeu_si128((__m128i *)ftimp,accs);

      }

  } // end of j >= YKERN section

    }

}



//
// fast SSE2 version
// NOTE: output buffer <ftim> better be aligned on 16-byte boundary
// NOTE: input image <im> better be aligned on 16-byte boundary
// NOTE: KSIZE (XKERN, YKERN) is fixed at 7
//
//

void
ost_do_prefilter_fast_u(
    const uint8_t *im,  // input image
    uint8_t *ftim,  // feature image output
    int xim, int yim, // size of image
    uint8_t ftzero, // feature offset from zero
    uint8_t *buf    // buffer storage
    )
{
  int i,j;

  // set up buffers, first align to 16 bytes
  uintptr_t bufp = (uintptr_t)buf;
  int16_t *accbuf, *accp;
  int16_t *intbuf, *intp; // intbuf is size xim
  const uint8_t *imp, *impp, *ftimp;
  __m128i acc, accs, acc1, acct, pxs, opxs, zeros;
  __m128i const_ftzero, const_ftzero_x48, const_ftzero_x2;

  int FACCBUFSIZE = xim+64;

  if (bufp & 0xF)
    bufp = (bufp+15) & ~(uintptr_t)0xF;
  buf = (uint8_t *)bufp;
  accbuf = (int16_t *)buf;

  intbuf = (int16_t *)&buf[FACCBUFSIZE*sizeof(int16_t)];  // integration buffer
  bufp = (uintptr_t)intbuf;
  if (bufp & 0xF)
    bufp = (bufp+15) & ~(uintptr_t)0xF;
  intbuf = (int16_t *)bufp;

  // clear buffers
  memclr_si128((__m128i *)accbuf, FACCBUFSIZE*sizeof(int16_t));
  memclr_si128((__m128i *)intbuf, 8*sizeof(uint16_t));
  impp = im;      // old row window pointer

  // constants
  zeros = _mm_setzero_si128();
  const_ftzero     = _mm_set1_epi16(ftzero);
  const_ftzero_x48 = _mm_set1_epi16(ftzero*48);
  const_ftzero_x2  = _mm_set1_epi16(ftzero*2);

  // loop over rows
  for (j=0; j<yim; j++, im+=xim, ftim+=xim)
    {
      accp = accbuf;    // start at beginning of buf
      intp = intbuf+8;
      acc = _mm_setzero_si128();
      imp = im;     // new row window ptr
      impp = im - PYKERN*xim;

      // initial row accumulation
      if (j<PYKERN)
  {
    for (i=0; i<xim; i+=16, imp+=16, intp+=16, accp+=16)
      {
        pxs = _mm_loadu_si128((__m128i *)imp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc = _mm_srli_si128(acc, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc1 = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)intp,acc1); // stored
        // next 8 pixels
        accs = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        acc = _mm_srli_si128(acc1, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc  = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)(intp+8),acc); // stored

        // update acc buffer, first 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc1,acct);
        accs = _mm_load_si128((__m128i *)accp); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)accp,acc1); // stored
        // update acc buffer, second 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7+8)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc,acct);
        accs = _mm_load_si128((__m128i *)(accp+8)); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)(accp+8),acc1); // stored

      }
  }


      else      // incremental accumulation
  {
    for (i=0; i<xim; i+=16, imp+=16, impp+=16, intp+=16, accp+=16)
      {
        pxs = _mm_loadu_si128((__m128i *)imp); // next 16 pixels
        opxs = _mm_loadu_si128((__m128i *)impp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc = _mm_srli_si128(acc, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        acct = _mm_unpacklo_epi8(opxs,zeros); // unpack first 8 into words
        accs = _mm_sub_epi16(accs, acct); // subtract it out

        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc1 = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)intp,acc1); // stored

        // next 8 pixels
        accs = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        acc = _mm_srli_si128(acc1, 14); // shift highest word to lowest
        accs = _mm_add_epi16(accs, acc); // add it in
        acct = _mm_unpackhi_epi8(opxs,zeros); // unpack first 8 into words
        accs = _mm_sub_epi16(accs, acct); // subtract it out

        // sum horizontally
        acct = _mm_slli_si128(accs, 2); // shift left one word
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 4); // shift left two words
        accs = _mm_add_epi16(accs, acct); // add it in
        acct = _mm_slli_si128(accs, 8); // shift left four words
        acc  = _mm_add_epi16(accs, acct); // add it in, done
        _mm_store_si128((__m128i *)(intp+8),acc); // stored

        // update acc buffer, first 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc1,acct);
        accs = _mm_load_si128((__m128i *)accp); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)accp,acc1); // stored
        // update acc buffer, second 8 vals
        acct = _mm_loadu_si128((__m128i *)(intp-7+8)); // previous int buffer values
        acc1 = _mm_sub_epi16(acc,acct);
        accs = _mm_load_si128((__m128i *)(accp+8)); // acc value
        acc1 = _mm_add_epi16(acc1,accs);
        _mm_store_si128((__m128i *)(accp+8),acc1); // stored
      }

    // now do normalization and saving of results
    accp = accbuf+6;  // start at beginning of good values, off by 1 pixel
    imp = im - (PYKERN/2)*xim + PXKERN/2;
    ftimp = ftim - (PYKERN/2)*xim + PXKERN/2;
    for (i=0; i<xim-8; i+=16, imp+=16, accp+=16, ftimp+=16)
      {
        // sum up weighted pixels in a 4-square pattern
        pxs = _mm_loadu_si128((__m128i *)imp); // next 16 pixels
        accs = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc  = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_slli_epi16(accs,2); // multiply by 4
        acc  = _mm_slli_epi16(acc,2); // multiply by 4
        pxs  = _mm_loadu_si128((__m128i *)(imp+1)); // 16 pixels to the right
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp-1)); // 16 pixels to the left
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp+xim)); // 16 pixels below
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        pxs  = _mm_loadu_si128((__m128i *)(imp-xim)); // 16 pixels above
        acct = _mm_unpacklo_epi8(pxs,zeros); // unpack first 8 into words
        acc1 = _mm_unpackhi_epi8(pxs,zeros); // unpack second 8 into words
        accs = _mm_add_epi16(acct,accs);
        acc  = _mm_add_epi16(acc1,acc);
        // first 8 values
        // times weighted center by 6, giving 48x single pixel value
        acct = _mm_slli_epi16(accs,2); // multiply by 4
        accs = _mm_add_epi16(accs,accs); // double
        accs = _mm_add_epi16(accs,acct); // now x6
        // subtract from window sum
        acct = _mm_loadu_si128((__m128i *)accp);
        accs = _mm_sub_epi16(accs,acct); // subtract out norm
        // normalize to ftzero and saturate, divide by 8
        accs = _mm_srai_epi16(accs,3); // divide by 8
        accs = _mm_adds_epi16(accs,const_ftzero); // normalize to ftzero
        // saturate to [0, ftzero*2]
        accs = _mm_max_epi16(accs,zeros); // floor of 0
        accs = _mm_min_epi16(accs,const_ftzero_x2);

        // second 8 values
        // times weighted center by 6
        acct = _mm_slli_epi16(acc,2); // multiply by 4
        acc = _mm_add_epi16(acc,acc); // double
        acc = _mm_add_epi16(acc,acct); // now x6
        // subtract from window sum, get absolute value
        acct = _mm_loadu_si128((__m128i *)(accp+8));
        acc1 = _mm_sub_epi16(acc,acct);
        // normalize to ftzero and saturate, divide by 8
        acc1 = _mm_srai_epi16(acc1,3); // divide by 8
        acc1 = _mm_adds_epi16(acc1,const_ftzero); // normalize to ftzero
        // saturate to [0, ftzero*2]
        acc1 = _mm_max_epi16(acc1,zeros); // floor of 0
        acc1 = _mm_min_epi16(acc1,const_ftzero_x2);

        // pack both results
        accs = _mm_packus_epi16(accs,acc1);
        _mm_storeu_si128((__m128i *)ftimp,accs);

      }

  } // end of j >= YKERN section

    }

}



#endif // __SSE2__

//
// sparse stereo
// returns disparity value in 1/16 pixel, -1 on failure
// refpat is a 16x16 patch around the feature pixel
//   feature pixel is at coordinate 7,7 within the patch
// rim is the right gradient image
// x,y is the feature pixel coordinate
// other parameters as in do_stereo
//

int
ost_do_stereo_sparse(uint8_t *refpat, uint8_t *rim, // input feature images
      int x, int y,         // position of feature pixel
    int xim, int yim, // size of images
    uint8_t ftzero, // feature offset from zero
    int dlen,   // size of disparity search, multiple of 8
    int tfilter_thresh, // texture filter threshold
    int ufilter_thresh  // uniqueness filter threshold, percent
    )
{
  int i,j,k,sum,min,ind;
  uint8_t *rp, *rpp, *rppp, *pp, *ppp;
  int cbuf[1024];   // shouldn't have more disparities than this...
  int *cp;
  float c, p, n, v;

  if (x < dlen || y < 7 || y > (yim-8)) return -1;

  min = (int)1e6;
  ind = 0;
  rp = rim + (y-7)*xim + x - dlen + 1 - 7; // upper left corner of dlen-1 disparity block
  cp = cbuf;
  for (i=0; i<dlen; i++, rp++, cp++) // loop over disparities
    {
      rpp = rp;
      pp  = refpat;
      sum = 0;
      for (j=0; j<15; j++, rpp+=xim, pp+=16)  // loop over rows
  {
    rppp = rpp;
    ppp = pp;
    for (k=0; k<15; k++)  // loop over columns
      sum += abs(*ppp++ - *rppp++);
  }
      // store sum, keep track of min
      *cp = sum;
      if (sum < min)
  {
    min = sum;
    ind = i;
  }
    }
  // do interpolation
  if (ind==0 || ind==(dlen-1))
    return (dlen-ind-1)*16;

  c = (float)min;
  p = (float)cbuf[ind+1];
  n = (float)cbuf[ind-1];
  v = (float)(dlen-ind-1) + (p-n)/(2*(p+n-2*c));
  return (int)(0.5 + 16*v);
}


