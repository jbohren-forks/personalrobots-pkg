//
// C version of stereo algorithm
// Basic column-oriented library
//
// Copyright 2008 by Kurt Konolige
// Videre Design
// kurt@videredesign.com
//


#include "stereolib.h"
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



#if 0
// not used in prefilter now...
//
// approximate square root
// uses piecewise linear function
// produces values bounded by maxv-2
// also has initial offset
// 

static inline uint32_t
isqrt_approx(uint32_t v, const uint32_t maxv, const uint32_t offv)
{
  uint32_t yoff;
  v -= offv;
  if (v <= maxv>>4)
    return v;

  yoff = maxv>>4;
  v -= maxv>>4;
  if (v <= maxv>>2)
    return (v>>1)+yoff;

  yoff += maxv>>3;
  v -= maxv>>2;
  if (v <= maxv)
    return (v>>2)+yoff;

  yoff += maxv>>2;
  v -= maxv;
  if (v <= maxv*4)
    return (v>>3)+yoff;

  return maxv;
}

// constant maxv of 32
static inline uint32_t
isqrt_approx_32(uint32_t v, const uint32_t offv)
{
  v -= offv;
  if (v <= 2)
    return v;

  v -= 2;
  if (v <= 8)
    return (v>>1)+2;

  v -= 8;
  if (v <= 32)
    return (v>>2)+2+4;

  v -= 32;
  if (v <= 128)
    return (v>>3)+2+4+8;

  return 32;
}
#endif


//
// simple normalization prefilter
// 9x9 mean intensity
// normalize center pixels
// feature values are unsigned bytes, offset at <ftzero> 
//
// image aligned at 16 bytes
// feature output aligned at 16 bytes
//

void
do_prefilter_norm(uint8_t *im,	// input image
	  uint8_t *ftim,	// feature image output
	  int xim, int yim,	// size of image
	  uint8_t ftzero,	// feature offset from zero
	  uint8_t *buf		// buffer storage
	  )
{
  int i,j;
  uint8_t *imp, *impp;
  uint16_t acc;
  uint8_t *topp, *toppp, *botp, *botpp, *cenp;
  int32_t qval;
  uint32_t uqval;
  uint8_t *ftimp;

  // set up buffers
  uint16_t *accbuf, *accp;
  int ACCBUFSIZE = xim+64;
  accbuf = (uint16_t *)buf;

  // image ptrs
  imp = im;			// leading
  impp = im;			// lagging

  // clear acc buffer
  memclr(accbuf, ACCBUFSIZE*sizeof(int16_t));

  // loop over rows
  for (j=0; j<yim; j++, imp+=xim)
    {
      acc = 0;			// accumulator
      accp = accbuf;		// start at beginning of buf
      topp  = imp;		// leading integration ptr
      toppp = imp;		// lagging integration ptr
      botp  = impp;		// leading integration ptr
      botpp = impp;		// lagging integration ptr

      if (j<YKERN)		// initial row accumulation
	{
	  for (i=0; i<XKERN; i++) // initial col accumulation
	    acc += *topp++;
	  for (; i<xim; i++) // incremental col acc
	    {
	      acc += *topp++ - *toppp++; // do line sum increment
	      *accp++ += acc;	// increment acc buf value
	    }
	}
      else			// incremental accumulation
	{
	  cenp = topp - (YKERN/2)*xim + XKERN/2; // center pixel
	  ftimp = ftim;
	  for (i=0; i<XKERN; i++) // initial col accumulation
	    acc += *topp++ - *botp++;
	  for (; i<xim; i++, accp++, cenp++) // incremental col acc
	    {
	      acc += *topp++ - *toppp++; // do line sum increment
	      acc -= *botp++ - *botpp++; // subtract out previous vals
	      *accp += acc;	// increment acc buf value

	      // now calculate diff from mean value and save
	      qval = 4*(*cenp) + *(cenp-1) + *(cenp+1) + *(cenp-xim) + *(cenp+xim);
	      qval = (qval*10) - *accp;	// difference with mean, norm is 81
//	      uqval = qval;
//	      uqval = uqval >> 1; // divide by 2
//	      uqval = isqrt_approx_32(uqval,4);	// limits to 32
	      qval = qval>>4;	// divide by 16 (cenp val divide by ~2)
	      if (qval < -ftzero)
		uqval = 0;
	      else if (qval > ftzero)
		uqval = ftzero+ftzero;
	      else
		uqval = ftzero - qval;
	      *ftimp++ = (uint8_t)uqval;
	    }
	  impp += xim;		// increment lagging image ptr
	  ftim +=  xim;		// increment output ptr
	}
    }
}



//
// stereo routines
//

//
// column-oriented stereo
// needs all rows before getting any disparity output
//

void
do_stereo_y(uint8_t *lim, uint8_t *rim, // input feature images
	  int16_t *disp,	// disparity output
	  int16_t *text,	// texture output
	  int xim, int yim,	// size of images
	  uint8_t ftzero,	// feature offset from zero
	  int xwin, int ywin,	// size of corr window, usually square
	  int dlen,		// size of disparity search, multiple of 8
	  int tfilter_thresh,	// texture filter threshold
	  int ufilter_thresh,	// uniqueness filter threshold, percent
	  uint8_t *buf		// buffer storage
	  )

{
  int i,j,d;			// iteration indices
  int16_t *accp, *accpp, *accminp; // acc buffer ptrs
  int8_t *limp, *rimp, *limpp, *rimpp, *limp2, *limpp2;	// feature image ptrs
  int8_t *corrend, *corrp, *corrpp; // corr buffer ptrs
  int16_t *intp, *intpp;	// integration buffer pointers
  int16_t *textpp;		// texture buffer pointer
  int16_t *dispp, *disppp;	// disparity output pointer
  int16_t *textp;		// texture output pointer
  int16_t acc, newv;
  int8_t  newc, temp;
  int16_t umin, uminthresh;	// for uniqueness check
  int dval;			// disparity value
  
  // set up buffers
#define YINTBUFSIZE (yim+64)
  int16_t *intbuf  = (int16_t *)buf;	// integration buffer
#define YTEXTBUFSIZE (yim+64)
  int16_t *textbuf = (int16_t *)&buf[YINTBUFSIZE*sizeof(int16_t)];	// texture buffer
#define YACCBUFSIZE (yim*dlen + 64)
  int16_t *accbuf  = (int16_t *)&buf[(YINTBUFSIZE + YTEXTBUFSIZE)*sizeof(int16_t)]; // accumulator buffer
  int8_t  *corrbuf = (int8_t *)&buf[(YINTBUFSIZE + YTEXTBUFSIZE + YACCBUFSIZE)*sizeof(int16_t)]; // correlation buffer

  // clear out buffers
  memclr(intbuf, yim*sizeof(int16_t));
  memclr(corrbuf, dlen*yim*xwin*sizeof(int8_t));
  memclr(accbuf, dlen*yim*sizeof(int16_t));
  memclr(textbuf, yim*sizeof(int16_t));

  // set up corrbuf pointers
  corrend = corrbuf + dlen*yim*xwin;
  corrp = corrbuf;

  // start further out on line to take care of disparity offsets
  limp = (int8_t *)lim + dlen;
  limp2 = limp;
  rimp = (int8_t *)rim + dlen;
  // offset to start of good disparity area
  dispp = disp + xim*(ywin+YKERN-2)/2 + dlen + (xwin+XKERN-2)/2; 
  textp = NULL;
  if (text != NULL)		// optional texture buffer output
    textp = text + dlen;

  // iterate over columns first
  // buffer is column-oriented, not line-oriented
  for (i=0; i<xim-XKERN-dlen+2; i++, limp++, rimp++)
    {    
      accp = accbuf;
      if (corrp >= corrend) corrp = corrbuf;
          
      // iterate over disparities, to do incremental calcs on each disparity
      // can't mix disparities!
      for (d=0; d<dlen; d++, accp+=yim, corrp+=yim)      
	{
	  limpp = limp;
	  rimpp = rimp-d;
	  intp = intbuf+ywin-1;
	  intpp = intbuf;
	  accpp = accp;
	  corrpp = corrp;
              
	  // iterate over rows
	  // have to skip down a row each time...
	  for (j=0; j<yim-YKERN+1; j++, limpp+=xim, rimpp+=xim)
	    {
	      newc = abs(*limpp - *rimpp); // new corr val
	      newv = newc - *corrpp + *intp++; // new acc val
	      *corrpp++ = newc;	// save new corr val
	      *intp = newv;	// save new acc val
	      *accpp++ += newv - *intpp++; // update window sum
	    }
	} 

      // average texture computation
      // use full corr window
      limpp = limp;
      limpp2 = limp2;
      intp = intbuf+ywin-1;
      intpp = intbuf;
      accpp = textbuf;
      acc = 0;
	  
      // iterate over rows
      // have to skip down a row each time...
      // check for initial period
      if (i < xwin)
	{
	  for (j=0; j<yim-YKERN+1; j++, limpp+=xim)
	    {
	      temp = abs(*limpp- ftzero); 
	      *intp = temp;
	      acc += *intp++ - *intpp++;
	      *accpp++ += acc;
	    }
	}
      else
	{
	  for (j=0; j<yim-YKERN+1; j++, limpp+=xim, limpp2+=xim)
	    {
	      temp = abs(*limpp-ftzero); 
	      *intp = temp - abs(*limpp2-ftzero);
	      acc += *intp++ - *intpp++;
	      *accpp++ += acc;
	    }
          limp2++;
	}
          
      // disparity extraction, find min of correlations
      if (i >= xwin)		// far enough along...
	{
	  disppp = dispp;
	  accp   = accbuf + (ywin-1); // results within initial corr window are partial
	  textpp = textbuf + (ywin-1); // texture measure

	  // iterate over rows
	  for (j=0; j<yim-ywin-YKERN+2; j++, accp++, disppp+=xim, textpp++) 
	    {
	      umin = 32000;
	      accpp = accp;
	      dval = -1;

	      if (*textpp < tfilter_thresh)
		{
		  *disppp = FILTERED;
		  continue;
		}

	      // find minimum and index over disparities
	      for (d=0; d<dlen; d++, accpp+=yim)
		{
		  if (*accpp < umin)
		    {
		      accminp = accpp;
		      umin = *accpp;
		      dval = d;
		    }
		}  

	      // check uniqueness
	      accpp = accp;	      
	      if (ufilter_thresh > 0)
		{
		  uminthresh = umin + (umin * ufilter_thresh) / 100;
		  for (d=0; d<dlen; d++, accpp+=yim)
		    {
		      if (*accpp <= uminthresh && (d < dval-1 || d > dval+1))
			{
			  dval = -1;
			  break;
			}
		    }
		}
	      if (dval < 0)
		*disppp = FILTERED;
	      else
		{
		  double c, p, n, v;
		  c = (double)*accminp;
		  p = (double)*(accminp-yim);
		  n = (double)*(accminp+yim);
		  v = (double)dval + (p-n)/(2*(p+n-2*c));
		  *disppp = (int)(0.5 + 16*v);
		}
	    } // end of row loop

	  dispp++;		// go to next column of disparity output
	}

    } // end of outer column loop

}

//
// inner loop here is over disparities
//

void
do_stereo_d(uint8_t *lim, uint8_t *rim, // input feature images
	    int16_t *disp,	// disparity output
	    int16_t *text,	// texture output
	    int xim, int yim,	// size of images
	    uint8_t ftzero,	// feature offset from zero
	    int xwin, int ywin,	// size of corr window, usually square
	    int dlen,		// size of disparity search, multiple of 8
	    int tfilter_thresh,	// texture filter threshold
	    int ufilter_thresh,	// uniqueness filter threshold, percent
	    uint8_t *buf	// buffer storage
	    )

{
  int i,j,d;			// iteration indices
  int16_t *accp, *accpp, *accminp; // acc buffer ptrs
  int8_t *limp, *rimp, *limpp, *rimpp, *limp2, *limpp2;	// feature image ptrs
  int8_t *corrend, *corrp, *corrpp; // corr buffer ptrs
  int16_t *intp, *intpp;	// integration buffer pointers
  int16_t *textpp;		// texture buffer pointer
  int16_t *dispp, *disppp;	// disparity output pointer
  int16_t *textp;		// texture output pointer
  int16_t acc, newv;
  int8_t  temp, newc;
  int16_t umin, uminthresh;	// for uniqueness check
  int dval;			// disparity value
  
  int16_t *intbuf, *textbuf, *accbuf;
  int8_t *corrbuf;

#define INTBUFSIZE ((yim-YKERN+ywin)*dlen+64)
  intbuf  = (int16_t *)buf;	// integration buffer
#define TEXTBUFSIZE (yim+64)
  textbuf = (int16_t *)&buf[INTBUFSIZE*sizeof(int16_t)];	// texture buffer
#define ACCBUFSIZE (yim*dlen + 64)
  accbuf  = (int16_t *)&buf[(INTBUFSIZE+TEXTBUFSIZE)*sizeof(int16_t)]; // accumulator buffer
  corrbuf = (int8_t *)&buf[(INTBUFSIZE+TEXTBUFSIZE+ACCBUFSIZE)*sizeof(int16_t)]; // correlation buffer

  // clear out buffers
  memclr(intbuf, dlen*yim*sizeof(int16_t));
  memclr(corrbuf, dlen*yim*xwin*sizeof(int8_t));
  memclr(accbuf, dlen*yim*sizeof(int16_t));
  memclr(textbuf, yim*sizeof(int16_t));

  // set up corrbuf pointers
  corrend = corrbuf + dlen*yim*xwin;
  corrp = corrbuf;

  // start further out on line to take care of disparity offsets
  limp = (int8_t *)lim + dlen - 1;
  limp2 = limp;
  rimp = (int8_t *)rim;
  dispp = disp + xim*(ywin+YKERN-2)/2 + dlen + (xwin+XKERN-2)/2; 
  textp = NULL;
  if (text != NULL)		// optional texture buffer output
    textp = text + dlen;

  // iterate over columns first
  // acc buffer is column-oriented, not line-oriented
  // at each iteration, move across one column
  for (i=0; i<xim-XKERN-dlen+2; i++, limp++, rimp++, corrp+=yim*dlen)
    {    
      accp = accbuf;
      if (corrp >= corrend) corrp = corrbuf;
      limpp = limp;
      rimpp = rimp;
      corrpp = corrp;
      intp = intbuf+(ywin-1)*dlen; // intbuf current ptr
      intpp = intbuf;	// intbuf old ptr
          
      // iterate over rows
      for (j=0; j<yim-YKERN+1; j++, limpp+=xim, rimpp+=xim)
	{
	  // iterate over disparities
	  // have to use an intbuf column for each disparity
	  for (d=0; d<dlen; d++, intp++)
	    {
	      // do SAD calculation
	      newc = abs(*limpp - rimpp[d]); // new corr val
	      newv = newc - *corrpp + *intp; // new acc val
	      *corrpp++ = newc;	// save new corr val
	      *(intp+dlen) = newv; // save new acc val
	      *accp++ += newv - *intpp++; // update window sum
	    }
	} 

      // average texture computation
      // use full corr window
      memclr(intbuf, ywin*sizeof(int16_t));
      limpp = limp;
      limpp2 = limp2;
      intp = intbuf+ywin-1;
      intpp = intbuf;
      accpp = textbuf;
      acc = 0;
	  
      // iterate over rows
      // have to skip down a row each time...
      // check for initial period
      if (i < xwin)
	{
	  for (j=0; j<yim-YKERN+1; j++, limpp+=xim)
	    {
	      temp = abs(*limpp- ftzero); 
	      *intp = temp;
	      acc += *intp++ - *intpp++;
	      *accpp++ += acc;
	    }
	}
      else
	{
	  for (j=0; j<yim-YKERN+1; j++, limpp+=xim, limpp2+=xim)
	    {
	      temp = abs(*limpp-ftzero); 
	      *intp = temp - abs(*limpp2-ftzero);
	      acc += *intp++ - *intpp++;
	      *accpp++ += acc;
	    }
          limp2++;
	}
          
      // disparity extraction, find min of correlations
      if (i >= xwin)		// far enough along...
	{
	  disppp = dispp;
	  accp   = accbuf + (ywin-1)*dlen; // results within initial corr window are partial
	  textpp = textbuf + (ywin-1); // texture measure

	  // iterate over rows
	  for (j=0; j<yim-ywin-YKERN+2; j++, accp+=dlen, disppp+=xim, textpp++) 
	    {
	      umin = 32000;
	      accpp = accp;
	      dval = -1;

	      if (*textpp < tfilter_thresh)
		{
		  *disppp = FILTERED;
		  continue;
		}

	      // find minimum and index over disparities
	      for (d=dlen-1; d>=0; d--, accpp++)
		{
		  if (*accpp < umin)
		    {
		      accminp = accpp;
		      umin = *accpp;
		      dval = d;
		    }
		}  

	      // check uniqueness
	      accpp = accp;	      
	      if (ufilter_thresh > 0)
		{
		  uminthresh = umin + (umin * ufilter_thresh) / 100;
		  for (d=dlen-1; d>=0; d--, accpp++)
		    {
		      if (*accpp <= uminthresh && (d < dval-1 || d > dval+1))
			{
			  dval = -1;
			  break;
			}
		    }
		}
	      if (dval < 0)
		*disppp = FILTERED;
	      else
		{
		  double c, p, n, v;
		  c = (double)*accminp;
		  p = (double)*(accminp+1);
		  n = (double)*(accminp-1);
		  v = (double)dval + (p-n)/(2*(p+n-2*c));
		  *disppp = (int)(0.5 + 16*v);
		}
	    } // end of row loop

	  dispp++;		// go to next column of disparity output
	}

    } // end of outer column loop

}


