#include <cv.h>
#include <stdio.h>

//////////////////////////////////////////////////////////////
// int cvUpdateCodeBook(uchar *p, codeBook &c, unsigned cbBounds)
// Updates the codebook entry with a new data point
//
// p            Pointer to a YUV pixel
// c            Codebook for this pixel
// cbBounds     Learning bounds for codebook (Rule of thumb: 10)
// numChannels  Number of color channels we're learning
//
// NOTES:
//      cvBounds must be of length equal to numChannels
//
// RETURN
//   codebook index
//
int cvUpdateCodeBook( uchar *p, codeBook &c,unsigned *cbBounds, 
                      int numChannels ){
   unsigned int high[3],low[3];
   for(n=0; n<numChannels; n++)
   {
      high[n] = *(p+n)+*(cbBounds+n);
      if(high[n] > 255) high[n] = 255;
      low[n] = *(p+n)-*(cbBounds+n);
      if(low[n] > 255) low[n] = 0;
   }
   int matchChannel;
   //SEE IF THIS FITS AN EXISTING CODEWORD
   for(int i=0; i<c.numEntries; i++){
      matchChannel = 0;
      for(n=0; n<numChannels; n++){
         if((c.cb[i]->learnLow[n] <= *(p+n)) && 
         //Found an entry for this channel 
         (*(p+n) <= c.cb[i]->learnHigh[n])) 
            {
               matchChannel++;
            }
      }
      if(matchChannel == numChannels) //If an entry was found 
      {
         c.cb[i]->t_last_update = c.t;
         //adjust this codeword for the first channel
         for(n=0; n<numChannels; n++){
            if(c.cb[i]->max[n] < *(p+n))
            {
               c.cb[i]->max[n] = *(p+n);
            }
            else if(c.cb[i]->min[n] > *(p+n))
            {
               c.cb[i]->min[n] = *(p+n);
            }
         }
         break;
      }
   }
   //OVERHEAD TO TRACK POTENTIAL STALE ENTRIES
   for(int s=0; s<c.numEntries; s++){
      //Track which codebook entries are going stale:
      int negRun = c.t - c.cb[s]->t_last_update;
      if(c.cb[s]->stale < negRun) c.cb[s]->stale = negRun;
   }
   //ENTER A NEW CODE WORD IF NEEDED
   if(i == c.numEntries) //if no existing code word found, make a one
   {
      code_element **foo = new code_element* [c.numEntries+1];
      for(int ii=0; ii<c.numEntries; ii++)
      {
      foo[ii] = c.cb[ii]; 
      }
      foo[c.numEntries] = new code_element;
      if(c.numEntries) delete [] c.cb;
      c.cb = foo;
      for(n=0; n<numChannels; n++)
      {
         c.cb[c.numEntries]->learnHigh[n] = high[n];
         c.cb[c.numEntries]->learnLow[n] = low[n];
         c.cb[c.numEntries]->max[n] = *(p+n);      
         c.cb[c.numEntries]->min[n] = *(p+n);
      }
      c.cb[c.numEntries]->t_last_update = c.t;
      c.cb[c.numEntries]->stale = 0;
      c.numEntries += 1;
   }
   //SLOWLY ADJUST LEARNING BOUNDS
   for(n=0; n<numChannels; n++)
   {
      if(c.cb[i]->learnHigh[n] < high[n]) c.cb[i]->learnHigh[n] += 1;
      if(c.cb[i]->learnLow[n] > low[n]) c.cb[i]->learnLow[n] -= 1;
   }
   return(i);
}



