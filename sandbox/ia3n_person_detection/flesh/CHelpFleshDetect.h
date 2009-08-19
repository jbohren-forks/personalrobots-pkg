//----------------------------------------------------------------------------
//
//	File name:  CHelpFleshDetect.h
//----------------------------------------------------------------------------
//
// Copyright (c) 2007, Rexee Inc
// All rights reserved.
//
//---------------------------------------------------------------------------- //
//   Author: Gary Bradski
//
//	Description  3D B,G,R probability model for flesh detection. 
//
//	Notes:
//	(1) This isn't a histogram, it's a P(flesh)/P(!flesh) decision model and needs a threshold to tell the decision
//  the completely neutral threshold would just be 1.0, but should probably be different to eliminate more false positives.
//
//	(2) Borrows lots from Asael's CHelpRecRGBCensus class.
//
//---------------------------------------------------------------------------- //
#pragma once
#include "cv.h"
#include "highgui.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <list>

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
// CHelpFleshDetect, was CCensus
class CHelpFleshDetect
{
private:
	vector<float> m_census;
	bool m_init;
	float m_threshold;   //Decision threshold.  Declare a pixel flesh if P(flesh)/P(!flesh) > m_threshold  DEFAULT IS 1.0;

	static const int DIM;
	int BITS_PER_DIM;
	int COLORSHIFT; //COLORSHIFT = 8-BITS_PER_DIM; 

public:
	CHelpFleshDetect(): BITS_PER_DIM(2), m_threshold(1.0) { Clear(); };
	CHelpFleshDetect(float thresh): BITS_PER_DIM(2) { m_threshold = thresh; Clear();};
	CHelpFleshDetect(int bitsperdim, float thresh) {SetBitsPerDim(bitsperdim); m_threshold = thresh; Clear();};
	CHelpFleshDetect(int bitsperdim): m_threshold(1.0) { SetBitsPerDim(bitsperdim); Clear();};

	~CHelpFleshDetect() {};

	//red, green blue are shifted indexies into m_census, effectively m_census[blue][green][red]
	inline float GetValAt(int red, int green, int blue) const { return m_census[red+(green<<BITS_PER_DIM)+(blue<<(BITS_PER_DIM<<1))]; };
	inline void SetValAt(int red, int green, int blue, float v) { m_census[red+(green<<BITS_PER_DIM)+(blue<<(BITS_PER_DIM<<1))] = v; };
	inline void IncValAt(int red, int green, int blue, float v = 1.0) { m_census[red+(green<<BITS_PER_DIM)+(blue<<(BITS_PER_DIM<<1))] += v; };

	//indexing
	inline int ColorToIndex(int color) { return (color>>COLORSHIFT);}; //Convert one color to its index value
	inline int ColorToOffset(int red, int green, int blue) { return(ColorToIndex(red)+(ColorToIndex(green)<<BITS_PER_DIM)+(ColorToIndex(blue)<<(BITS_PER_DIM<<1)));};
	void ColorsToIndex(int red, int green, int blue, int &red_indx, int &green_indx, int &blue_index); //Convert colors to their indexs

	//parameter access
	inline int GetSize() const { return (1<<(DIM*BITS_PER_DIM)); };
	int GetBitsPerDim() { return BITS_PER_DIM; };
	int GetColorShift() { return COLORSHIFT; };
	void SetBitsPerDim(int bpd) { if (bpd < 0) bpd = 2; if (bpd > 8) bpd = 8; BITS_PER_DIM = bpd; COLORSHIFT = 8-BITS_PER_DIM; Clear(); };
	float GetThreshold() { return m_threshold; };
	void SetThreshold(float t) { m_threshold = t;};


	inline void Clear() { COLORSHIFT = 8-BITS_PER_DIM; m_init = false; m_census.resize(1<<(DIM*BITS_PER_DIM), 0); };

	bool init(string flesh_model); //Init a histogram model

	/////////////////////////////////////////////////////////////////////////////////
	//IbgrIn	- is the input 8u, Blue, Green, Red ordered image
	//Iout,		- if set, must be same size as IbgrIn, must be 8u, can be 1 channel or 3 channel.  
	//            Detected flesh is marked 255, or else zero if not flesh. If color image, only the first
	//            (blue) channel is written, if 1 channel grayscale image, that image becomes a 255 or 0 flesh mask.
	//            This image can be same as IbgrIn if you want it overwritten. If NULL, no image marking is done
	//rect		- if set is used as an ROI, If NULL, do whole image
	// RETURNS:   fraction of pixels which are flesh
	float MarkColor(IplImage *IbgrIn, CvRect *rect, IplImage *Iout);

	//This is a forumula for "flesh", just mainly for comparison
	float SkinScore(IplImage *IbgrIn, CvRect *rect,  IplImage *Iout);


#ifdef COLORSAMPING 
	//FillCensusFromFrame -- if r is set, it's used as a sampleing region in the image.
	// set mark if you want to see where you are sampling from.
	void FillCensusFromFrame(IplImage * img, CvRect *rect = NULL, bool mark = false);

	const CHelpFleshDetect & operator += (const CHelpFleshDetect & other) {
		for (int x = 0; x < GetSize(); ++x)	IncValAt(x, other.GetValAt(x));
		return (*this);
	}
#endif 


};


