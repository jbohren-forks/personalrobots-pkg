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
#include "CHelpFleshDetect.h"

/////////////////////////////////////////////////////////////////////////////////
// CHelpFleshDetect
const int CHelpFleshDetect::DIM = 3;
//const int CHelpFleshDetect::BITS_PER_DIM = 2;

/////////////////////////////////////////////////////////////////////////////////
// INDEXING INTO m_census
void CHelpFleshDetect::ColorsToIndex(int red, int green, int blue, int &red_indx, int &green_indx, int &blue_indx) //Convert colors to their indexs
{
	red_indx = ColorToIndex(red);
	green_indx = ColorToIndex(green);
	blue_indx = ColorToIndex(blue);
}


/////////////////////////////////////////////////////////////////////////////////
//IbgrIn	- is the input 8u, Blue, Green, Red ordered image
//Iout,		- if set, must be same size as IbgrIn, must be 8u, can be 1 channel or 3 channel.  
//                Detected flesh is marked 255 in first channel of color
//                image or in the only channel of a grayscale (one channel)
//                image, else set to 0. Image can be same as IbgrIn. If NULL, no image marking is done
//rect		- if set is used as an ROI, If NULL, do whole image
// RETURNS      - fraction of pixels which are flesh
float CHelpFleshDetect::MarkColor(IplImage *IbgrIn, CvRect *rect, IplImage *Iout)
{
	if(!m_init)
		return 0.0;
	unsigned char *bl,*g,*r;
	//SET IMAGE REGION OFFSETS
	int start,len,jump,height;
	if(rect){
		start = (IbgrIn->widthStep)*(rect->y) + (rect->x)*3;
		len = rect->width;
		height = rect->height;
		jump = IbgrIn->widthStep - (rect->width*3);
	}
	else { //NO RECT, WHOLE WINDOW
		start = 0;
		len = IbgrIn->width;
		height = IbgrIn->height;
		jump = IbgrIn->widthStep - (IbgrIn->width*3);
	}

	//SET INPUT POINTERS
	bl = (unsigned char *)(IbgrIn->imageData) + start;
	g = bl+1;
	r = g+1;
	float color_likelihood;
	int count_pixels = 0;
	int count_flesh = 0;

	if(Iout) { //MARK FLESH ON THE OUTPUT IMAGE
		//SET IMAGE OUTPUT REGION OFFSETS
		int Ostep = Iout->nChannels;
		int Ostart,Ojump;
		if(rect){
			Ostart = (Iout->widthStep)*(rect->y) + (rect->x)*Ostep;
			Ojump = Iout->widthStep - (rect->width*Ostep);
		}
		else { //NO RECT, WHOLE WINDOW
			Ostart = 0;
			Ojump = Iout->widthStep - (Iout->width*Ostep);
		}
		unsigned char *mark = (unsigned char *)(Iout->imageData) + Ostart; //Set the output pointer
		//PROCESS THE DATA
		for(int y = 0; y<height; ++y) {
			for(int x = 0; x<len; ++x) {
				color_likelihood = m_census[ColorToOffset(*r, *g, *bl)];
				if(color_likelihood >= m_threshold) {
					count_flesh += 1;
					*mark = 255;//Mark the flesh in the blue or in the single channel of the output image
				}
                                else {
                                  *mark = 0;
                                }
				count_pixels += 1;
				mark += Ostep;
				bl += 3;
				g += 3;
				r += 3;
			}
			mark += Ojump;
			bl += jump;
			g += jump;
			r += jump;
		}	
	} else { //NOT MARKING, JUST CALCULATE FLESH RATIO
		//PROCESS THE DATA
		for(int y = 0; y<height; ++y) {
			for(int x = 0; x<len; ++x) {
				color_likelihood = m_census[ColorToOffset(*r, *g, *bl)];
				if(color_likelihood >= m_threshold)
					count_flesh += 1;
				count_pixels += 1;
				bl += 3;
				g += 3;
				r += 3;
			}
			bl += jump;
			g += jump;
			r += jump;
		}	
	}

	//OUTPUT FLESH RATIO
	if(count_pixels == 0) count_pixels = 1;
	return ((float)count_flesh/(float)count_pixels);
}



	

#ifdef COLORSAMPING 
void CHelpFleshDetect::FillCensusFromFrame(IplImage * img, CvRect *rect, bool mark)
{
	unsigned char *bl,*g,*r;
	//SET IMAGE REGION OFFSETS
	int start,len,jump,height;
	if(rect){
		start = (img->widthStep)*(rect->y) + (rect->x)*3;
		len = rect->width;
		height = rect->height;
		jump = img->widthStep - (rect->width*3);
	}
	else { //NO RECT, WHOLE WINDOW
		start = 0;
		len = img->width;
		height = img->height;
		jump = img->widthStep - (img->width*3);
	}

	//SET POINTERS
	bl = (unsigned char *)(img->imageData) + start;
	g = bl+1;
	r = g+1;

	if(!mark){ //Just process
		//PROCESS THE DATA
		for(int y = 0; y<height; ++y) {
			for(int x = 0; x<len; ++x) {
				const int blue = (*bl)>>(8-BITS_PER_DIM);
				const int green = (*g)>>(8-BITS_PER_DIM);
				const int red = (*r)>>(8-BITS_PER_DIM);
				IncValAt(red, green, blue, 1);
				bl += 3;
				g += 3;
				r += 3;
			}
			bl += jump;
			g += jump;
			r += jump;
		}	
	}
	else {//Mark the data
		//PROCESS THE DATA
		for(int y = 0; y<height; ++y) {
			for(int x = 0; x<len; ++x) {
				const int blue = (*bl)>>(8-BITS_PER_DIM);
				*bl = 255;
				const int green = (*g)>>(8-BITS_PER_DIM);
				const int red = (*r)>>(8-BITS_PER_DIM);
				IncValAt(red,green,blue, 1);
				bl += 3;
				g += 3;
				r += 3;
			}
			bl += jump;
			g += jump;
			r += jump;
		}
	}
}
#endif 
int MaxOfThree(int r,int g, int bl)
{
	int max; 
	if(r > g) max = r;
	else max = g;
	if(max < bl) max = bl;
	return max;
}
int MinOfThree(int r, int g, int bl)
{
	int min;
	if(r < g) min = r;
	else min = g;
	if(min > bl) min = bl;
	return min;
}
//This is a forumula for "flesh", just mainly for comparison
float CHelpFleshDetect::SkinScore(IplImage *IbgrIn, CvRect *rect, IplImage *Iout)
{

	unsigned char *bl,*g,*r;
	//SET IMAGE REGION OFFSETS
	int start,len,jump,height;
	if(rect){
		start = (IbgrIn->widthStep)*(rect->y) + (rect->x)*3;
		len = rect->width;
		height = rect->height;
		jump = IbgrIn->widthStep - (rect->width*3);
	}
	else { //NO RECT, WHOLE WINDOW
		start = 0;
		len = IbgrIn->width;
		height = IbgrIn->height;
		jump = IbgrIn->widthStep - (IbgrIn->width*3);
	}

	//SET INPUT POINTERS
	bl = (unsigned char *)(IbgrIn->imageData) + start;
	g = bl+1;
	r = g+1;


	int x,y;
	int B,G,R,max,min,max_min,RG;
	float area = (float)height*len;
	if(area == 0.0) area = 1.0;
	int skin_count = 0;
	if(Iout) { //MARK FLESH ON THE OUTPUT IMAGE
		//SET IMAGE OUTPUT REGION OFFSETS
		int Ostep = Iout->nChannels;
		int Ostart,Ojump;
		if(rect){
			Ostart = (Iout->widthStep)*(rect->y) + (rect->x)*Ostep;
			Ojump = Iout->widthStep - (rect->width*Ostep);
		}
		else { //NO RECT, WHOLE WINDOW
			Ostart = 0;
			Ojump = Iout->widthStep - (Iout->width*Ostep);
		}
		unsigned char *mark = (unsigned char *)(Iout->imageData) + Ostart; //Set the output pointer
		for(y = 0; y<height; ++y)
		{
			for(x = 0; x<len; ++x)
			{
				B = (int)(*bl);
				G = (int)(*g);
				R = (int)(*r);
				//CLASSIFY SKIN
				if((R>95)&&(G>40)&&(B>20)&&((R<220)&&(G<210)))
				{
					max = MaxOfThree(R,G,B);
					min = MinOfThree(R,G,B);
					max_min = max-min;
					if(max_min > 15){
						RG = R - G;
						if(RG < 0) RG = -RG;
						if((RG>15)&&(R>G)&&(R>B)&&(RG < 110))
						{ //THIS IS FLESH
							skin_count += 1;
							*mark = 255;
						}
					}
				}
				mark += Ostep;
				bl += 3;
				g += 3;
				r += 3;
			}
			mark += Ojump;
			bl += jump;
			g += jump;
			r += jump;
		}
	} else { //NO DRAWING
		for(y = 0; y<height; ++y)
		{
			for(x = 0; x<len; ++x)
			{
				B = (int)(*bl);
				G = (int)(*g);
				R = (int)(*r);
				//CLASSIFY SKIN
				if((R>95)&&(G>40)&&(B>20)&&((R<220)&&(G<210)))
				{
					max = MaxOfThree(R,G,B);
					min = MinOfThree(R,G,B);
					max_min = max-min;
					if(max_min > 15){
						RG = R - G;
						if(RG < 0) RG = -RG;
						if((RG>15)&&(R>G)&&(R>B)&&(RG < 110))
						{ //THIS IS FLESH
							skin_count += 1;
						}
					}
				}
				bl += 3;
				g += 3;
				r += 3;
			}
			bl += jump;
			g += jump;
			r += jump;
		}
	}
	return((float)skin_count/area);
}

/////////////////////////////////////////////////////////////////////////////////
bool CHelpFleshDetect::init(string flesh_model){
	fstream InFile;
	InFile.open(flesh_model.c_str(), ios::in);
	if (InFile.fail())
	{
		cerr << "Could not open " << flesh_model.c_str() << endl;
		return false;
	}
//	string bpd_string;
//	InFile >> bpd_string;
//	cout << "Label: " << bpd_string.c_str() << endl;
	InFile >> BITS_PER_DIM;
	if(BITS_PER_DIM < 0) {
		BITS_PER_DIM = 2;
		return false;
	}
	if (BITS_PER_DIM > 8) {
		BITS_PER_DIM = 8;
		return false;
	}
	int len_per_dim = 1<<BITS_PER_DIM;
	cout << "BITS_PER_DIM = " << BITS_PER_DIM << " => len = " << 1<<(DIM*BITS_PER_DIM) << ". Len_per_dim = " << len_per_dim << endl;
	Clear(); //properly resize and zero out the m_census vector
	int blue, green, red;
	float v;
	for(red = 0; red < len_per_dim; ++red) {
		for(green = 0; green < len_per_dim; ++green) {
			for(blue = 0; blue < len_per_dim; ++blue) {
				InFile >> v;
				cout << v << " ";
				SetValAt(red,green,blue,v);
			}
			cout << "\n";
		}
		cout << "\n" << endl;
	}
	cout << "Done " << endl;
	m_init = true;
	return true;
}


















