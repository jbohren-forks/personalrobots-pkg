//Created by Alexey Latyshev
// Set of functions for Generic Hough Transform (GHT)
#include <cv.h>
#include <highgui.h>
#include "outlet_detection/generalized_hough.h"
#include "outlet_detection/features.h"
//using namespace cv;

CV_IMPL CvSparseMat*
cvCreateOutletSparseMat( int dims, const int* sizes, int type )
{
	type = CV_MAT_TYPE( type );
	int pix_size1 = CV_ELEM_SIZE1(type);
	int pix_size = pix_size1*CV_MAT_CN(type);
	int i, size;
	CvMemStorage* storage;

	if( pix_size == 0 )
		CV_Error( CV_StsUnsupportedFormat, "invalid array data type" );

	if( dims <= 0 || dims > CV_MAX_DIM_HEAP )
		CV_Error( CV_StsOutOfRange, "bad number of dimensions" );

	if( !sizes )
		CV_Error( CV_StsNullPtr, "NULL <sizes> pointer" );

	for( i = 0; i < dims; i++ )
	{
		if( sizes[i] <= 0 )
			CV_Error( CV_StsBadSize, "one of dimesion sizes is non-positive" );
	}

	CvSparseMat* arr = (CvSparseMat*)cvAlloc(sizeof(*arr)+MAX(0,dims-CV_MAX_DIM)*sizeof(arr->size[0]));

	arr->type = CV_SPARSE_MAT_MAGIC_VAL | type;
	arr->dims = dims;
	arr->refcount = 0;
	arr->hdr_refcount = 1;
	memcpy( arr->size, sizes, dims*sizeof(sizes[0]));

	arr->valoffset = (int)cvAlign(sizeof(CvSparseNode), pix_size1);
	arr->idxoffset = (int)cvAlign(arr->valoffset + pix_size, sizeof(int));
	size = (int)cvAlign(arr->idxoffset + dims*sizeof(int), sizeof(CvSetElem));


	storage = cvCreateMemStorage( 10000000 );

	arr->heap = cvCreateSet( 0, sizeof(CvSet), size, storage );

	arr->hashsize = CV_SPARSE_HASH_SIZE0;
	size = arr->hashsize*sizeof(arr->hashtable[0]);

	arr->hashtable = (void**)cvAlloc( size );
	memset( arr->hashtable, 0, size );

	return arr;
}
// Calculates outlet's center from given feature and affine transform
CvPoint* getOutletCenter(feature_t feature, const vector<feature_t>& train_features, int feature_id, float angle1, float x_scale, float y_scale, float angle2)
{
	CvPoint* result;
	int train_length = (int)train_features.size();
	if ((feature_id < 0)||(feature_id > (train_length-1)))
		return NULL;
	CvPoint outlet_center;
	outlet_center.x = 0;
	outlet_center.y = 0;
	for (int i=0;i<train_length;i++)
	{
		outlet_center.x += train_features[i].pt.x;
		outlet_center.y += train_features[i].pt.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	float rel_center_x = - outlet_center.x + train_features[feature_id].pt.x; 
	float rel_center_y = - outlet_center.y + train_features[feature_id].pt.y; 
	float t1 = rel_center_x * cos(angle1) + rel_center_y*sin(angle1); 
	float t2 = - rel_center_x * sin(angle1) + rel_center_y * cos(angle1); 
	rel_center_x = t1*x_scale;
	rel_center_y = t2*y_scale;
	t1 = rel_center_x * cos(angle2) + rel_center_y*sin(angle2); 
	t2 = - rel_center_x * sin(angle2) + rel_center_y * cos(angle2); 

	result = new CvPoint();
	result->x = feature.pt.x-t1;
	result->y = feature.pt.y-t2;

	return result;

}
// Builds 6-dimension histogram [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvSparseMat* buildHoughHist(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges)
{
	CvSparseMat* hist;

	hist = cvCreateOutletSparseMat(6,hist_size,CV_32FC1);
	int* idx = new int[6];

	float blur_coeff = 0.17f;

	for (int n = 0; n < (int)input.size();n++)
	{
		for (int feature_id = 0; feature_id < (int)train_features.size(); feature_id++)
		{
			if (input[n].class_id != train_features[feature_id].class_id)
				continue;

			for (float angle1 = ranges[2][0]+(ranges[2][1]-ranges[2][0])/(hist_size[2]*2); angle1 <= ranges[2][1]; angle1 += ((ranges[2][1]-ranges[2][0] > 0) ? (ranges[2][1]-ranges[2][0])/hist_size[2] : 1))
			{
				for (float x_scale = ranges[3][0]+(ranges[3][1]-ranges[3][0])/(hist_size[3]*2); x_scale <= ranges[3][1]; x_scale += ((ranges[3][1]-ranges[3][0] > 0) ? (ranges[3][1]-ranges[3][0])/hist_size[3] : 1))
				{
					for (float y_scale = ranges[4][0]+(ranges[4][1]-ranges[4][0])/(hist_size[4]*2); y_scale <= ranges[4][1]; y_scale += ((ranges[4][1]-ranges[4][0] > 0) ? (ranges[4][1]-ranges[4][0])/hist_size[4] : 1))
					{
						for (float angle2 = ranges[5][0]+(ranges[5][1]-ranges[5][0])/(hist_size[5]*2); angle2 <= ranges[5][1]; angle2 += ((ranges[5][1]-ranges[5][0] > 0) ? (ranges[5][1]-ranges[5][0])/hist_size[5] : 1))
						{

							CvPoint* center = getOutletCenter(input[n],train_features,feature_id,angle1,x_scale,y_scale,angle2);

							if (center && (center->x >= ranges[0][0]) && (center->x < ranges[0][1]) && (center->y >= ranges[1][0]) && (center->y < ranges[1][1]))
							{
								//Incrementing histogram
								float idx0 = ((center->x - ranges[0][0])/(ranges[0][1]-ranges[0][0]) * hist_size[0]);
								float idx1 = ((center->y - ranges[1][0])/(ranges[1][1]-ranges[1][0]) * hist_size[1]);
								idx[0] = (int)idx0;
								idx[1] = (int)idx1;
								idx[2] = (ranges[2][1] != ranges[2][0]) ? (int)((angle1 - ranges[2][0])/(ranges[2][1]-ranges[2][0]) * hist_size[2]) : 0;
								idx[3] = (ranges[3][1] != ranges[3][0]) ?(int)((x_scale - ranges[3][0])/(ranges[3][1]-ranges[3][0]) * hist_size[3]) : 0;
								idx[4] = (ranges[4][1] != ranges[4][0]) ?(int)((y_scale - ranges[4][0])/(ranges[4][1]-ranges[4][0]) * hist_size[4]) : 0;
								idx[5] = (ranges[5][1] != ranges[5][0]) ?(int)((angle2 - ranges[5][0])/(ranges[5][1]-ranges[5][0]) * hist_size[5]) : 0;

								bool isOK = true;
								for (int i=0;i<2; i++)
								{
									if (idx[i] >= hist_size[i])
									{
										idx[i] = hist_size[i]-1;
										isOK = false;
									}
									if (idx[i] < 0)
									{
										idx[i] = 0;
										isOK = false;
									}
								}
								
								if (isOK)
								{

									float value = cvGetRealND(hist,idx);
									cvSetRealND(hist,idx,++value);

									// Fast blur
									int idx2[6];
									for (int i=0;i<6;i++)
										idx2[i]=idx[i];

									int move_x = 0;
									int move_y = 0;


									if (((idx0 - idx[0]) < blur_coeff)&&(idx[0] > 0))
										move_x=-1;
									else
										if (((-idx0 + idx[0]+1) < blur_coeff)&&(idx[0] <(hist_size[0]-1)))
											move_x=1;
									if (((idx1 - idx[1]) < blur_coeff)&&(idx[1] > 0))
										move_y=-1;
									else
										if (((-idx1 + idx[1]+1) < blur_coeff)&&(idx[1] < (hist_size[1]-1)))
											move_y=1;

									idx2[0] = idx[0]+move_x;
									idx2[1] = idx[1]+move_y;

									if ((move_x != 0) || (move_y !=0))
									{
										float value2 = cvGetRealND(hist,idx2);
										//if (value2 > value)
											cvSetRealND(hist,idx2,++value2);
										//else 
										//	cvSetRealND(hist,idx,++value);
									}

									//End

									//Bins Blur (x & y)
									//int idx2[6];
									//for (int i=0;i<6;i++)
									//	idx2[i]=idx[i];
									//for (int x = -1;x<=1;x++)
									//	for (int y= -1;y<=1;y++)
									//	{
									//		if (((idx[0]+x) >=0)&&((idx[1]+y) >=0)&&((idx[0]+x) < hist_size[0])&&((idx[1]+y) < hist_size[1]))
									//		{
									//			idx2[0] = idx[0]+x;
									//			idx2[1] = idx[1]+y;
									//			float value = cvGetRealND(hist,idx2);
									//			cvSetRealND(hist,idx2,++value);
									//		}
									//	}
									//End


								}


								delete center;
								center = 0;
							}
							if (center)
								delete center;
						}
					}
				}
			}
		}
	}
	delete[] idx;

	return hist;
}

// Builds 6-dimension histogram based on Sparse Matrix [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvMatND* buildHoughHistSparse(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges)
{
	int sizes[4];
	for (int i=0;i<4;i++)
		sizes[i] = hist_size[i+2];
	CvSparseMat* sparse;
	int sizesxy[2];
	sizesxy[0] = hist_size[0];
	sizesxy[1] = hist_size[1];

	int64 time0 = cvGetTickCount();
	CvMatND* mat = cvCreateMatND(4,sizes,CV_32SC1);
	for (int i1=0;i1<sizes[0];i1++)
		for (int i2=0;i2<sizes[1];i2++)
			for (int i3=0;i3<sizes[2];i3++)
				for (int i4=0;i4<sizes[3];i4++)
				{
					int idx_[4];
					idx_[0] = i1;
					idx_[1] = i2;
					idx_[2] = i3;
					idx_[3] = i4;
					sparse = cvCreateSparseMat(2,sizesxy,CV_32FC1);
					CvScalar s;
					s.val[0] = (size_t)sparse;
					cvSetND(mat,idx_,s);
				}
				int64 time1 = cvGetTickCount();
#if defined(_VERBOSE)
				printf("Allocation time: %f\n",float(time1 - time0)/cvGetTickFrequency()*1e-6);
#endif
				time0 = cvGetTickCount();

				int* idx = new int[6];

				for (int n = 0; n < (int)input.size();n++)
				{
					for (int feature_id = 0; feature_id < (int)train_features.size(); feature_id++)
					{
						if (input[n].class_id != train_features[feature_id].class_id)
							continue;

						for (float angle1 = ranges[2][0]+(ranges[2][1]-ranges[2][0])/(hist_size[2]*2); angle1 <= ranges[2][1]; angle1 += ((ranges[2][1]-ranges[2][0] > 0) ? (ranges[2][1]-ranges[2][0])/hist_size[2] : 1))
						{
							for (float x_scale = ranges[3][0]+(ranges[3][1]-ranges[3][0])/(hist_size[3]*2); x_scale <= ranges[3][1]; x_scale += ((ranges[3][1]-ranges[3][0] > 0) ? (ranges[3][1]-ranges[3][0])/hist_size[3] : 1))
							{
								for (float y_scale = ranges[4][0]+(ranges[4][1]-ranges[4][0])/(hist_size[4]*2); y_scale <= ranges[4][1]; y_scale += ((ranges[4][1]-ranges[4][0] > 0) ? (ranges[4][1]-ranges[4][0])/hist_size[4] : 1))
								{
									for (float angle2 = ranges[5][0]+(ranges[5][1]-ranges[5][0])/(hist_size[5]*2); angle2 <= ranges[5][1]; angle2 += ((ranges[5][1]-ranges[5][0] > 0) ? (ranges[5][1]-ranges[5][0])/hist_size[5] : 1))
									{

										CvPoint* center = getOutletCenter(input[n],train_features,feature_id,angle1,x_scale,y_scale,angle2);
										if (center && (center->x >= ranges[0][0]) && (center->x < ranges[0][1]) && (center->y >= ranges[1][0]) && (center->y < ranges[1][1]))
										{
											//Incrementing histogram

											idx[0] = (int)((center->x - ranges[0][0])/(ranges[0][1]-ranges[0][0]) * hist_size[0]);
											idx[1] = (int)((center->y - ranges[1][0])/(ranges[1][1]-ranges[1][0]) * hist_size[1]);
											idx[2] = (ranges[2][1] != ranges[2][0]) ? (int)((angle1 - ranges[2][0])/(ranges[2][1]-ranges[2][0]) * hist_size[2]) : 0;
											idx[3] = (ranges[3][1] != ranges[3][0]) ?(int)((x_scale - ranges[3][0])/(ranges[3][1]-ranges[3][0]) * hist_size[3]) : 0;
											idx[4] = (ranges[4][1] != ranges[4][0]) ?(int)((y_scale - ranges[4][0])/(ranges[4][1]-ranges[4][0]) * hist_size[4]) : 0;
											idx[5] = (ranges[5][1] != ranges[5][0]) ?(int)((angle2 - ranges[5][0])/(ranges[5][1]-ranges[5][0]) * hist_size[5]) : 0;

											bool isOK = true;
											for (int i=0;i<2; i++)
											{
												if (idx[i] >= hist_size[i])
												{
													idx[i] = hist_size[i]-1;
													isOK = false;
												}
												if (idx[i] < 0)
												{
													idx[i] = 0;
													isOK = false;
												}
											}

											if (isOK)
											{

												int idx_[4];
												int idx__[2];
												for (int i=0;i<4;i++)
													idx_[i] = idx[i+2];
												idx__[0] = idx[0];
												idx__[1] = idx[1];
												sparse = (CvSparseMat*)((int)(cvGetND(mat,idx_).val[0]));
												CvScalar s;
												s = cvGetND(sparse,idx__);
												s.val[0]++;
												cvSetND(sparse,idx__,s);

											}


											delete center;
											center = 0;
										}
										if (center)
											delete center;
									}
								}
							}
						}
					}
				}
				delete[] idx;
				time1 = cvGetTickCount();
#if defined(_VERBOSE)
				printf("Histogram calculating time: %f\n",float(time1 - time0)/cvGetTickFrequency()*1e-6);
#endif
				//return hist;
				return mat;
}

void releaseHistMat(CvMatND** hist, int* hist_size)
{
	int sizes[4];
	int idx[4];
	for (int i=0;i<4;i++)
		sizes[i] = hist_size[i+2];
	CvSparseMat* sparse_mat;
	int64 time0 = cvGetTickCount();
	//CvMatND* mat = cvCreateMatND(4,sizes,CV_32SC1);
	for (int i1=0;i1<sizes[0];i1++)
		for (int i2=0;i2<sizes[1];i2++)
			for (int i3=0;i3<sizes[2];i3++)
				for (int i4=0;i4<sizes[3];i4++)
				{
					idx[0] = i1;
					idx[1] = i2;
					idx[2] = i3;
					idx[3] = i4;

					sparse_mat = (CvSparseMat*)((int)(cvGetND(*hist,idx).val[0]));
					cvReleaseSparseMat(&sparse_mat);
				}
				cvReleaseMatND(hist);
				int64 time1 = cvGetTickCount();
#if defined(_VERBOSE)
				printf("Deallocation time: %f\n",float(time1 - time0)/cvGetTickFrequency()*1e-6);
#endif
}
//---------
// Calculates maximums of histogram.
float** getMaxHistValues(const CvHistogram* hist, int* hist_size)
{
	float** values = new float*[1];
	*values = new float[6];

	float max_val, min_val;
	int* idx = new int[6];
	cvGetMinMaxHistValue(hist,&min_val,&max_val,0,idx);
	printf("\nVotes: %f\n ",max_val);

	(*values)[0] = hist->thresh[0][0]+(hist->thresh[0][1]-hist->thresh[0][0])/hist_size[0]*idx[0];
	(*values)[1] = hist->thresh[1][0]+(hist->thresh[1][1]-hist->thresh[1][0])/hist_size[1]*idx[1];
	(*values)[2] = hist->thresh[2][0]+(hist->thresh[2][1]-hist->thresh[2][0])/hist_size[2]*idx[2];
	(*values)[3] = hist->thresh[3][0]+(hist->thresh[3][1]-hist->thresh[3][0])/hist_size[3]*idx[3];
	(*values)[4] = hist->thresh[4][0]+(hist->thresh[4][1]-hist->thresh[4][0])/hist_size[4]*idx[4];
	(*values)[5] = hist->thresh[5][0]+(hist->thresh[5][1]-hist->thresh[5][0])/hist_size[5]*idx[5];

	delete[] idx;
	return values;
}
//---------
//---------
// Calculates maximums of histogram.
void getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float** &maxs, int& count, int MIN_VOTES)
{
	count = 0;
	//int MIN_VOTES = 4;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( (CvSparseMat*)hist->bins, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		if (val >= MIN_VOTES)
			(count) ++;
	}
	//Endof

	if (count > 0)
	{
		maxs = new float*[count];
		for (int i=0; i<(count); i++)
			maxs[i] = new float[6];

		int i=0;
		node = cvInitSparseMatIterator( hist, &mat_iterator );
		for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
		{
			int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
			float val = *(float*)CV_NODE_VAL( (CvSparseMat*)hist, node ); /* get value of the element
																		  (assume that the type is CV_32FC1) */
			if (val >= MIN_VOTES)
			{
				maxs[i][0] = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5);
				maxs[i][1] = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5);
				maxs[i][2] = ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[2]+0.5);
				maxs[i][3] = ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[3]+0.5);
				maxs[i][4] = ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[4]+0.5);
				maxs[i][5] = ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[5]+0.5);
				i++;
			}

			//delete[] idx;
		}

	}
	else
	{
		maxs = NULL;
		count = 0;
	}

}
//---------
int getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float** &maxs, int& count)
{
	//IplImage* img = cvCreateImage(cvSize(640,480),8,3); 
	count = 0;
	float MIN_VOTES = 0;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		//if (val > 0)
		//{
		//	//CvPoint pt;
		//	int* idx = CV_NODE_IDX( hist, node );
		//	int x = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5);
		//	int y = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5);
		//	int blue = ((uchar*)(img->imageData + img->widthStep*y))[x*3];
		//	int green = ((uchar*)(img->imageData + img->widthStep*y))[x*3+1];
		//	int red = ((uchar*)(img->imageData + img->widthStep*y))[x*3+2];
		//	int v = blue+green+red;

		//	v+=val;
		//	blue = ((v / 255 > 0) ? 255 : v % 255);

		//	green = ((v / 510 > 0) ? 255 : ((blue == 255) ? v % 510 - 255 : 0));

		//	red = ((green == 255) ? v % 765 - 510 : 0);

		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3] = blue;
		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3+1] = 100;
		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3+2] = red;
		//	

		//	//CvScalar color; color.val[0] = img->imageData[pt.y*img->width + pt.x];
		//	//color.val[0] +=5;
		//	// img->imageData[pt.y*img->width + pt.x]+=5;
		//	cvDrawRect(img,cvPoint(x-2,y-2),cvPoint(x+2,y+2),cvScalar(blue,green,red),CV_FILLED);

		//}
		if (val > MIN_VOTES)
			MIN_VOTES = val;
	}
	//cvSaveImage("d:\\1.bmp",img);

	node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		if (val == MIN_VOTES)
			(count) ++;
	}
	//Endof

	if (count > 0)
	{
		maxs = new float*[count];
		for (int i=0; i<(count); i++)
			maxs[i] = new float[6];

		int i=0;
		node = cvInitSparseMatIterator( hist, &mat_iterator );
		for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
		{
			int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
			float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
															(assume that the type is CV_32FC1) */
			if (val == MIN_VOTES)
			{
				maxs[i][0] = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5);
				maxs[i][1] = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5);
				maxs[i][2] = ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[2]+0.5);
				maxs[i][3] = ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[3]+0.5);
				maxs[i][4] = ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[4]+0.5);
				maxs[i][5] = ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[5]+0.5);
				i++;
			}

			//delete[] idx;
		}

	}
	else
	{
		maxs = NULL;
		count = 0;
	}

	int res = (int)MIN_VOTES;
	return res;

}
//---------
// Calculates maximums of histogram.
void getMaxSparseHistValues(const CvMatND* hist, int* hist_size, float** ranges, float** &maxs, int& count, int MIN_VOTES)
{
	count = 0;
	//int MIN_VOTES = 4;
	CvSparseMat* sparse_mat;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node;
	int sizes[4];
	int idx[4];
	for (int i=0;i<4;i++)
		sizes[i] = hist_size[i+2];

	for (int i1=0;i1<sizes[0];i1++)
		for (int i2=0;i2<sizes[1];i2++)
			for (int i3=0;i3<sizes[2];i3++)
				for (int i4=0;i4<sizes[3];i4++)
				{

					idx[0] = i1;
					idx[1] = i2;
					idx[2] = i3;
					idx[3] = i4;
					CvSparseMat* sparse_mat = (CvSparseMat*)((int)(cvGetND(hist,idx).val[0]));

					node = cvInitSparseMatIterator(sparse_mat, &mat_iterator );

					for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
					{
						//  const int* idx = CV_NODE_IDX( (CvSparseMat*)hist->bins, node ); /* get pointer to the element indices */
						float val = *(float*)CV_NODE_VAL( sparse_mat, node ); /* get value of the element
																			  (assume that the type is CV_32FC1) */
						if (val >= MIN_VOTES)
							(count) ++;
					}
				}
				//Endof

				if (count > 0)
				{
					maxs = new float*[count];
					for (int i=0; i<(count); i++)
						maxs[i] = new float[6];

					int i=0;
					for (int i1=0;i1<sizes[0];i1++)
						for (int i2=0;i2<sizes[1];i2++)
							for (int i3=0;i3<sizes[2];i3++)
								for (int i4=0;i4<sizes[3];i4++)
								{

									idx[0] = i1;
									idx[1] = i2;
									idx[2] = i3;
									idx[3] = i4;
									sparse_mat = (CvSparseMat*)((int)(cvGetND(hist,idx).val[0]));
									node = cvInitSparseMatIterator( sparse_mat, &mat_iterator );
									for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
									{
										int* idx_xy = CV_NODE_IDX( sparse_mat, node ); /* get pointer to the element indices */
										float val = *(float*)CV_NODE_VAL( sparse_mat, node ); /* get value of the element
																							  (assume that the type is CV_32FC1) */
										if (val >= MIN_VOTES)
										{
											maxs[i][0] = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx_xy[0]+0.5);
											maxs[i][1] = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx_xy[1]+0.5);
											maxs[i][2] = ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[0]+0.5);
											maxs[i][3] = ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[1]+0.5);
											maxs[i][4] = ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[2]+0.5);
											maxs[i][5] = ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[3]+0.5);
											i++;
										}

										//delete[] idx;
									}
								}

				}
				else
				{
					maxs = NULL;
					count = 0;
				}

}
//---------
void getMaxSparseHistValues(const CvMatND* hist, int* hist_size, float** ranges, float** &maxs, int& count)
{

	count = 0;
	float MIN_VOTES = 0;
	CvSparseMat* sparse_mat;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node;
	int sizes[4];
	int idx[4];
	for (int i=0;i<4;i++)
		sizes[i] = hist_size[i+2];

	for (int i1=0;i1<sizes[0];i1++)
		for (int i2=0;i2<sizes[1];i2++)
			for (int i3=0;i3<sizes[2];i3++)
				for (int i4=0;i4<sizes[3];i4++)
				{

					idx[0] = i1;
					idx[1] = i2;
					idx[2] = i3;
					idx[3] = i4;
					sparse_mat = (CvSparseMat*)((int)(cvGetND(hist,idx).val[0]));

					node = cvInitSparseMatIterator( sparse_mat, &mat_iterator );
					//int nv = 0;

					for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
					{
						//  const int* idx = CV_NODE_IDX( (CvSparseMat*)hist->bins, node ); /* get pointer to the element indices */
						float val = *(float*)CV_NODE_VAL( sparse_mat, node ); /* get value of the element
																			  (assume that the type is CV_32FC1) */
						if (val > MIN_VOTES)
							MIN_VOTES = val;

					}

				}
				printf("Votes: %f\n",MIN_VOTES);

				for (int i1=0;i1<sizes[0];i1++)
					for (int i2=0;i2<sizes[1];i2++)
						for (int i3=0;i3<sizes[2];i3++)
							for (int i4=0;i4<sizes[3];i4++)
							{

								idx[0] = i1;
								idx[1] = i2;
								idx[2] = i3;
								idx[3] = i4;
								sparse_mat = (CvSparseMat*)((int)(cvGetND(hist,idx).val[0]));
								node = cvInitSparseMatIterator( sparse_mat, &mat_iterator );

								for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
								{
									//  const int* idx = CV_NODE_IDX( (CvSparseMat*)hist->bins, node ); /* get pointer to the element indices */
									float val = *(float*)CV_NODE_VAL( sparse_mat, node ); /* get value of the element
																						  (assume that the type is CV_32FC1) */
									if (val == MIN_VOTES)
										(count) ++;
								}
								//Endof
							}

							if (count > 0)
							{
								maxs = new float*[count];
								for (int i=0; i<(count); i++)
									maxs[i] = new float[6];

								int i=0;
								for (int i1=0;i1<sizes[0];i1++)
									for (int i2=0;i2<sizes[1];i2++)
										for (int i3=0;i3<sizes[2];i3++)
											for (int i4=0;i4<sizes[3];i4++)
											{

												idx[0] = i1;
												idx[1] = i2;
												idx[2] = i3;
												idx[3] = i4;
												CvSparseMat* sparse_mat = (CvSparseMat*)((int)(cvGetND(hist,idx).val[0]));
												node = cvInitSparseMatIterator( sparse_mat, &mat_iterator );
												for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
												{
													int* idx_xy = CV_NODE_IDX( sparse_mat, node ); /* get pointer to the element indices */
													float val = *(float*)CV_NODE_VAL( sparse_mat, node ); /* get value of the element
																										  (assume that the type is CV_32FC1) */
													if (val == MIN_VOTES)
													{
														maxs[i][0] = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx_xy[0]+0.5);
														maxs[i][1] = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx_xy[1]+0.5);
														maxs[i][2] = ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[0]+0.5);
														maxs[i][3] = ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[1]+0.5);
														maxs[i][4] = ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[2]+0.5);
														maxs[i][5] = ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[3]+0.5);
														i++;
													}

													//delete[] idx;
												}
											}

							}
							else
							{
								maxs = NULL;
								count = 0;
							}

}

// Calculates outlet features from given train outlet and affine transform
// Affine transform is array [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
void calcOutletPosition(const vector<feature_t>& train_features, float* affine_transform, vector<feature_t>& features)
{
	CvPoint center = cvPoint(affine_transform[0],affine_transform[1]);
	float angle1 = affine_transform[2];
	float x_scale = affine_transform[3];
	float y_scale = affine_transform[4];
	float angle2 = affine_transform[5];
	int train_length = (int)train_features.size();
	//CvPoint* result = new CvPoint[train_length];

	CvPoint outlet_center;
	outlet_center.x = 0;
	outlet_center.y = 0;
	for (int i=0;i<train_length;i++)
	{
		outlet_center.x += train_features[i].pt.x;
		outlet_center.y += train_features[i].pt.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	for (int i=0; i< train_length; i++)
	{
		float rel_center_x = - outlet_center.x + train_features[i].pt.x; 
		float rel_center_y = - outlet_center.y + train_features[i].pt.y; 
		float t1 = rel_center_x * cos(angle1) + rel_center_y*sin(angle1); 
		float t2 = - rel_center_x * sin(angle1) + rel_center_y * cos(angle1); 
		rel_center_x = t1*x_scale;
		rel_center_y = t2*y_scale;
		t1 = rel_center_x * cos(angle2) + rel_center_y*sin(angle2); 
		t2 = - rel_center_x * sin(angle2) + rel_center_y * cos(angle2); 

		CvPoint result_point;
		result_point.x = center.x+t1;
		result_point.y = center.y+t2;
		feature_t feature;
		feature.size = train_features[i].size;
		feature.pt = result_point;
		feature.class_id = train_features[i].class_id;

		features.push_back(feature);
	}


}

void calcExactLocation1(vector<feature_t>& features, vector<feature_t>& outlet)
{
	int features_length = (int)(features.size());
	int outlet_length = (int)(outlet.size());
	CvPoint* distance_vectors = new CvPoint[outlet_length];
	float* distances = new float[outlet_length];
	int* indexes = new int[outlet_length];
	float* angles = new float[outlet_length];
	for (int i=0;i<outlet_length;i++)
	{
		int min_index = 0;
		float min_distance = 1e30;
		for (int j=0;j<features_length;j++)
		{
			if (features[j].class_id == outlet[i].class_id)
			{
				float distance = (features[j].pt.x - outlet[i].pt.x)*(features[j].pt.x - outlet[i].pt.x)+
					(features[j].pt.y - outlet[i].pt.y)*(features[j].pt.y - outlet[i].pt.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].pt - outlet[i].pt;
			if (distances[i] > 0)
			{
				if (float angle = asin(distance_vectors[i].y/sqrt(distances[i])) > 0)
				{
					angles[i] = acos(distance_vectors[i].x/sqrt(distances[i]));
				}
				else 
				{
					angles[i] = acos(distance_vectors[i].x/sqrt(distances[i])) + CV_PI;
				}
			}
		}
	}

	CvPoint move = cvPoint(0,0);

	//for (int i=0;i<outlet_length;i++)
	//{
	//	if (distances[i]<distances[index])
	//		index = i;
	//}
	// move = distance_vectors[index];
	int nsegm = 16;
	float maxd = 0;
	float mind = 1e38;
	float mina = CV_PI*2;
	float maxa = 0;
	for (int i=0; i <outlet_length; i++)
	{
		indexes[i] = 1;
		if (maxd<distances[i])
			maxd = distances[i];
		if (mind>distances[i])
			mind = distances[i];
		if (maxa<angles[i])
			maxa = angles[i];
		if (mina>angles[i])
			mina = angles[i];
	}

	int* votes = new int[nsegm];
	int maxind = 0;
	for (int i=0;i<nsegm;i++)
		votes[i] = 0;
	for (int i=0; i <outlet_length; i++)
	{
		int ind = (int)((angles[i]-mina)/(maxa-mina)*nsegm);
		if (ind == nsegm)
			ind--;
		ind = ind % nsegm;
		votes[ind]++;
		//Blur
		if (ind > 1)
			votes[ind-1]++;
		if (ind < (nsegm-1))
			votes[ind+1]++;
		//End of
		if (votes[maxind] < votes[ind])
			maxind = ind;
	}

	for (int i=0; i <outlet_length; i++)
	{
		int ind = (int)((angles[i]-mina)/(maxa-mina)*nsegm);
		if (ind == nsegm)
			ind--;
		ind = ind % nsegm;
		//	int ind = (int)(angles[i]*nsegm*0.5/CV_PI);
		if (abs(ind-maxind)>1)
			indexes[i]=0;
	}

	maxind = 0;
	for (int i=0;i<nsegm;i++)
		votes[i] = 0;
	for (int i=0; i <outlet_length; i++)
	{
		if (indexes[i] > 0)
		{
			int ind = (int)((distances[i]-mind)/(maxd-mind)*nsegm);
			if (ind == nsegm)
				ind--;
			ind = ind % nsegm;
			votes[ind]++;
			//Votes Blur
			if (ind > 1)
				votes[ind-1]++;
			if (ind < (nsegm-1))
				votes[ind+1]++;
			//End of
			if (votes[maxind] < votes[ind])
				maxind = ind;
		}
	}
	for (int i=0; i <outlet_length; i++)
	{
		if (indexes[i] > 0)
		{
			int ind = (int)((distances[i]-mind)/(maxd-mind)*nsegm);
			if (ind == nsegm)
				ind--;
			ind = ind % nsegm;
			if (abs(ind-maxind)>1)
				indexes[i]=0;
		}
	}

	delete[] votes;

	int count = 0;
	for (int i=0;i<outlet_length;i++)
	{
		if (indexes[i] > 0)
		{
			//Added
			//	if (distances[maxindex] > distances[i])
			//		maxindex = i;
			//End
			move.x += distance_vectors[i].x;
			move.y += distance_vectors[i].y;
			count++;
		}
	}

	move.x /= count;
	move.y /= count;

	//Added
	//move = distance_vectors[maxindex];
	//End

	for (int i=0;i<outlet_length;i++)
	{
		outlet[i].pt.x += move.x;
		outlet[i].pt.y += move.y;
	}

	delete[] distances;
	delete[] indexes;
	delete[] distance_vectors;
	delete[] angles;
}

//-------------------
void calcExactLocation2(vector<feature_t>& features, vector<feature_t>& outlet, int accuracy)
{
	int features_length = (int)(features.size());
	int outlet_length = (int)(outlet.size());
	//int accuracy = 2;
	CvRect outlet_rect = cvRect(outlet[0].pt.x,outlet[0].pt.y,0,0);
	for (int i=1;i<(int)(outlet.size());i++)
	{
		if (outlet[i].pt.x < outlet_rect.x)
		{
			outlet_rect.width += (outlet_rect.x - outlet[i].pt.x);
			outlet_rect.x = outlet[i].pt.x;
		}

		if (outlet[i].pt.x > (outlet_rect.x+outlet_rect.width))
		{
			outlet_rect.width = outlet[i].pt.x - outlet_rect.x;
		}

		if (outlet[i].pt.y < outlet_rect.y)
		{
			outlet_rect.height += (outlet_rect.y - outlet[i].pt.y);
			outlet_rect.y = outlet[i].pt.y;
		}

		if (outlet[i].pt.y > (outlet_rect.x+outlet_rect.height))
		{
			outlet_rect.height = outlet[i].pt.y - outlet_rect.y;
		}
	}

	//	float x_ranges[2] = {outlet_rect.x - outlet_rect.width/2 , outlet_rect.x + 3*outlet_rect.width/2};
	//  float y_ranges[2] = {outlet_rect.y - outlet_rect.height/2 , outlet_rect.y + 4*outlet_rect.height/2};
	float x_ranges[2] = {-outlet_rect.width , outlet_rect.width};
	float y_ranges[2] = {-outlet_rect.height ,outlet_rect.height};

	CvPoint movement;
	movement.x = x_ranges[0];
	movement.y = y_ranges[0];
	int votes = 0;

	for (int x = x_ranges[0]; x<=x_ranges[1];x+=accuracy)
	{
		for (int y = y_ranges[0]; y<=y_ranges[1];y+=accuracy)
		{
			int nvotes = 0;
			for (int i = 0; i< outlet_length; i++)
			{
				for (int j = 0; j< features_length;j++)
				{
					if (outlet[i].class_id == features[j].class_id)
					{
						// Change to normal metrics
						float dist = (outlet[i].pt.x + x - features[j].pt.x)*(outlet[i].pt.x + x - features[j].pt.x) + 
                            (outlet[i].pt.y + y - features[j].pt.y)*(outlet[i].pt.y + y - features[j].pt.y);
						if (dist < accuracy*accuracy)
						{
							nvotes ++;
						}
					}
				}
			}
			if (nvotes > votes)
			{
				movement.x = x;
				movement.y = y;
				votes = nvotes;
			}

		}
	}

	for (int i = 0; i< outlet_length; i++)
	{
		outlet[i].pt.x +=movement.x;
		outlet[i].pt.y +=movement.y;
	}


}
//---------------------
void calcExactLocation3(vector<feature_t>& features, vector<feature_t>& outlet, int accuracy)
{
	int features_length = (int)(features.size());
	int outlet_length = (int)(outlet.size());
	CvPoint* distance_vectors = new CvPoint[outlet_length];
	float* distances = new float[outlet_length];
	int* indexes = new int[outlet_length];
	int* votes = new int[outlet_length];
	for (int i=0;i<outlet_length;i++)
	{
		votes[i]=0;
		int min_index = 0;
		float min_distance = 1e30;
		for (int j=0;j<features_length;j++)
		{
			if (features[j].class_id == outlet[i].class_id)
			{
				float distance = (features[j].pt.x - outlet[i].pt.x)*(features[j].pt.x - outlet[i].pt.x)+
					(features[j].pt.y - outlet[i].pt.y)*(features[j].pt.y - outlet[i].pt.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].pt - outlet[i].pt;
		}
	}

	for (int i=0;i<outlet_length;i++)
	{
		for (int j=0;j<features_length;j++)
		{
			for (int k=0;k<outlet_length;k++)
			{
				if (k!=i)
				{
					float d = (outlet[k].pt.x + distance_vectors[i].x - features[j].pt.x)*(outlet[k].pt.x + distance_vectors[i].x - features[j].pt.x)+
						(outlet[k].pt.y + distance_vectors[i].y- features[j].pt.y)*(outlet[k].pt.y + distance_vectors[i].y- features[j].pt.y);
					if (d < accuracy*accuracy)
					{
						votes[i]++;
						//	j = features_length;
						k = outlet_length;

					}
				}
				//if ((indexes[i] != j) && (d < accuracy*accuracy))
				//{
				//	votes[i]++;
				//	// Try to comment next line
				////	j = features_length;
				//}
			}
		}
	}

	int maxindex = 0;

	for (int i=1; i<outlet_length;i++)
	{
		if (votes[i] > votes[maxindex])
			maxindex = i;
	}

	for (int i = 0; i< outlet_length; i++)
	{
		outlet[i].pt.x +=distance_vectors[maxindex].x;
		outlet[i].pt.y +=distance_vectors[maxindex].y;
	}



	delete[] distances;
	delete[] indexes;
	delete[] distance_vectors;
	delete[] votes;
}


void calcExactLocation4(vector<feature_t>& features, vector<feature_t>& outlet)
{
	int features_length = (int)(features.size());
	int outlet_length = (int)(outlet.size());
	CvPoint* distance_vectors = new CvPoint[outlet_length];
	float* distances = new float[outlet_length];
	int* indexes = new int[outlet_length];
	float* angles = new float[outlet_length];
	for (int i=0;i<outlet_length;i++)
	{
		int min_index = 0;
		float min_distance = 1e30;
		for (int j=0;j<features_length;j++)
		{
			if (features[j].class_id == outlet[i].class_id)
			{
				float distance = (features[j].pt.x - outlet[i].pt.x)*(features[j].pt.x - outlet[i].pt.x)+
					(features[j].pt.y - outlet[i].pt.y)*(features[j].pt.y - outlet[i].pt.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].pt - outlet[i].pt;
			if (distances[i] > 0)
			{
				if (float angle = asin(distance_vectors[i].y/sqrt(distances[i])) > 0)
				{
					angles[i] = acos(distance_vectors[i].x/sqrt(distances[i]));
				}
				else 
				{
					angles[i] = acos(distance_vectors[i].x/sqrt(distances[i])) + CV_PI;
				}
			}
		}
	}

	int index = 0;
	CvPoint move = cvPoint(0,0);

	//for (int i=0;i<outlet_length;i++)
	//{
	//	if (distances[i]<distances[index])
	//		index = i;
	//}
	// move = distance_vectors[index];
	int nsegm = 16;
	float maxd = 0;
	float mind = 1e38;
	float mina = CV_PI*2;
	float maxa = 0;
	for (int i=0; i <outlet_length; i++)
	{
		indexes[i] = 1;
		if (maxd<distances[i])
			maxd = distances[i];
		if (mind>distances[i])
			mind = distances[i];
		if (maxa<angles[i])
			maxa = angles[i];
		if (mina>angles[i])
			mina = angles[i];
	}

	int* votes = new int[nsegm];
	int maxind = 0;
	for (int i=0;i<nsegm;i++)
		votes[i] = 0;
	for (int i=0; i <outlet_length; i++)
	{
		int ind = (int)((angles[i]-mina)/(maxa-mina)*nsegm);
		if (ind == nsegm)
			ind--;
		ind = ind % nsegm;
		votes[ind]++;
		//Votes Angles Blur
		if (ind > 1)
			votes[ind-1]++;
		if (ind < (nsegm-1))
			votes[ind+1]++;
		//End of
		if (votes[maxind] < votes[ind])
			maxind = ind;
	}

	for (int i=0; i <outlet_length; i++)
	{
		int ind = (int)((angles[i]-mina)/(maxa-mina)*nsegm);
		if (ind == nsegm)
			ind--;
		ind = ind % nsegm;
		//	int ind = (int)(angles[i]*nsegm*0.5/CV_PI);
		if (abs(ind-maxind)>1)
			indexes[i]=0;
	}

	maxind = 0;
	for (int i=0;i<nsegm;i++)
		votes[i] = 0;
	for (int i=0; i <outlet_length; i++)
	{
		if (indexes[i] > 0)
		{
			int ind = (int)((distances[i]-mind)/(maxd-mind)*nsegm);
			if (ind == nsegm)
				ind--;
			ind = ind % nsegm;
			votes[ind]++;
			//Votes distance Blur
			//if (ind > 1)
			//	votes[ind-1]++;
			//if (ind < (nsegm-1))
			//	votes[ind+1]++;
			//End of
			if (votes[maxind] < votes[ind])
				maxind = ind;
		}
	}
	for (int i=0; i <outlet_length; i++)
	{
		if (indexes[i] > 0)
		{
			int ind = (int)((distances[i]-mind)/(maxd-mind)*nsegm);
			if (ind == nsegm)
				ind--;
			ind = ind % nsegm;
			if (abs(ind-maxind)>1)
				indexes[i]=0;
		}
	}

	delete[] votes;

	int count = 0;
	int maxindex = 0;
	for (int i=0;i<outlet_length;i++)
	{
		if (indexes[i] > 0)
		{
			//Added
			//	if (distances[maxindex] > distances[i])
			//		maxindex = i;
			//End
			outlet[i].pt.x += distance_vectors[i].x;
			outlet[i].pt.y += distance_vectors[i].y;
			//move.x += distance_vectors[i].x;
			//	move.y += distance_vectors[i].y;
			count++;
		}
	}

	//move.x /= count;
	//move.y /= count;

	//////Added
	//////move = distance_vectors[maxindex];
	//////End

	//for (int i=0;i<outlet_length;i++)
	//{
	//	if (indexes[i] == 0)
	//	{
	//		outlet[i].center.x += move.x;
	//		outlet[i].center.y += move.y;
	//	}
	//}

	delete[] distances;
	delete[] indexes;
	delete[] distance_vectors;
	delete[] angles;
}

void calcExactLocation(vector<feature_t>& features,const vector<feature_t>& train_features, vector<feature_t>& src_outlet, vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy)
{
	if (((int)train_features.size()) == ((int)src_outlet.size()))
	{
		vector<CvPoint> train_points;
		train_points.clear();

		//vector<CvPoint> src_outlet_points;
		vector<CvPoint> features_points;
		features_points.clear();
		int* indexes = new int[(int)train_features.size()];
		float max_diff_coeff = 2.0f;

		for (int i=0;i<(int)train_features.size();i++)
		{
			indexes[i] = -1;
		}
		//for (int i=0;i<(int)src_outlet.size();i++)
		//{
		//	src_outlet_points.push_back(src_outlet[i].center);
		//}
		for (int i=0;i<(int)src_outlet.size();i++)
		{
			int min_index = -1;
			float min_distance = 1e30;
			float last_min_distance;
			for (int j=0;j<(int)features.size();j++)
			{
				if (features[j].class_id == src_outlet[i].class_id)
				{
					float distance = (features[j].pt.x - src_outlet[i].pt.x)*(features[j].pt.x - src_outlet[i].pt.x)+
						(features[j].pt.y - src_outlet[i].pt.y)*(features[j].pt.y - src_outlet[i].pt.y);
					if ((distance < min_distance)/* && (distance < accuracy*accuracy)*/)
					{
						last_min_distance = min_distance;
						min_distance = distance;
						min_index = j;
					}	
					else
					{
						if (distance < last_min_distance)
						{
							last_min_distance = distance;
						}
					}
				}

			}
			if (min_distance < accuracy*accuracy)
				indexes[i] = min_index;
			else
				min_index = -1;
			if (min_index > -1)
			{
				if ((min_distance > 0) && (last_min_distance/min_distance <= max_diff_coeff))
				{
					indexes[i] = -1;
					//printf("---->Unable to choose feature\n");
				}
			}
		}

		// Removing the same indexes
		for (int i=0;i< (int)(src_outlet.size());i++)
		{
			if (indexes[i] >=0)
			{
				bool wasRepeat = false;
				for (int j=i+1;j< (int)(src_outlet.size());j++)			
				{
					if (indexes[i]==indexes[j])
					{
						indexes[j] = -1;
						wasRepeat = true;
					}
				}
				if (wasRepeat)
					indexes[i] = -1;
			}
		}
		//-

		for (int i=0;i< (int)(src_outlet.size());i++)
		{
			if (indexes[i] >=0)
			{
				train_points.push_back(train_features[i].pt);
				features_points.push_back(features[indexes[i]].pt);
			}
		}

		if (((int)train_points.size() > 3) /*&& ((int)train_points.size() > ((int)train_features.size()/2))*/)
		{
			CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
			FindAffineTransform(train_points, features_points, homography);
			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography) + 1000000 - 10000*(int)train_points.size();
			dst_outlet.clear();
			MapFeaturesAffine(train_features, dst_outlet, homography);

			for (int i=0;i< (int)(dst_outlet.size());i++)
			{

					// The second attraction
					//Temp
				int min_index = -1;
				float min_distance = 1e38;
				float last_min_distance;
				for (int j=0;j<(int)features.size();j++)
				{				
					if (features[j].class_id == dst_outlet[i].class_id)
					{
						float distance = (features[j].pt.x - dst_outlet[i].pt.x)*(features[j].pt.x - dst_outlet[i].pt.x)+
							(features[j].pt.y - dst_outlet[i].pt.y)*(features[j].pt.y - dst_outlet[i].pt.y);
						if (distance < min_distance)
						{
							last_min_distance = distance;
							//setting min distance to power holes distance / 3
							float acc = (float)((train_features[1].pt.x - train_features[0].pt.x)*(train_features[1].pt.x - train_features[0].pt.x)+
											(train_features[1].pt.y - train_features[0].pt.y)*(train_features[1].pt.y - train_features[0].pt.y))/9;
							if (distance < acc)
							{
								min_distance = distance;
								min_index = j;
							}
						}	
						else
						{
							if (distance < last_min_distance)
							{
								last_min_distance = distance;
							}
						}
					}
				}
				if (min_index >= 0)
				{
					if (((min_distance > 0) && (last_min_distance/min_distance <= max_diff_coeff))||(min_distance == 0))
						dst_outlet[i] = features[min_index];
					
				}
				else

					//End

					if (indexes[i] >=0)
					{
						dst_outlet[i] = features[indexes[i]];
					}
			}			


			//// Other calculation of reproj. error
			//train_points.clear();
			//features_points.clear();
			//for (int i=0;i< (int)(src_outlet.size());i++)
			//{

			//	train_points.push_back(train_features[i].center);
			//	features_points.push_back(dst_outlet[i].center);

			//}
			//reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography)/* + 1000*(int)train_points.size()*/;
			//// end

			cvReleaseMat(&homography);
		}
		else
		{
			dst_outlet.clear();
			reprojectionError = 1e38;
		}


		delete[] indexes;
	}
	else
	{
		dst_outlet.clear();
		reprojectionError = 1e38;
	}


}

void calcExactLocation_(vector<feature_t>& features,const vector<feature_t>& train_features, vector<feature_t>& src_outlet, vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy)
{
	if (((int)train_features.size()) == ((int)src_outlet.size()))
	{
		vector<CvPoint> train_points;
		//vector<CvPoint> src_outlet_points;
		vector<CvPoint> features_points;

		//for (int i=0;i<(int)src_outlet.size();i++)
		//{
		//	src_outlet_points.push_back(src_outlet[i].center);
		//}
		CvMemStorage** storage = new CvMemStorage*[(int)src_outlet.size()];
		CvSeq** seq = new CvSeq*[(int)src_outlet.size()];
		CvPoint2D32f elem;

		for (int i=0;i<(int)src_outlet.size();i++)
		{
			storage[i] = cvCreateMemStorage(0); 
			seq[i] = cvCreateSeq( CV_32FC2, /* sequence of integer elements */ // 1st Field - number, 2nd - distance
				sizeof(CvSeq), /* header size - no extra fields */
				sizeof(float)*2, /* element size */
				storage[i] /* the container storage */ );

			for (int j=0;j<(int)features.size();j++)
			{
				if (features[j].class_id == src_outlet[i].class_id)
				{
					float distance = (features[j].pt.x - src_outlet[i].pt.x)*(features[j].pt.x - src_outlet[i].pt.x)+
						(features[j].pt.y - src_outlet[i].pt.y)*(features[j].pt.y - src_outlet[i].pt.y);
					if ( (distance < accuracy*accuracy))
					{
						elem.x = j;
						elem.y = distance;
						cvSeqPush(seq[i],&elem);
					}		
				}

			}
		}

		int total = 1;

		int* seq_indexes = new int[src_outlet.size()];
		int* seq_indexes_min = new int[src_outlet.size()];
		float min_distance = 1e38;
		for (size_t i=0;i<src_outlet.size(); i++)
		{
			seq_indexes[i] = 0;
			seq_indexes_min[i] = -1;
		}

		int counter = 0;

		for (int i=0;i< (int)(src_outlet.size());i++)
		{
			if (seq[i]->total > 0)
				counter++;
		}


		if ((counter > 3) &&(counter > (int)train_features.size()/2))
		{
			bool wasChange = true;
			while (wasChange)
			{
				train_points.clear();
				features_points.clear();
				for (int i=0;i< (int)(src_outlet.size());i++)
				{
					if (seq[i]->total >0)
					{
						elem = *(CvPoint2D32f*)(cvGetSeqElem(seq[i],seq_indexes[i]));
						train_points.push_back(train_features[i].pt);
						features_points.push_back(features[(int)(elem.x)].pt);
					}
				}

				//if (((int)train_points.size() > 3) && ((int)train_points.size() > ((int)train_features.size()/2)))
				//{
				CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
				FindAffineTransform(train_points, features_points, homography);
				reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);
				dst_outlet.clear();
				MapFeaturesAffine(train_features, dst_outlet, homography);



				reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);

				if (reprojectionError < min_distance)
				{
					reprojectionError = min_distance;
					for (int j=0;j<(int)(src_outlet.size());j++)
					{
						seq_indexes_min[j] = seq_indexes[j];
					}
				}

				cvReleaseMat(&homography);


				//}

				// Incrementing seq_indexes;

				for (int i=src_outlet.size()-1; i>=0;i--)
				{
					if (seq[i]->total > 0)
					{
						if (seq_indexes[i] < (seq[i]->total-1))
						{
							seq_indexes[i]++;
							i=0;
							wasChange = true;
						}
						else
						{
							seq_indexes[i] = 0;
							wasChange = false;
						}
					}

				}
			}

			//----------------
			// Add retrieving element from seq with seq_indexes_min_min[] and so on...
			train_points.clear();
			features_points.clear();
			for (int i=0;i< (int)(src_outlet.size());i++)
			{
				if (seq[i]->total >0)
				{
					elem = *(CvPoint2D32f*)(cvGetSeqElem(seq[i],seq_indexes_min[i]));
					train_points.push_back(train_features[i].pt);
					features_points.push_back(features[(int)(elem.x)].pt);
				}
			}

			//if (((int)train_points.size() > 3) && ((int)train_points.size() > ((int)train_features.size()/2)))
			//{
			CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
			FindAffineTransform(train_points, features_points, homography);
			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);
			dst_outlet.clear();
			MapFeaturesAffine(train_features, dst_outlet, homography);



			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);


			cvReleaseMat(&homography);

			for (int i=0;i< (int)(dst_outlet.size());i++)
			{

				//Temp
				int min_index = -1;
				float min_distance = 1e38;
				for (int j=0;j<(int)features.size();j++)
				{
					if (features[j].class_id == dst_outlet[i].class_id)
					{
						float distance = (features[j].pt.x - dst_outlet[i].pt.x)*(features[j].pt.x - dst_outlet[i].pt.x)+
							(features[j].pt.y - dst_outlet[i].pt.y)*(features[j].pt.y - dst_outlet[i].pt.y);
						if ((distance < min_distance) && (distance < accuracy*accuracy/4))
						{
							min_distance = distance;
							min_index = j;
						}		
					}
				}
				if (min_index >= 0)
				{
					dst_outlet[i] = features[min_index];
					//dst_outlet[i].part_id = src_outlet[i].part_id;
				}
				else

					//End

					if (seq[i]->total >0)
					{
						elem = *(CvPoint2D32f*)(cvGetSeqElem(seq[i],seq_indexes_min[i]));
						dst_outlet[i] = features[(int)(elem.x)];
					}
			}			


			//-------------------------------	

		}
		else
		{
			dst_outlet.clear();
			reprojectionError = 1e38;
		}


		delete[] seq_indexes;
		delete[] seq_indexes_min;
		for (int i=0;i<(int)src_outlet.size();i++)
		{
			cvReleaseMemStorage(&(storage[i]));
		}
		delete[] storage;
		delete[] seq;

		//delete[] indexes;
	}
	else
	{
		dst_outlet.clear();
		reprojectionError = 1e38;
	}
}

float generalizedHoughTransform(vector<feature_t>& hole_candidates, const vector<feature_t>& train_features, int* hist_size, float** ranges,vector<outlet_t>& holes, IplImage* ghtImage, IplImage* resImage)
{
	CvSparseMat* hist = buildHoughHist(hole_candidates,train_features,hist_size,ranges);
	float** values = new float*[1];// = getMaxHistValues(hist,hist_size);
	int count = 1;
	int votes = 0;
	votes = getMaxHistValues(hist,hist_size,ranges,values,count);
#if defined(_VERBOSE)
		printf("Votes: %d\n",votes);
#endif
	cvReleaseSparseMat(&hist);

//// Second GHT
//	x_size = test_image->width/11;
//	y_size = test_image->height/11;
//	int hist_size2[] = {x_size, y_size, angle1_size, x_scale_size, y_scale_size, angle2_size};
//	hist = buildHoughHist(hole_candidates,train_features,hist_size2,ranges);
//	float** values2 = new float*[1];// = getMaxHistValues(hist,hist_size);
//	int count2 = 1;
//	int votes2 = 0;
//	votes2 = getMaxHistValues(hist,hist_size2,ranges,values2,count2);
//#if defined(_VERBOSE)
//		printf("Votes2: %d\n",votes2);
//#endif
//	cvReleaseSparseMat(&hist);
//
//	if (votes2 > votes)
//	{
//		for (int i=0;i<count;i++)
//			delete[] values[i];
//		delete[] values;
//		values = new float*[count2];
//		for (int i=0;i<count2;i++)
//		{
//			values[i] = new float[6];
//			for (int j=0;j<6;j++)
//			{
//				values[i][j] = values2[i][j];
//			}
//		}
//		count = count2;
//		votes = votes2;
//	}
//	for (int i=0;i<count2;i++)
//		delete[] values2[i];
//	delete[] values2;
//
//// End
    
	vector<feature_t> hole_features;
	vector<feature_t> hole_features_corrected;
	vector<feature_t> res_features;
  
	float accuracy = sqrt((float)((train_features[1].pt.x -train_features[0].pt.x)*(train_features[1].pt.x -train_features[0].pt.x)+
			(train_features[1].pt.y -train_features[0].pt.y)*(train_features[1].pt.y -train_features[0].pt.y)));
	float error = 1e38;
	int index = -1;
	for (int j=0;j<count;j++)
	{
		float currError;
		hole_features.clear();
		calcOutletPosition(train_features, values[j],hole_features);
if (ghtImage)
{
		for(int i = 0; i < (int)hole_features.size(); i++)
		{
			CvScalar pointColor = hole_features[i].class_id == 0 ? cvScalar(0,255,50) : cvScalar(255,0,50);	
			cvLine(ghtImage, cvPoint(hole_features[i].pt.x+7, hole_features[i].pt.y), cvPoint(hole_features[i].pt.x-7, hole_features[i].pt.y),pointColor,2); 
			cvLine(ghtImage, cvPoint(hole_features[i].pt.x, hole_features[i].pt.y+7), cvPoint(hole_features[i].pt.x, hole_features[i].pt.y-7),pointColor,2); 
            
		}
}


		//accuracy = 21;
		calcExactLocation(hole_candidates,train_features,hole_features,hole_features_corrected,currError,accuracy);
		//calcExactLocation_(hole_candidates,train_features,hole_features,hole_features_corrected,currError,image->width/x_size+1);
		if (currError < error)
		{
			index = j;
			error = currError;
			res_features.clear();
			for (int i =0; i< (int)hole_features_corrected.size(); i++)
				res_features.push_back(hole_features_corrected[i]);
		}
        
	}

	   // int outlet_holes_count = 3;
	if ((int)(res_features.size()) > 0)
	{
		holes.clear();
		outlet_t outlet;



		for (int i=0;i<(int)res_features.size()/3;i++)
		{
			outlet.hole1 = res_features[2*i].pt;
			outlet.hole2 = res_features[2*i+1].pt;
			outlet.ground_hole = res_features[2*(int)res_features.size()/3+i].pt;
			holes.push_back(outlet);
		}

if (resImage)
{
		for(int i = 0; i < (int)res_features.size(); i++)
		{
			
			CvScalar pointColor = res_features[i].class_id == 0 ? cvScalar(0,255,50) : cvScalar(255,0,50);	
			cvLine(resImage, cvPoint(res_features[i].pt.x+7, res_features[i].pt.y), cvPoint(res_features[i].pt.x-7, res_features[i].pt.y),pointColor,2); 
			cvLine(resImage, cvPoint(res_features[i].pt.x, res_features[i].pt.y+7), cvPoint(res_features[i].pt.x, res_features[i].pt.y-7),pointColor,2); 
		}
}

	}
    
	
	for (int i=0;i<count;i++)
		delete[] values[i];
	delete[] values;
    
    return error;
}
