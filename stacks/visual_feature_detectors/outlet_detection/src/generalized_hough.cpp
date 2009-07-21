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
		outlet_center.x += train_features[i].center.x;
		outlet_center.y += train_features[i].center.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	float rel_center_x = - outlet_center.x + train_features[feature_id].center.x; 
	float rel_center_y = - outlet_center.y + train_features[feature_id].center.y; 
	float t1 = rel_center_x * cos(angle1) + rel_center_y*sin(angle1); 
	float t2 = - rel_center_x * sin(angle1) + rel_center_y * cos(angle1); 
	rel_center_x = t1*x_scale;
	rel_center_y = t2*y_scale;
	t1 = rel_center_x * cos(angle2) + rel_center_y*sin(angle2); 
	t2 = - rel_center_x * sin(angle2) + rel_center_y * cos(angle2); 

	result = new CvPoint();
	result->x = feature.center.x-t1;
	result->y = feature.center.y-t2;

	return result;

}
// Builds 6-dimension histogram [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvSparseMat* buildHoughHist(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges)
{
	CvSparseMat* hist;


	/*float x_ranges[] = { 0, imageSize.width }; 
	float y_ranges[] = { 0, imageSize.height };
	float angle1_ranges[] = { -CV_PI/2, CV_PI/2 };
	float angle2_ranges[] = { -CV_PI/2, CV_PI/2 };
	float x_scale_ranges[] = { 0.8, 1.2 };
	float y_scale_ranges[] = { 0.8, 1.2 };
	float* ranges[] ={ x_ranges, y_ranges, angle1_ranges, x_scale_ranges, y_scale_ranges, angle2_ranges};*/
	//hist = cvCreateHist( 6, hist_size, CV_HIST_SPARSE, ranges, 1);
	hist = cvCreateOutletSparseMat(6,hist_size,CV_32FC1);
	int* idx = new int[6];
	//bool wasPower = false;
	//bool wasGround = false;
	for (int n = 0; n < (int)input.size();n++)
	{
		for (int feature_id = 0; /*!(wasPower && wasGround)*/feature_id < (int)train_features.size(); feature_id++)
		{
			if (input[n].part_id != train_features[feature_id].part_id)
				continue;
			//else
			//{
			//	if (input[n].part_id == 0)
			//		wasGround = true;
			//	else
			//		if (input[n].part_id == 1)
			//			wasPower = true;
			//}
			for (float angle1 = ranges[2][0]; angle1 <= ranges[2][1]; angle1 += ((ranges[2][1]-ranges[2][0] > 0) ? (ranges[2][1]-ranges[2][0])/hist_size[2] : 1))
			{
				for (float x_scale = ranges[3][0]; x_scale <= ranges[3][1]; x_scale += ((ranges[3][1]-ranges[3][0] > 0) ? (ranges[3][1]-ranges[3][0])/hist_size[3] : 1))
				{
					for (float y_scale = ranges[4][0]; y_scale <= ranges[4][1]; y_scale += ((ranges[4][1]-ranges[4][0] > 0) ? (ranges[4][1]-ranges[4][0])/hist_size[4] : 1))
					{
						for (float angle2 = ranges[5][0]; angle2 <= ranges[5][1]; angle2 += ((ranges[5][1]-ranges[5][0] > 0) ? (ranges[5][1]-ranges[5][0])/hist_size[5] : 1))
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
								//Comment
								//					int idx2[6];
								//Endof							
								bool isOK = true;
								for (int i=0;i<6; i++)
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
								//Cooment
								//for (int ind=0;ind<243;ind++)
								//{
								//	bool isOK = true;
								//	for (int i=0;i< 6; i++)
								//	{
								//		idx2[i] = idx[i] + (ind % (int)(pow(3.0f,i))) - 1;
								//		if ((idx2[i] >= hist_size[i])||(idx2[i] < 0))
								//		{
								//			isOK = false;
								//			break;
								//		}
								//	}
								//	if (!isOK)
								//		continue;
								//	float* value = cvGetHistValue_nD(hist,idx2);
								//	*value+=1;
								//}
								//Endof
								if (isOK)
								{
									float value = cvGetRealND(hist,idx);
									cvSetRealND(hist,idx,++value);
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
					s.val[0] = (int)sparse;
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
						if (input[n].part_id != train_features[feature_id].part_id)
							continue;

						for (float angle1 = ranges[2][0]; angle1 <= ranges[2][1]; angle1 += ((ranges[2][1]-ranges[2][0] > 0) ? (ranges[2][1]-ranges[2][0])/hist_size[2] : 1))
						{
							for (float x_scale = ranges[3][0]; x_scale <= ranges[3][1]; x_scale += ((ranges[3][1]-ranges[3][0] > 0) ? (ranges[3][1]-ranges[3][0])/hist_size[3] : 1))
							{
								for (float y_scale = ranges[4][0]; y_scale <= ranges[4][1]; y_scale += ((ranges[4][1]-ranges[4][0] > 0) ? (ranges[4][1]-ranges[4][0])/hist_size[4] : 1))
								{
									for (float angle2 = ranges[5][0]; angle2 <= ranges[5][1]; angle2 += ((ranges[5][1]-ranges[5][0] > 0) ? (ranges[5][1]-ranges[5][0])/hist_size[5] : 1))
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
											for (int i=0;i<6; i++)
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
void getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float** &maxs, int& count)
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
	//printf("Votes: %f\n",MIN_VOTES);
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
		outlet_center.x += train_features[i].center.x;
		outlet_center.y += train_features[i].center.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	for (int i=0; i< train_length; i++)
	{
		float rel_center_x = - outlet_center.x + train_features[i].center.x; 
		float rel_center_y = - outlet_center.y + train_features[i].center.y; 
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
		feature.scale = train_features[i].scale;
		feature.center = result_point;
		feature.part_id = train_features[i].part_id;

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
			if (features[j].part_id == outlet[i].part_id)
			{
				float distance = (features[j].center.x - outlet[i].center.x)*(features[j].center.x - outlet[i].center.x)+
					(features[j].center.y - outlet[i].center.y)*(features[j].center.y - outlet[i].center.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].center - outlet[i].center;
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
	int maxindex = 0;
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
		outlet[i].center.x += move.x;
		outlet[i].center.y += move.y;
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
	CvRect outlet_rect = cvRect(outlet[0].center.x,outlet[0].center.y,0,0);
	for (int i=1;i<(int)(outlet.size());i++)
	{
		if (outlet[i].center.x < outlet_rect.x)
		{
			outlet_rect.width += (outlet_rect.x - outlet[i].center.x);
			outlet_rect.x = outlet[i].center.x;
		}

		if (outlet[i].center.x > (outlet_rect.x+outlet_rect.width))
		{
			outlet_rect.width = outlet[i].center.x - outlet_rect.x;
		}

		if (outlet[i].center.y < outlet_rect.y)
		{
			outlet_rect.height += (outlet_rect.y - outlet[i].center.y);
			outlet_rect.y = outlet[i].center.y;
		}

		if (outlet[i].center.y > (outlet_rect.x+outlet_rect.height))
		{
			outlet_rect.height = outlet[i].center.y - outlet_rect.y;
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
					if (outlet[i].part_id == features[j].part_id)
					{
						// Change to normal metrics
						float dist = (outlet[i].center.x + x - features[j].center.x)*(outlet[i].center.x + x - features[j].center.x) + (outlet[i].center.y + y - features[j].center.y)*(outlet[i].center.y + y - features[j].center.y);
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
		outlet[i].center.x +=movement.x;
		outlet[i].center.y +=movement.y;
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
			if (features[j].part_id == outlet[i].part_id)
			{
				float distance = (features[j].center.x - outlet[i].center.x)*(features[j].center.x - outlet[i].center.x)+
					(features[j].center.y - outlet[i].center.y)*(features[j].center.y - outlet[i].center.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].center - outlet[i].center;
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
					float d = (outlet[k].center.x + distance_vectors[i].x - features[j].center.x)*(outlet[k].center.x + distance_vectors[i].x - features[j].center.x)+
						(outlet[k].center.y + distance_vectors[i].y- features[j].center.y)*(outlet[k].center.y + distance_vectors[i].y- features[j].center.y);
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
		outlet[i].center.x +=distance_vectors[maxindex].x;
		outlet[i].center.y +=distance_vectors[maxindex].y;
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
			if (features[j].part_id == outlet[i].part_id)
			{
				float distance = (features[j].center.x - outlet[i].center.x)*(features[j].center.x - outlet[i].center.x)+
					(features[j].center.y - outlet[i].center.y)*(features[j].center.y - outlet[i].center.y);
				if (distance < min_distance)
				{
					min_distance = distance;
					min_index = j;
				}		
			}
			distances[i] = min_distance;
			indexes[i] = min_index;
			distance_vectors[i] = features[min_index].center - outlet[i].center;
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
			outlet[i].center.x += distance_vectors[i].x;
			outlet[i].center.y += distance_vectors[i].y;
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
		//vector<CvPoint> src_outlet_points;
		vector<CvPoint> features_points;
		int* indexes = new int[(int)train_features.size()];

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
			for (int j=0;j<(int)features.size();j++)
			{
				if (features[j].part_id == src_outlet[i].part_id)
				{
					float distance = (features[j].center.x - src_outlet[i].center.x)*(features[j].center.x - src_outlet[i].center.x)+
						(features[j].center.y - src_outlet[i].center.y)*(features[j].center.y - src_outlet[i].center.y);
					if ((distance < min_distance) && (distance < accuracy*accuracy))
					{
						min_distance = distance;
						min_index = j;
					}		
				}

			}
			indexes[i] = min_index;
		}

		for (int i=0;i< (int)(src_outlet.size());i++)
		{
			if (indexes[i] >=0)
			{
				train_points.push_back(train_features[i].center);
				features_points.push_back(features[indexes[i]].center);
			}
		}

		if (((int)train_points.size() > 3) && ((int)train_points.size() > ((int)train_features.size()/2)))
		{
			CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
			FindAffineTransform(train_points, features_points, homography);
			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);
			dst_outlet.clear();
			MapFeaturesAffine(train_features, dst_outlet, homography);

			for (int i=0;i< (int)(dst_outlet.size());i++)
			{

				//Temp
				int min_index = -1;
				float min_distance = 1e38;
				for (int j=0;j<(int)features.size();j++)
				{
					if (features[j].part_id == src_outlet[i].part_id)
					{
						float distance = (features[j].center.x - src_outlet[i].center.x)*(features[j].center.x - src_outlet[i].center.x)+
							(features[j].center.y - src_outlet[i].center.y)*(features[j].center.y - src_outlet[i].center.y);
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

					if (indexes[i] >=0)
					{
						dst_outlet[i] = features[indexes[i]];
					}
			}			

			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography);

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
				if (features[j].part_id == src_outlet[i].part_id)
				{
					float distance = (features[j].center.x - src_outlet[i].center.x)*(features[j].center.x - src_outlet[i].center.x)+
						(features[j].center.y - src_outlet[i].center.y)*(features[j].center.y - src_outlet[i].center.y);
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
		for (int i=0;i<src_outlet.size(); i++)
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
						train_points.push_back(train_features[i].center);
						features_points.push_back(features[(int)(elem.x)].center);
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
					train_points.push_back(train_features[i].center);
					features_points.push_back(features[(int)(elem.x)].center);
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
					if (features[j].part_id == dst_outlet[i].part_id)
					{
						float distance = (features[j].center.x - dst_outlet[i].center.x)*(features[j].center.x - dst_outlet[i].center.x)+
							(features[j].center.y - dst_outlet[i].center.y)*(features[j].center.y - dst_outlet[i].center.y);
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