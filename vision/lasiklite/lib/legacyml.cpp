#include <iostream>
#include "legacyml.h"
#include <stdio.h>

namespace legacy {

#define ptrdiff_t long

void CV_CDECL icvDefaultSplitIdx_C( int compidx, float threshold,
                                    CvMat* idx, CvMat** left, CvMat** right,
                                    void* userdata )
{
    CvMat* trainData = (CvMat*) userdata;
    int i = 0;

    *left = cvCreateMat( 1, trainData->cols, CV_32FC1 );
    assert(CV_IS_MAT_HDR(*left));
    *right = cvCreateMat( 1, trainData->cols, CV_32FC1 );
    (*left)->cols = (*right)->cols = 0;
    if( idx == NULL )
    {
        for( i = 0; i < trainData->cols; i++ )
        {
            if( CV_MAT_ELEM( *trainData, float, compidx, i ) < threshold )
            {
                (*left)->data.fl[(*left)->cols++] = (float) i;
            }
            else
            {
                (*right)->data.fl[(*right)->cols++] = (float) i;
            }
        }
    }
    else
    {
        uchar* idxdata;
        int idxnum;
        int idxstep;
        int index;

        idxdata = idx->data.ptr;
        idxnum = (idx->rows == 1) ? idx->cols : idx->rows;
        idxstep = (idx->rows == 1) ? CV_ELEM_SIZE( idx->type ) : idx->step;
        for( i = 0; i < idxnum; i++ )
        {
            index = (int) *((float*) (idxdata + i * idxstep));
            if( CV_MAT_ELEM( *trainData, float, compidx, index ) < threshold )
            {
                (*left)->data.fl[(*left)->cols++] = (float) index;
            }
            else
            {
                (*right)->data.fl[(*right)->cols++] = (float) index;
            }
        }
    }

    assert(CV_IS_MAT_HDR(*left));
}

void CV_CDECL icvDefaultSplitIdx_R( int compidx, float threshold,
                                    CvMat* idx, CvMat** left, CvMat** right,
                                    void* userdata )
{
    CvMat* trainData = (CvMat*) userdata;
    int i = 0;

    *left = cvCreateMat( 1, trainData->rows, CV_32FC1 );
    assert(CV_IS_MAT_HDR(*left));
    *right = cvCreateMat( 1, trainData->rows, CV_32FC1 );
    (*left)->cols = (*right)->cols = 0;

    if( idx == NULL )
    {
        for( i = 0; i < trainData->rows; i++ )
        {
            if( CV_MAT_ELEM( *trainData, float, i, compidx ) < threshold )
            {
                (*left)->data.fl[(*left)->cols++] = (float) i;
		assert(CV_IS_MAT_HDR(*left));
            }
            else
            {
                (*right)->data.fl[(*right)->cols++] = (float) i;
	    }
        }
    }
    else
    {
        uchar* idxdata;
        int idxnum;
        int idxstep;
        int index;

        idxdata = idx->data.ptr;
        idxnum = (idx->rows == 1) ? idx->cols : idx->rows;
        idxstep = (idx->rows == 1) ? CV_ELEM_SIZE( idx->type ) : idx->step;
        for( i = 0; i < idxnum; i++ )
        {
            index = (int) *((float*) (idxdata + i * idxstep));
            if( CV_MAT_ELEM( *trainData, float, index, compidx ) < threshold )
            {
                (*left)->data.fl[(*left)->cols++] = (float) index;
            }
            else
            {
                (*right)->data.fl[(*right)->cols++] = (float) index;
            }
        }
    }
}

float cvEvalCARTBOOSTClassifier( const CvClassifier* classifier,
                                 const CvMat* sample, CvMat* )
{
    CV_FUNCNAME( "cvEvalCARTBOOSTClassifier" );

    int idx;

    idx = 0;
    __BEGIN__;


    CV_ASSERT( classifier != NULL );
    CV_ASSERT( sample != NULL );
    CV_ASSERT( CV_MAT_TYPE( sample->type ) == CV_32FC1 );
    CV_ASSERT( sample->rows == 1 || sample->cols == 1 );

    if( sample->rows == 1 )
    {
        do
        {
            if( (CV_MAT_ELEM( (*sample), float, 0,
                    ((CvCARTBOOSTClassifier*) classifier)->comp_idx[idx] )) <
                ((CvCARTBOOSTClassifier*) classifier)->threshold[idx] ) 
            {
                idx = ((CvCARTBOOSTClassifier*) classifier)->left[idx];
            }
            else
            {
                idx = ((CvCARTBOOSTClassifier*) classifier)->right[idx];
            }
        } while( idx > 0 );
    }
    else
    {
        do
        {
            if( (CV_MAT_ELEM( (*sample), float,
                    ((CvCARTBOOSTClassifier*) classifier)->comp_idx[idx], 0 )) <
                ((CvCARTBOOSTClassifier*) classifier)->threshold[idx] ) 
            {
                idx = ((CvCARTBOOSTClassifier*) classifier)->left[idx];
            }
            else
            {
                idx = ((CvCARTBOOSTClassifier*) classifier)->right[idx];
            }
        } while( idx > 0 );
    } 

    __END__;

    return ((CvCARTBOOSTClassifier*) classifier)->val[-idx];
}

void cvReleaseCARTBOOSTClassifier( CvStatModel** classifier )
{
    CV_FUNCNAME( "cvReleaseCARTBOOSTClassifier" );

    __BEGIN__;

    /* TODO: add type check */
    
    CV_CALL( cvFree( (void**) classifier ) );
    if( classifier )
        *classifier = NULL;

    __END__;
}


CvCARTBOOSTClassifier* icvAllocCARTBOOSTClassifier( int num_internal_nodes )
{
    CvCARTBOOSTClassifier* ptr = NULL;

    CV_FUNCNAME( "icvAllocCARTBOOSTClassifier" );

    __BEGIN__;
    size_t header_size = 0;

    if( num_internal_nodes < 1 )
        CV_ERROR( CV_StsBadArg, "Number of internal nodes must be positive" );

    header_size = sizeof( *ptr ) + num_internal_nodes *
        (sizeof( *ptr->comp_idx ) + sizeof( *ptr->threshold ) +
         sizeof( *ptr->left ) + sizeof( *ptr->right )) +
        (num_internal_nodes + 1) * sizeof( *ptr->val );

    CV_CALL( ptr = (CvCARTBOOSTClassifier*) cvCreateStatModel( CV_STAT_MODEL_MAGIC_VAL,
        (int)header_size, cvReleaseCARTBOOSTClassifier, cvEvalCARTBOOSTClassifier, NULL ));

    ptr->count = num_internal_nodes;
    
    ptr->comp_idx = (int*) (ptr + 1);
    ptr->threshold = (float*) (ptr->comp_idx + num_internal_nodes);
    ptr->left = (int*) (ptr->threshold + num_internal_nodes);
    ptr->right = (int*) (ptr->left + num_internal_nodes);
    ptr->val = (float*) (ptr->right + num_internal_nodes);
    
    __END__;

    if( cvGetErrStatus() < 0 )
        cvReleaseCARTBOOSTClassifier( (CvStatModel**) &ptr );

    return ptr;
}


typedef struct CvCARTBOOSTNode
{
    CvMat* sampleIdx;
    CvStumpClassifier* stump;
    int parent;
    int leftflag;
    float errdrop;
} CvCARTBOOSTNode;


CvClassifier* cvCreateCARTBOOSTClassifier( CvMat* trainData,
                     int flags,
                     CvMat* trainClasses,
                     CvMat* typeMask,
                      CvMat* missedMeasurementsMask,
                      CvMat* compIdx,
                      CvMat* sampleIdx,
                      CvMat* weights,
                      CvClassifierTrainParams* trainParams )
{
  //std::cerr<<"Enter cvCreateCARTBOOSTClassifier"<<std::endl;

  if (sampleIdx)
    assert(CV_IS_MAT_HDR(sampleIdx));
  //else
    //std::cerr<<"cvCreateCARTBOOSTClassifier: sampleIdx is null"<<std::endl;

  CvCARTBOOSTClassifier* CARTBOOST = NULL;
  
  CV_FUNCNAME( "cvCreateCARTBOOSTClassifier" );
  
  __BEGIN__;
  
  size_t datasize = 0;
  int count = 0;
  int i = 0;
  int j = 0;
  
  CvCARTBOOSTNode* intnode = NULL;
  CvCARTBOOSTNode* list = NULL;
  int listcount = 0;
  CvMat* lidx = NULL;
  CvMat* ridx = NULL;
  
  float maxerrdrop = 0.0F;
  int idx = 0;
  
  void (*splitIdxCallback)( int compidx, float threshold,
			    CvMat* idx, CvMat** left, CvMat** right,
			    void* userdata );
  void* userdata;
  
  count = ((CvCARTBOOSTTrainParams*) trainParams)->count;
    
  assert( count > 0 );
  
  //std::cerr<<"1A"<<std::endl;
  CV_CALL( CARTBOOST = icvAllocCARTBOOSTClassifier( count ) );
  //std::cerr<<"1B"<<std::endl;

    datasize = sizeof( CvCARTBOOSTNode ) * (count + count);
    intnode = (CvCARTBOOSTNode*) cvAlloc( datasize );
    memset( intnode, 0, datasize );
    list = (CvCARTBOOSTNode*) (intnode + count);

    splitIdxCallback = ((CvCARTBOOSTTrainParams*) trainParams)->splitIdx;
    userdata = ((CvCARTBOOSTTrainParams*) trainParams)->userdata;
    if( splitIdxCallback == NULL )
    {
        splitIdxCallback = ( CV_IS_ROW_SAMPLE( flags ) )
            ? icvDefaultSplitIdx_R : icvDefaultSplitIdx_C;
        userdata = trainData;
    }

    /* create root of the tree */
    //if (sampleIdx)
    //assert(CV_IS_MAT_HDR(sampleIdx));
    //else
    //std::cerr<<"Null 1"<<std::endl;
      
    intnode[0].sampleIdx = sampleIdx;
    intnode[0].stump = (CvStumpClassifier*)
        ((CvCARTBOOSTTrainParams*) trainParams)->stumpConstructor( trainData, flags,
            trainClasses, typeMask, missedMeasurementsMask, compIdx, sampleIdx, weights,
            ((CvCARTBOOSTTrainParams*) trainParams)->stumpTrainParams );
    CARTBOOST->left[0] = CARTBOOST->right[0] = 0;

    /* build tree */
    listcount = 0;
    for( i = 1; i < count; i++ )
    {
        /* split last added node */
        splitIdxCallback( intnode[i-1].stump->compidx, intnode[i-1].stump->threshold,
			  intnode[i-1].sampleIdx, &lidx, &ridx, userdata );
        
        if( intnode[i-1].stump->lerror != 0.0F )
        {
            list[listcount].sampleIdx = lidx;
            list[listcount].stump = (CvStumpClassifier*)
	      ((CvCARTBOOSTTrainParams*) trainParams)->stumpConstructor( trainData, flags,
	trainClasses, typeMask, missedMeasurementsMask, compIdx,
                    list[listcount].sampleIdx,
                    weights, ((CvCARTBOOSTTrainParams*) trainParams)->stumpTrainParams );

            list[listcount].errdrop = intnode[i-1].stump->lerror
                - (list[listcount].stump->lerror + list[listcount].stump->rerror);
            list[listcount].leftflag = 1;
            list[listcount].parent = i-1;
            listcount++;
        }
        else
        {
            cvReleaseMat( &lidx );
        }
        if( intnode[i-1].stump->rerror != 0.0F )
        {
            list[listcount].sampleIdx = ridx;

            list[listcount].stump = (CvStumpClassifier*)
                ((CvCARTBOOSTTrainParams*) trainParams)->stumpConstructor( trainData, flags,
                    trainClasses, typeMask, missedMeasurementsMask, compIdx,
                    list[listcount].sampleIdx,
                    weights, ((CvCARTBOOSTTrainParams*) trainParams)->stumpTrainParams );
            list[listcount].errdrop = intnode[i-1].stump->rerror
                - (list[listcount].stump->lerror + list[listcount].stump->rerror);
            list[listcount].leftflag = 0;
            list[listcount].parent = i-1;
            listcount++;
        }
        else
        {
            cvReleaseMat( &ridx );
        }
        
        if( listcount == 0 ) break;

        /* find the best node to be added to the tree */
        idx = 0;
        maxerrdrop = list[idx].errdrop;
        for( j = 1; j < listcount; j++ )
        {
            if( list[j].errdrop > maxerrdrop )
            {
                idx = j;
                maxerrdrop = list[j].errdrop;
            }
        }
        intnode[i] = list[idx];
        if( list[idx].leftflag )
        {
            CARTBOOST->left[list[idx].parent] = i;
        }
        else
        {
            CARTBOOST->right[list[idx].parent] = i;
        }
        if( idx != (listcount - 1) )
        {
            list[idx] = list[listcount - 1];
        }
        listcount--;
    }

    /* fill <CARTBOOST> fields */
    j = 0;
    CARTBOOST->count = 0;
    for( i = 0; i < count && (intnode[i].stump != NULL); i++ )
    {
        CARTBOOST->count++;
        CARTBOOST->comp_idx[i] = intnode[i].stump->compidx;
        CARTBOOST->threshold[i] = intnode[i].stump->threshold;
        
        /* leaves */
        if( CARTBOOST->left[i] <= 0 )
        {
            CARTBOOST->left[i] = -j;
            CARTBOOST->val[j] = intnode[i].stump->left;
            j++;
        }
        if( CARTBOOST->right[i] <= 0 )
        {
            CARTBOOST->right[i] = -j;
            CARTBOOST->val[j] = intnode[i].stump->right;
            j++;
        }
    }
    
    /* CLEAN UP */
    for( i = 0; i < count && (intnode[i].stump != NULL); i++ )
    {
        intnode[i].stump->release( (CvClassifier**) &(intnode[i].stump) );
        if( i != 0 )
        {	 	 
	  //<HACK>
	  //Sometimes the split function fails to find anything below
	  //threshold, and we end up with sampleIdx having zero cols.
	  //cvReleaseMat then thinks it is not a valid matrix and blows up.
	  //This is a fix for that, but may obscure an underlying problem
	  //if the threshold case is not meant to occur
	  //intnode[i].sampleIdx->cols = 1;
	  //</HACK>

	  cvReleaseMat( &(intnode[i].sampleIdx) );
        }
    }
    for( i = 0; i < listcount; i++ )
    {
        list[i].stump->release( (CvClassifier**) &(list[i].stump) );
        cvReleaseMat( &(list[i].sampleIdx) );
    }
    
    cvFree( (void**) &intnode );

    __END__;

    return (CvClassifier*) CARTBOOST;
}



#define CV_DECLARE_QSORT( func_name, T, less_than )                     \
void func_name( T* array, size_t length, int aux );

#define less_than( a, b ) ((a) < (b))

CV_DECLARE_QSORT( icvSort_32f, float, less_than )

CV_IMPLEMENT_QSORT( icvSort_32f, float, less_than )

typedef struct CvValArray
{
    uchar* data;
    int    step;
} CvValArray;

#define CMP_VALUES( idx1, idx2 )                                 \
    ( *( (float*) (aux->data + ((int) (idx1)) * aux->step ) ) <  \
      *( (float*) (aux->data + ((int) (idx2)) * aux->step ) ) )

CV_IMPLEMENT_QSORT_EX( icvSortIndexedValArray_16s, short, CMP_VALUES, CvValArray* )

CV_IMPLEMENT_QSORT_EX( icvSortIndexedValArray_32s, int,   CMP_VALUES, CvValArray* )

CV_IMPLEMENT_QSORT_EX( icvSortIndexedValArray_32f, float, CMP_VALUES, CvValArray* )


int icvGetIdxAt( CvMat* idx, int pos )
{
    if( idx == NULL )
    {
        return pos;
    }
    else
    {
        CvScalar sc;
        int type;

        type = CV_MAT_TYPE( idx->type );
        cvRawDataToScalar( idx->data.ptr + pos *
            ( (idx->rows == 1) ? CV_ELEM_SIZE( type ) : idx->step ), type, &sc );

        return (int) sc.val[0];
    }
}


#define CV_MAT2VEC( mat, vdata, vstep, num )       \
    assert( (mat).rows == 1 || (mat).cols == 1 );  \
    (vdata) = ((mat).data.ptr);                    \
    if( (mat).rows == 1 )                          \
    {                                              \
        (vstep) = CV_ELEM_SIZE( (mat).type );      \
        (num) = (mat).cols;                        \
    }                                              \
    else                                           \
    {                                              \
        (vstep) = (mat).step;                      \
        (num) = (mat).rows;                        \
    }


#define CV_LB_PROB_THRESH      0.01F
#define CV_LB_WEIGHT_THRESHOLD 0.0001F

CvMat* cvTrimWeights( CvMat* weights, CvMat* idx, float factor )
{
    CvMat* ptr;

    CV_FUNCNAME( "cvTrimWeights" );
    
    ptr = NULL;
    __BEGIN__;
    int i, index, num;
    float sum_weights;
    uchar* wdata;
    int wstep;
    int wnum;
    float threshold;
    int count;
    float* sorted_weights;

    CV_ASSERT( CV_MAT_TYPE( weights->type ) == CV_32FC1 );

    ptr = idx;
    sorted_weights = NULL;

    if( factor > 0.0F && factor < 1.0F )
    {
        size_t data_size;

        CV_MAT2VEC( *weights, wdata, wstep, wnum );
        num = ( idx == NULL ) ? wnum : MAX( idx->rows, idx->cols );

        data_size = num * sizeof( *sorted_weights );
        sorted_weights = (float*) cvAlloc( data_size );
        memset( sorted_weights, 0, data_size );

        sum_weights = 0.0F;
        for( i = 0; i < num; i++ )
        {
            index = icvGetIdxAt( idx, i );
            sorted_weights[i] = *((float*) (wdata + index * wstep));
            sum_weights += sorted_weights[i];
        }

        icvSort_32f( sorted_weights, num, 0 );

        sum_weights *= (1.0F - factor);

        i = -1;
        do { sum_weights -= sorted_weights[++i]; }
        while( sum_weights > 0.0F && i < (num - 1) );

        threshold = sorted_weights[i];

        while( i > 0 && sorted_weights[i-1] == threshold ) i--;

        if( i > 0 )
        {
            CV_CALL( ptr = cvCreateMat( 1, num - i, CV_32FC1 ) );
            count = 0;
            for( i = 0; i < num; i++ )
            {
                index = icvGetIdxAt( idx, i );
                if( *((float*) (wdata + index * wstep)) >= threshold )
                {
                    CV_MAT_ELEM( *ptr, float, 0, count ) = (float) index;
                    count++;
                }
            }
        
            assert( count == ptr->cols );
        }
        cvFree( (void**) &sorted_weights );
    }

    __END__;

    return ptr;
}


void
icvResponsesAndWeightsLB( CvMat* responses, CvMat* weak_train_resp, CvMat* weights,
                          CvMat* sum_resp )
{
    //CV_FUNCNAME( "icvResponsesAndWeightsLB" );

    __BEGIN__;

    int i;

    for( i = 0; i < responses->cols; ++i )
    {
        float p = 1.0F / (1.0F + expf( -2.0F * CV_MAT_ELEM( *sum_resp, float, 0, i ) ));
        CV_MAT_ELEM( *weights, float, 0, i ) = 
            MAX( p * (1.0F - p), CV_LB_WEIGHT_THRESHOLD );
        if( CV_MAT_ELEM( *responses, int, 0, i ) == 1 )
        {
            CV_MAT_ELEM( *weak_train_resp, float, 0, i ) = 
                1.0F / (MAX( p, CV_LB_PROB_THRESH ));
        }
        else
        {
            CV_MAT_ELEM( *weak_train_resp, float, 0, i ) = 
                -1.0F / (MAX( 1.0F - p, CV_LB_PROB_THRESH ));
        }
    }
    __END__;
}


#define CV_LOGRATIO_THRESHOLD 0.00001F

/* log( val / (1 - val ) ) */
CV_INLINE float cvLogRatio( float val );

CV_INLINE float cvLogRatio( float val )
{
    float tval;

    tval = MAX(CV_LOGRATIO_THRESHOLD, MIN( 1.0F - CV_LOGRATIO_THRESHOLD, (val) ));
    return logf( tval / (1.0F - tval) );
}


typedef void (*CvBoostStartTraining)( CvMat* responses,
                                      CvMat* weak_train_resp,
                                      CvMat* weights,
                                      CvMat* sum_resp );

typedef double (*CvBoostNextWeakClassifier)( CvMat* weak_eval,
                                             CvMat* responses,
                                             CvMat* weak_train_resp,
                                             CvMat* weights,
                                             CvMat* sum_resp );

void
icvBoostStartTraining( CvMat* responses, CvMat* weak_train_resp, CvMat* /* weights */,
                       CvMat* /* sum_resp */ )
{
    CV_FUNCNAME( "icvBoostStartTraining" );
    __BEGIN__;
    CV_CALL( cvConvertScale( responses, weak_train_resp, 2, -1 ) );
    __END__;
}

double
icvBoostNextWeakClassifierDAB( CvMat* weak_eval, CvMat* responses,
                               CvMat* /* weak_train_resp */, CvMat* weights,
                               CvMat* /* sum_resp */ )
{
    double c = 0;
    //CV_FUNCNAME( "icvBoostNextWeakClassifierDAB" );

    __BEGIN__;
    float sumw;
    float err;
    int i;

    sumw = 0.0F;
    err = 0.0F;
    for( i = 0; i < responses->cols; ++i )
    {
        sumw += CV_MAT_ELEM( *weights, float, 0, i );
        err += CV_MAT_ELEM( *weights, float, 0, i ) *
            ( CV_MAT_ELEM( *weak_eval, float, 0, i ) !=
              (2.0F * CV_MAT_ELEM( *responses, int, 0, i ) - 1.0F) );
    }
    if( sumw != 0 ) err /= sumw;
    err = -cvLogRatio( err );
    
    sumw = 0;
    for( i = 0; i < responses->cols; ++i )
    {
        CV_MAT_ELEM( *weights, float, 0, i ) *= expf( err *
            ( CV_MAT_ELEM( *weak_eval, float, 0, i ) !=
              (2.0F * CV_MAT_ELEM( *responses, int, 0, i ) - 1.0F) ) );

        sumw += CV_MAT_ELEM( *weights, float, 0, i );
    }
    if( sumw != 0 )
        for( i = 0; i < weights->cols; ++i )
            CV_MAT_ELEM( *weights, float, 0, i ) /= sumw;

    c = err;
    __END__;
    
    return c;
}

double
icvBoostNextWeakClassifierRAB( CvMat* weak_eval, CvMat* responses,
                               CvMat* /* weak_train_resp */, CvMat* weights,
                               CvMat* /* sum_resp */ )
{
    double c = 0.5;
    //CV_FUNCNAME( "icvBoostNextWeakClassifierRAB" );

    __BEGIN__;
    float sumw;
    int i;

    sumw = 0;
    for( i = 0; i < responses->cols; ++i )
    {
        CV_MAT_ELEM( *weights, float, 0, i ) *= expf( 
            -(CV_MAT_ELEM( *responses, int, 0, i ) - 0.5F) *
             CV_MAT_ELEM( *weak_eval, float, 0, i ) );

        sumw += CV_MAT_ELEM( *weights, float, 0, i );
    }
    if( sumw != 0 )
        for( i = 0; i < weights->cols; ++i )
            CV_MAT_ELEM( *weights, float, 0, i ) /= sumw;
    __END__;
    
    return c;
}

void
icvBoostStartTrainingLB( CvMat* responses, CvMat* weak_train_resp, CvMat* weights,
                         CvMat* sum_resp )
{
    CV_FUNCNAME( "icvBoostStartTraining" );
    __BEGIN__;
    /* CV_CALL( cvSet( sum_resp, cvScalar( 0 ) ) ); */
    CV_CALL( icvResponsesAndWeightsLB( responses, weak_train_resp, weights, sum_resp ));
    __END__;
}

double
icvBoostNextWeakClassifierLB( CvMat* weak_eval, CvMat* responses, CvMat* weak_train_resp,
                              CvMat* weights, CvMat* sum_resp )
{
    double c = 0.5;
    CV_FUNCNAME( "icvBoostNextWeakClassifierLB" );
    __BEGIN__;
    int i;
    for( i = 0; i < sum_resp->cols; ++i )
        CV_MAT_ELEM( *sum_resp, float, 0, i ) +=
            0.5F * CV_MAT_ELEM( *weak_eval, float, 0, i );
    CV_CALL( icvResponsesAndWeightsLB( responses, weak_train_resp, weights, sum_resp ));
    __END__;
    return c;
}

double
icvBoostNextWeakClassifierGAB( CvMat* weak_eval, CvMat* responses,
                               CvMat* /* weak_train_resp */, CvMat* weights,
                               CvMat* /* sum_resp */ )
{
    double c = 1;
    //CV_FUNCNAME( "icvBoostNextWeakClassifierGAB" );
    __BEGIN__;
    float sumw;
    int i;

    sumw = 0;
    for( i = 0; i < responses->cols; ++i )
    {
        CV_MAT_ELEM( *weights, float, 0, i ) *= expf( 
            -(2.0F * CV_MAT_ELEM( *responses, int, 0, i ) - 1.0F) *
             CV_MAT_ELEM( *weak_eval, float, 0, i ) );

        sumw += CV_MAT_ELEM( *weights, float, 0, i );
    }
    if( sumw != 0 )
        for( i = 0; i < weights->cols; ++i )
            CV_MAT_ELEM( *weights, float, 0, i ) /= sumw;
    __END__;
    return c;
}


CvBoostStartTraining startTraining[] = {
        icvBoostStartTraining,
        icvBoostStartTraining,
        icvBoostStartTrainingLB,
        icvBoostStartTraining
    };

CvBoostNextWeakClassifier nextWeakClassifier[] = {
        icvBoostNextWeakClassifierDAB,
        icvBoostNextWeakClassifierRAB,
        icvBoostNextWeakClassifierLB,
        icvBoostNextWeakClassifierGAB
    };


static int
icvCmpIntegers( const void* a, const void* b )
{
    return *(const int*)a - *(const int*)b;
}

static int
icvCmpIntegersPtr( const void* _a, const void* _b )
{
    int a = **(const int**)_a;
    int b = **(const int**)_b;
    return (a < b ? -1 : 0)|(a > b);
}

int icvCmpSparseVecElems( const void* a, const void* b )
{
    return ((CvSparseVecElem32f*)a)->idx - ((CvSparseVecElem32f*)b)->idx;
}

	int
icvEvalCARTBOOSTCol( CvCARTBOOSTClassifier* tree, const float* sample, ptrdiff_t step )
{
    assert( tree != NULL );
    assert( sample != NULL );

    int idx = 0;

    do
    {
        if( *((float*) ((char*) sample + step * tree->comp_idx[idx])) <
                tree->threshold[idx] )
            idx = tree->left[idx];
        else
            idx = tree->right[idx];
    } while( idx > 0 );

    return -idx;
}

	double
icvBoostIterate( CvMat* weak_eval, CvMat* responses, CvMat* weak_train_resp,
                 CvMat* weights, int type, CvMat* sum_resp )
{
    double c = 1;

    //CV_FUNCNAME( "icvBoostIterate" );
    
    __BEGIN__;

    /* the function is internal so asserts are used for input arguments checks */
   
    assert( CV_IS_MAT( responses ) );
    assert( CV_MAT_TYPE( responses->type ) == CV_32SC1 );
    assert( responses->rows == 1 );

    assert( !weak_eval || CV_IS_MAT( weak_eval ) );
    assert( !weak_eval || CV_MAT_TYPE( weak_eval->type ) == CV_32FC1 );
    assert( !weak_eval || weak_eval->rows == 1 );
    assert( !weak_eval || weak_eval->cols == responses->cols );
    
    assert( CV_IS_MAT( weak_train_resp ) );
    assert( CV_MAT_TYPE( weak_train_resp->type ) == CV_32FC1 );
    assert( weak_train_resp->rows == 1 );
    assert( weak_train_resp->cols == responses->cols );
    
    assert( CV_IS_MAT( weights ) );
    assert( CV_MAT_TYPE( weights->type ) == CV_32FC1 );
    assert( weights->rows == 1 );
    assert( weights->cols == responses->cols );

    assert( type >= CV_BT_DISCRETE );
    assert( type <= CV_BT_GENTLE );

    assert( CV_IS_MAT( weights ) );
    assert( CV_MAT_TYPE( weights->type ) == CV_32FC1 );
    assert( weights->rows == 1 );
    assert( weights->cols == responses->cols );

    assert( type != CV_BT_LOGIT || sum_resp );
    assert( !sum_resp || CV_IS_MAT( sum_resp ) );
    assert( !sum_resp || CV_MAT_TYPE( sum_resp->type ) == CV_32FC1 );
    assert( !sum_resp || sum_resp->rows == 1 );
    assert( !sum_resp || sum_resp->cols == responses->cols );

    if( !weak_eval )
    {
        /* start training */
        startTraining[type]( responses, weak_train_resp, weights, sum_resp );
    }
    else
    {
        c = nextWeakClassifier[type]( weak_eval, responses, weak_train_resp, weights, 
            sum_resp );
    }

    __END__;

    return c;
}






int
cvPrepareTrainData( const char* /*funcname*/,
                    int flags,
                    const CvArr* _train_data, int tflag,
                    const CvMat* responses,
                    const CvMat* comp_idx,
                    const CvMat* sample_idx,
                    const CvMat* type_mask,
                    const CvMat* missval_mask,
                    const float*** _out_train_data,
                    int* allocated_train_data,
                    int* sample_count,
                    int* _dims,
                    int* _dims_all,
                    CvMat** out_responses,
                    CvMat** out_response_map,
                    CvMat** out_comp_idx,
                    CvMat** _out_sample_idx,
                    CvMat** out_type_mask,
                    const uchar*** _out_missval_mask,
                    int* _is_sparse )
{
    int ok = 0;
    int** temp_response_ptr = 0;
    float** out_train_data = 0;
    uchar** out_missval_mask = 0;
    int* inverse_map0 = 0;
    int* inverse_map1 = 0;
    int* counters = 0;
    int copy_train_data = 0;
    CvMat* out_sample_idx = 0;

    CV_FUNCNAME( "cvPrepareTrainData" );

    // step 0. clear all the output pointers to ensure we do not try
    // to call free() with uninitialized pointers
    if( out_responses )
        *out_responses = 0;

    if( out_response_map )
        *out_response_map = 0;

    if( out_comp_idx )
        *out_comp_idx = 0;

    if( out_type_mask )
        *out_type_mask = 0;

    if( _out_missval_mask )
        *_out_missval_mask = 0;

    if( _out_train_data )
        *_out_train_data = 0;

    if( allocated_train_data )
        *allocated_train_data = 0;

    if( sample_count )
        *sample_count = 0;

    if( _dims )
        *_dims = 0;

    if( _dims_all )
        *_dims_all = 0;

    if( _is_sparse )
        *_is_sparse = 0;
    
    __BEGIN__;

    int is_categorical_response = -1;
    int samples_all = 0, samples_selected = 0;
    int dims_all = 0, dims_selected = 0;
    int c_type = -1;
    int i, j;
    int is_sparse = CV_IS_SPARSE_MAT(_train_data);
    CvMat* train_data = (CvMat*)_train_data;
    int transpose_train_data = 0;
    int defragment_train_data = 0;
    int out_train_data_rows = 0;
    int out_train_data_cols = 0;
    int* row_map = 0;
    int* col_map = 0;
    float* dst_data = 0;
    const float* src_data = 0;
    int src_step = 0;
    uchar* dst_mask = 0;
    const uchar* src_mask = 0;
    int src_mask_step = 0;
    int nd_dims, nd_size[CV_MAX_DIM];
    int train_data_type;

    copy_train_data = is_sparse ||
        (flags & CV_TRAIN_STATMODEL_ALWAYS_COPY_TRAIN_DATA) != 0;

    tflag = tflag != CV_ROW_SAMPLE; // make it 0 or 1.

    CV_CALL( nd_dims = cvGetDims( train_data, nd_size )); 
    CV_CALL( train_data_type = cvGetElemType( train_data ));

    // check training data
    if( nd_dims != 2 )
        CV_ERROR( CV_StsBadSize, "training data must be a dense or sparse matrix" );

    if( !_out_train_data || !sample_count || !_dims || !_dims_all )
        CV_ERROR( CV_StsNullPtr,
        "INTERNAL ERROR: some of out_train_data, sample_count, dims "
        "and dims_all pointers are NULL" );

    if( train_data_type != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat, "Only 32FC1 type of training data is supported" );

    if( !tflag )
    {
        samples_all = samples_selected = nd_size[0];
        dims_all = dims_selected = nd_size[1];
    }
    else
    {
        samples_all = samples_selected = nd_size[1];
        dims_all = dims_selected = nd_size[0];
    }

    // check and preprocess component indices
    if( comp_idx )
    {
        int c_len, c_step;
        
        if( !CV_IS_MAT(comp_idx) )
            CV_ERROR( CV_StsBadArg, "Invalid comp_idx array" );

        if( comp_idx->rows != 1 && comp_idx->cols != 1 )
            CV_ERROR( CV_StsBadSize, "comp_idx array must be 1-dimensional" );

        if( !out_comp_idx )
            CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: output out_comp_idx pointer must be passed" );

        c_len = comp_idx->rows + comp_idx->cols - 1;
        c_step = comp_idx->step ? comp_idx->step/CV_ELEM_SIZE(comp_idx->type) : 1;

        c_type = CV_MAT_TYPE(comp_idx->type);

        switch( c_type )
        {
        case CV_8UC1:
        case CV_8SC1:
            {
            uchar* c_data = comp_idx->data.ptr;
                
            // comp_idx is array of 1's and 0's -
            // i.e. it is a mask of the selected components
            if( c_len != dims_all )
                CV_ERROR( CV_StsUnmatchedSizes,
                "Component mask should contain as many elements as the total number of input variables" );
            
            dims_selected = 0;
            for( i = 0; i < c_len; i++ )
                dims_selected += c_data[i*c_step] != 0;

            if( dims_selected == 0 )
                CV_ERROR( CV_StsOutOfRange, "No components/input_variables is selected!" );

            if( dims_selected == dims_all )
                c_type = -1, comp_idx = 0;
            }
            break;
        case CV_32SC1:
            // comp_idx is array of indices of the selected components
            if( c_len > dims_all )
                CV_ERROR( CV_StsOutOfRange,
                "comp_idx array may not contain more elements than the total number of input variables" );
            dims_selected = c_len;
            break;
        default:
            CV_ERROR( CV_StsUnsupportedFormat, "Unsupported comp_idx array data type "
                                               "(it should be 8uC1, 8sC1 or 32sC1)" );
        }

        if( c_type >= 0 )
        {
            int* out_c_data; 
            CV_CALL( *out_comp_idx = cvCreateMat( 1, dims_selected, CV_32SC1 ));
            out_c_data = (*out_comp_idx)->data.i;

            if( c_type < CV_32SC1 )
            {
                uchar* c_data = comp_idx->data.ptr;
                for( i = 0; i < c_len; i++ )
                    if( c_data[i*c_step] )
                        *out_c_data++ = i;
                out_c_data -= dims_selected;
            }
            else
            {
                int* c_data = comp_idx->data.i;
                int out_of_order = 0;

                for( i = 0; i < c_len; i++ )
                {
                    out_c_data[i] = c_data[i*c_step];
                    if( i > 0 && out_c_data[i] < out_c_data[i-1] )
                        out_of_order = 1;
                }

                if( out_of_order )
                    qsort( out_c_data, c_len, sizeof(out_c_data[0]), icvCmpIntegers );
                
                for( i = 0; i < c_len; i++ )
                    if( (unsigned)out_c_data[i] >= (unsigned)dims_all ||
                        (i > 0 && out_c_data[i] <= out_c_data[i-1]))
                        CV_ERROR( CV_StsBadArg,
                        "There are duplicated or out-of-range component indices"
                        /*"There component indices are not ordered, "
                        "or there are duplicated or out-of-range indices"*/ );
            }
        }
    }

    // check and preprocess sample indices
    if( sample_idx )
    {
        int s_len, s_step;
        int s_type = 0;
        int* out_s_data = 0;
        
        if( !CV_IS_MAT(sample_idx) )
            CV_ERROR( CV_StsBadArg, "Invalid sample_idx array" );

        if( !_out_sample_idx && !(flags & CV_TRAIN_STATMODEL_DEFRAGMENT_TRAIN_DATA) )
            CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: output out_sample_idx pointer must be passed" );

        if( sample_idx->rows != 1 && sample_idx->cols != 1 )
            CV_ERROR( CV_StsBadSize, "sample_idx array must be 1-dimensional" );

        s_len = sample_idx->rows + sample_idx->cols - 1;
        s_step = sample_idx->step ? sample_idx->step/CV_ELEM_SIZE(sample_idx->type) : 1;

        s_type = CV_MAT_TYPE(sample_idx->type);

        switch( s_type )
        {
        case CV_8UC1:
        case CV_8SC1:
            {
            uchar* s_data = sample_idx->data.ptr;
                
            // sample_idx is array of 1's and 0's -
            // i.e. it is a mask of the selected samples
            if( s_len != samples_all )
                CV_ERROR( CV_StsUnmatchedSizes,
                "Sample mask should contain as many elements as the total number of samples" );
            
            samples_selected = 0;
            for( i = 0; i < s_len; i++ )
                samples_selected += s_data[i*s_step] != 0;

            if( samples_selected == 0 )
                CV_ERROR( CV_StsOutOfRange, "No samples is selected!" );
            }
            break;
        case CV_32SC1:
            if( s_len > samples_all )
                CV_ERROR( CV_StsOutOfRange,
                "sample_idx array may not contain more elements than the total number of samples" );
            samples_selected = s_len;
            break;
        default:
            CV_ERROR( CV_StsUnsupportedFormat, "Unsupported sample_idx array data type "
                                               "(it should be 8uC1, 8sC1 or 32sC1)" );
        }

        CV_CALL( out_sample_idx = cvCreateMat( 1, samples_selected, CV_32SC1 ));
        out_s_data = out_sample_idx->data.i;

        if( s_type < CV_32SC1 )
        {
            uchar* s_data = sample_idx->data.ptr;
            for( i = 0; i < s_len; i++ )
                if( s_data[i*s_step] )
                    *out_s_data++ = i;
            out_s_data -= samples_selected;
        }
        else
        {
            int* s_data = sample_idx->data.i;
            int out_of_order = 0;

            for( i = 0; i < s_len; i++ )
            {
                out_s_data[i] = s_data[i*s_step];
                if( i > 0 && out_s_data[i] < out_s_data[i-1] )
                    out_of_order = 1;
            }

            if( out_of_order )
                qsort( out_s_data, s_len, sizeof(out_s_data[0]), icvCmpIntegers );
            
            for( i = 0; i < s_len; i++ )
                if( (unsigned)out_s_data[i] >= (unsigned)samples_all ||
                    (i > 0 && out_s_data[i] <= out_s_data[i-1]))
                    CV_ERROR( CV_StsBadArg,
                    "There are duplicated or out-of-range sample indices"
                    /*"There component indices are not ordered, "
                    "or there are duplicated or out-of-range indices"*/ );
        }
    }

    // check and preprocess type mask
    if( type_mask )
    {
        int t_step;
        int* map = out_comp_idx && *out_comp_idx ? map = (*out_comp_idx)->data.i : 0;
        
        if( !CV_IS_MAT(type_mask) )
            CV_ERROR( type_mask ? CV_StsBadArg : CV_StsNullPtr, "Invalid type_mask array" );

        if( type_mask->rows != 1 && type_mask->cols != 1 )
            CV_ERROR( CV_StsBadSize, "type_mask array must be 1-dimensional" );

        if( !CV_IS_MASK_ARR(type_mask))
            CV_ERROR( CV_StsUnsupportedFormat, "type mask must be 8uC1 or 8sC1 array" );

        if( (unsigned)(type_mask->rows + type_mask->cols - 1 - dims_all) > 1 )
            CV_ERROR( CV_StsUnmatchedSizes,
            "type mask array must contain as many elements as the total number of input [& output] variables" );

        t_step = type_mask->step ? type_mask->step/CV_ELEM_SIZE(type_mask->type) : 1;

        if( !out_type_mask )
            CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: output out_type_mask pointer must be passed" );

        CV_CALL( *out_type_mask = cvCreateMat( 1, dims_selected + 1, CV_8UC1 ));

        src_mask = type_mask->data.ptr;
        dst_mask = (*out_type_mask)->data.ptr;

        for( i = 0; i < dims_selected; i++ )
        {
            int idx = map ? map[i] : i;
            assert( (unsigned)idx < (unsigned)dims_all );
            dst_mask[i] = (uchar)(src_mask[idx*t_step] != 0);
        }
        
        if( type_mask->rows + type_mask->cols - 1 > dims_all )
            is_categorical_response = src_mask[dims_all*t_step] != 0;
    }

    // check and preprocess response
    if( responses )
    {
        int r_type, r_step;
        int is_categorical_response2 = -1;
        
        if( !CV_IS_MAT(responses) )
            CV_ERROR( CV_StsBadArg, "Invalid response array" );

        if( responses->rows != 1 && responses->cols != 1 )
            CV_ERROR( CV_StsBadSize, "Response array must be 1-dimensional" );

        if( responses->rows + responses->cols - 1 != samples_all )
            CV_ERROR( CV_StsUnmatchedSizes,
            "Response array must contain as many elements as the total number of samples" );

        r_type = CV_MAT_TYPE(responses->type);
        if( r_type != CV_32FC1 && r_type != CV_32SC1 )
            CV_ERROR( CV_StsUnsupportedFormat, "Unsupported response type" );

        r_step = responses->step ? responses->step / CV_ELEM_SIZE(responses->type) : 1;

        switch(flags & (CV_TRAIN_STATMODEL_CATEGORICAL_RESPONSE|
                        CV_TRAIN_STATMODEL_ORDERED_RESPONSE))
        {
        case CV_TRAIN_STATMODEL_CATEGORICAL_RESPONSE:
            is_categorical_response2 = 1;
            break;
        case CV_TRAIN_STATMODEL_ORDERED_RESPONSE:
            is_categorical_response2 = 0;
            break;
        case CV_TRAIN_STATMODEL_CATEGORICAL_RESPONSE|
             CV_TRAIN_STATMODEL_ORDERED_RESPONSE:
            is_categorical_response2 = is_categorical_response;
            break;
        default:
            CV_ERROR( CV_StsBadArg, "INTERNAL ERROR: At least one of possible response type must be specified" );
        }

        if( is_categorical_response < 0 )
            is_categorical_response = is_categorical_response2;

        if( is_categorical_response < 0 ||
            is_categorical_response != is_categorical_response2 )
            CV_ERROR( CV_StsBadArg, "Undefined or incorrect response type" );

        if( out_type_mask && *out_type_mask )
            (*out_type_mask)->data.ptr[dims_selected] = (uchar)is_categorical_response;

        if( out_responses )
        {
            //CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: output out_responses pointer must be passed" );
            CV_CALL( *out_responses = cvCreateMat( 1, samples_selected,
                                                   is_categorical_response ?
                                                   CV_32SC1 : CV_32FC1 ));

            if( flags & CV_TRAIN_STATMODEL_RESPONSES_ON_OUTPUT )
            {
                //if( !out_response_map )
                //   CV_ERROR( CV_StsNullPtr,
                //   "If the <responses> array is output, output pointer out_response_map must be passed" );
            }
            else
            {
                int* map = out_sample_idx ? out_sample_idx->data.i : 0;
                int* r_idata = responses->data.i;
                float* r_fldata = (float*)r_idata;

                if( is_categorical_response )
                {
                    int* r_out_data = (*out_responses)->data.i;
                    int cls_count = 1;
                    int prev_cls;
            
                    if( !out_response_map )
                        CV_ERROR( CV_StsNullPtr,
                        "In case of categorical response the output "
                        "pointer out_response_map must be passed" );

                    CV_CALL( temp_response_ptr = (int**)cvAlloc( samples_selected*sizeof(int*)));

                    for( i = 0; i < samples_selected; i++ )
                    {
                        int idx = map ? map[i] : i;
                        assert( (unsigned)idx < (unsigned)samples_all );
                        if( r_type == CV_32SC1 )
                            r_out_data[i] = r_idata[idx*r_step];
                        else
                        {
                            float rf = r_fldata[idx*r_step];
                            int ri = cvRound(rf);
                            if( ri != rf )
                                CV_ERROR( CV_StsBadArg, "Some of responses are not integral" );
                            r_out_data[i] = ri;
                        }
                        temp_response_ptr[i] = r_out_data + i;
                    }

                    qsort( temp_response_ptr, samples_selected, sizeof(int*), icvCmpIntegersPtr );
            
                    // count the classes
                    for( i = 1; i < samples_selected; i++ )
                        cls_count += *temp_response_ptr[i] != *temp_response_ptr[i-1];

                    if( cls_count < 2 )
                        CV_ERROR( CV_StsBadArg, "There is only a single class" );

                    CV_CALL( *out_response_map = cvCreateMat( 1, cls_count, CV_32SC1 ));
            
                    // compact the class indices and build the map
                    prev_cls = ~*temp_response_ptr[0];
                    cls_count = -1;

                    for( i = 0; i < samples_selected; i++ )
                    {
                        int cur_cls = *temp_response_ptr[i];
                        if( cur_cls != prev_cls )
                        {
                            (*out_response_map)->data.i[++cls_count] = cur_cls;
                            prev_cls = cur_cls;
                        }
                        *temp_response_ptr[i] = cls_count;
                    }
                }
                else
                {
                    float* r_out_data = (*out_responses)->data.fl;
                    for( i = 0; i < samples_selected; i++ )
                    {
                        int idx = map ? map[i] : i;
                        assert( (unsigned)idx < (unsigned)samples_all );
                        r_out_data[i] = r_type == CV_32SC1 ? (float)r_idata[idx*r_step] :
                                                                    r_fldata[idx*r_step];
                    }
                }
            }
        }
    }

    switch( flags & (CV_TRAIN_STATMODEL_SAMPLES_AS_ROWS|CV_TRAIN_STATMODEL_SAMPLES_AS_COLUMNS) )
    {
    case CV_TRAIN_STATMODEL_SAMPLES_AS_ROWS:
        if( tflag )
            copy_train_data = transpose_train_data = 1;
        break;
    case CV_TRAIN_STATMODEL_SAMPLES_AS_COLUMNS:
        if( !tflag )
            copy_train_data = transpose_train_data = 1;
        break;
    default:
        /* pass */
        ;
    }

    if( (flags & CV_TRAIN_STATMODEL_DEFRAGMENT_TRAIN_DATA) &&
        (!(tflag ^ transpose_train_data) && out_comp_idx && *out_comp_idx ||
          (tflag ^ transpose_train_data) && out_sample_idx) )
        copy_train_data = defragment_train_data = 1;

    *_dims_all = dims_all;

    if( !(tflag ^ transpose_train_data) )
    {
        *sample_count = out_train_data_rows = samples_selected;
        *_dims = out_train_data_cols = defragment_train_data ? dims_selected : dims_all;
        row_map = out_sample_idx ? out_sample_idx->data.i : 0;
        col_map = out_comp_idx && *out_comp_idx ? (*out_comp_idx)->data.i : 0;
    }
    else
    {
        *_dims = out_train_data_rows = dims_selected;
        *sample_count = out_train_data_cols = defragment_train_data ?
                                        samples_selected : samples_all;
        row_map = out_comp_idx && *out_comp_idx ? (*out_comp_idx)->data.i : 0;
        col_map = out_sample_idx ? out_sample_idx->data.i : 0;
    }

    if( is_sparse )
    {
        int src_len, dst_len, step = 1;
        CvSparseMat* sparse = (CvSparseMat*)train_data;
        CvSparseNode* node;
        CvSparseMatIterator mat_iterator;
        int convert_to_dense = (flags & CV_TRAIN_STATMODEL_SPARSE_AS_SPARSE) == 0;
        int *map0 = row_map, *map1 = col_map, *t;

        if( sparse->heap->active_count*3 > nd_size[0]*nd_size[1] )
            convert_to_dense = 1;

        if( missval_mask )
            CV_ERROR( CV_StsNotImplemented, "missing value mask can not be used with sparse train data" );

        if( tflag )
            CV_SWAP( map0, map1, t );

        src_len = nd_size[0];
        dst_len = !transpose_train_data ? out_train_data_rows : out_train_data_cols;
        if( convert_to_dense )
            step = !transpose_train_data ? out_train_data_cols : 1;
        CV_CALL( inverse_map0 = (int*)cvAlloc( src_len*sizeof(int) ));
        memset( inverse_map0, -1, src_len*sizeof(int));
        for( i = 0; i < dst_len; i++ )
            inverse_map0[!map0 ? i : map0[i]] = i*step;

        src_len = nd_size[1];
        dst_len = !transpose_train_data ? out_train_data_cols : out_train_data_rows;
        if( convert_to_dense )
            step = !transpose_train_data ? 1 : out_train_data_cols;
        CV_CALL( inverse_map1 = (int*)cvAlloc( src_len*sizeof(int) ));
        memset( inverse_map1, -1, src_len*sizeof(int));
        for( i = 0; i < dst_len; i++ )
            inverse_map1[!map1 ? i : map1[i]] = i*step;

        if( !(flags & CV_TRAIN_STATMODEL_SPARSE_AS_SPARSE) )
        {
            CV_CALL( out_train_data = (float**)cvAlloc( out_train_data_rows*
                    (sizeof(out_train_data[0]) + 
                    out_train_data_cols*sizeof(out_train_data[0][0]))));

            for( i = 0; i < out_train_data_rows; i++ )
                out_train_data[i] = (float*)(out_train_data +
                    out_train_data_rows) + i*out_train_data_cols;

            memset( out_train_data + out_train_data_rows, 0, out_train_data_rows*
                    out_train_data_cols*sizeof(out_train_data[0][0]));

            node = cvInitSparseMatIterator( sparse, &mat_iterator );
            for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
            {
                int* idx = CV_NODE_IDX( sparse, node );
                float val = *(float*)CV_NODE_VAL( sparse, node );
                int i0 = inverse_map0[idx[0]];
                int i1 = inverse_map1[idx[1]];
                if( (i0 | i1) < 0 )
                    continue;
                ((float*)(out_train_data + out_train_data_rows))[i0+i1] = val;
            }
        }
        else
        {
            int idx0 = transpose_train_data ^ tflag, idx1 = idx0 ^ 1;
            int total = out_train_data_rows;
            CvSparseVecElem32f* ptr;
            
            CV_CALL( counters = (int*)cvAlloc( out_train_data_rows*sizeof(counters[0])));
            memset( counters, 0, out_train_data_rows*sizeof(counters[0]));

            node = cvInitSparseMatIterator( sparse, &mat_iterator );
            for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
            {
                int* idx = CV_NODE_IDX( sparse, node );
                int i0 = inverse_map0[idx[idx0]];
                int i1 = inverse_map1[idx[idx1]];
                if( (i0 | i1) < 0 )
                    continue;
                counters[i0]++;
            }

            for( i = 0; i < out_train_data_rows; i++ )
                total += counters[i];

            CV_CALL( out_train_data = (float**)cvAlloc( (out_train_data_rows+1)*
                    sizeof(out_train_data[0]) +
                    total*sizeof(CvSparseVecElem32f)));
            ptr = (CvSparseVecElem32f*)(out_train_data + out_train_data_rows + 1);
            for( i = 0; i < out_train_data_rows; i++ )
            {
                out_train_data[i] = (float*)ptr;
                ptr += counters[i] + 1;
            }
            out_train_data[i] = (float*)ptr;

            node = cvInitSparseMatIterator( sparse, &mat_iterator );
            for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
            {
                int* idx = CV_NODE_IDX( sparse, node );
                int i0 = inverse_map0[idx[idx0]];
                int i1 = inverse_map1[idx[idx1]];
                if( (i0 | i1) < 0 )
                    continue;
                ptr = (CvSparseVecElem32f*)(out_train_data[i0]);
                ptr->idx = i1;
                ptr->val = *(float*)CV_NODE_VAL( sparse, node );
                out_train_data[i0] = (float*)(ptr + 1);
            }

            for( i = 0; i < out_train_data_rows; i++ )
            {
                ptr = (CvSparseVecElem32f*)(out_train_data[i]);
                ptr->idx = -1;
                out_train_data[i] = (float*)(ptr - counters[i]);
                if( counters[i] > 1 )
                    qsort( out_train_data[i], counters[i],
                           sizeof(ptr[0]), icvCmpSparseVecElems );
            }
        }

        if( _is_sparse )
            *_is_sparse = !convert_to_dense;

        ok = 1;
        EXIT;
    }

    if( missval_mask )
    {
        if( !_out_missval_mask )
            CV_ERROR( CV_StsNullPtr,
            "INTERNAL ERROR: output out_missval_mask pointer must be passed" );
    }

    CV_CALL( out_train_data = (float**)cvAlloc( out_train_data_rows*
            (sizeof(out_train_data[0]) + (copy_train_data ?
            out_train_data_cols*sizeof(out_train_data[0][0]) : 0))));

    if( !CV_IS_MAT(train_data) )
        CV_ERROR( CV_StsBadArg, "Input matrix is not array" );

    src_step = train_data->step / sizeof(out_train_data[0][0]);

    if( missval_mask )
    {
        CV_CALL( out_missval_mask = (uchar**)cvAlloc( out_train_data_rows*
            (sizeof(out_missval_mask[0]) + (copy_train_data ? out_train_data_cols : 0))));
        src_mask_step = missval_mask->step;
    }

    if( !copy_train_data )
    {
        for( i = 0; i < out_train_data_rows; i++ )
        {
            int row_idx = row_map ? row_map[i] : i;
            out_train_data[i] = train_data->data.fl + row_idx*src_step;
            if( out_missval_mask )
                out_missval_mask[i] = missval_mask->data.ptr + row_idx*missval_mask->step;
        }
    }
    else if( !transpose_train_data )
    {
        //assert( col_map != 0 );

        for( i = 0; i < out_train_data_rows; i++ )
        {
            int row_idx = row_map ? row_map[i] : i;
            dst_data = out_train_data[i] =
                (float*)(out_train_data + out_train_data_rows) + i*out_train_data_cols;
            src_data = (float*)(train_data->data.ptr + row_idx*train_data->step);

            if( out_missval_mask )
            {
                dst_mask = out_missval_mask[i] = (uchar*)(out_missval_mask +
                    out_train_data_rows) + i*out_train_data_cols;
                src_mask = missval_mask->data.ptr + row_idx*missval_mask->step;
            }

            for( j = 0; j < out_train_data_cols; j++ )
            {
                int col_idx = !col_map ? j : col_map[j];
                dst_data[j] = src_data[col_idx];
                if( dst_mask )
                    dst_mask[j] = src_mask[col_idx];
            }
        }
    }
    else if( transpose_train_data && !defragment_train_data )
    {
        assert( col_map == 0 );

        for( i = 0; i < out_train_data_rows; i++ )
        {
            int col_idx = row_map ? row_map[i] : i;
            dst_data = out_train_data[i] =
                (float*)(out_train_data + out_train_data_rows) + i*out_train_data_cols;
            src_data = train_data->data.fl + col_idx;

            if( out_missval_mask )
            {
                dst_mask = out_missval_mask[i] = (uchar*)(out_missval_mask +
                    out_train_data_rows) + i*out_train_data_cols;
                src_mask = missval_mask->data.ptr + col_idx;
            }

            for( j = 0; j < out_train_data_cols; j++ )
            {
                dst_data[j] = src_data[j*src_step];
                if( dst_mask )
                    dst_mask[j] = src_mask[j*src_mask_step];
            }
        }
    }
    else
    {
        assert( transpose_train_data && defragment_train_data && col_map != 0 );

        for( i = 0; i < out_train_data_rows; i++ )
        {
            int col_idx = row_map ? row_map[i] : i;
            dst_data = out_train_data[i] =
                (float*)(out_train_data + out_train_data_rows) + i*out_train_data_cols;
            src_data = train_data->data.fl + col_idx;

            if( out_missval_mask )
            {
                dst_mask = out_missval_mask[i] = (uchar*)(out_missval_mask +
                    out_train_data_rows) + i*out_train_data_cols;
                src_mask = missval_mask->data.ptr + col_idx;
            }

            for( j = 0; j < out_train_data_cols; j++ )
            {
                int row_idx = col_map ? col_map[j] : j;
                dst_data[j] = src_data[row_idx*src_step];
                if( out_missval_mask )
                    dst_mask[j] = src_mask[row_idx*src_mask_step];
            }
        }
    }

    ok = 1;

    __END__;

    cvFree( (void**)&temp_response_ptr );
    cvFree( (void**)&counters );
    cvFree( (void**)&inverse_map0 );
    cvFree( (void**)&inverse_map1 );

    if( cvGetErrStatus() >= 0 )
    {
        *_out_train_data = (const float**)out_train_data;
        
        if( _out_missval_mask )
            *_out_missval_mask = (const uchar**)out_missval_mask;

        if( allocated_train_data )
            *allocated_train_data = copy_train_data;
        if( _out_sample_idx )
            *_out_sample_idx = out_sample_idx;
    }
    else
    {
        cvFree( (void**)&out_train_data );
        cvFree( (void**)&out_missval_mask );
        
        if( out_responses )
            cvReleaseMat( out_responses );

        if( out_response_map )
            cvReleaseMat( out_response_map );

        if( out_comp_idx )
            cvReleaseMat( out_comp_idx );

        if( out_sample_idx )
            cvReleaseMat( &out_sample_idx );

        if( out_type_mask )
            cvReleaseMat( out_type_mask );
    }

    return ok;
}


void
icvBtNewTrainData( CvBtClassifier* bt,
                   CvMat* train_data,
                   int flags,
                   CvMat* responses,
                   CvMat* comp_idx,
                   CvMat* sample_idx,
                   CvMat* type_mask,
                   CvMat* missval_mask )
{
    const float** new_raw_train_data = NULL;
    CvMat* new_responses = NULL;
    CvMat* new_resp_map = NULL;
    CvMat* new_comp_idx = NULL;
    CvMat* new_sample_idx = NULL;
    CvMat* new_type_mask = NULL;

    CV_FUNCNAME( "icvBtNewTrainData" );
    __BEGIN__;

    int i;
    int allocated_train_data = 0;
    
    int total_samples = 0;
    int selected_features = 0;
    int total_features = 0;
    
    int new_bt = (bt->weak->total == 0);

    CvBoostTrainState* ts = (CvBoostTrainState*) bt->ts;

    CV_CALL( cvPrepareTrainData( "cvUpdateBtClassifier",
            CV_TRAIN_STATMODEL_SAMPLES_AS_ROWS |
            CV_TRAIN_STATMODEL_DEFRAGMENT_TRAIN_DATA |
            CV_TRAIN_STATMODEL_ALWAYS_COPY_TRAIN_DATA |
            CV_TRAIN_STATMODEL_CATEGORICAL_RESPONSE,
            train_data, flags, responses,
            comp_idx, sample_idx, type_mask, missval_mask,
            &new_raw_train_data, &allocated_train_data,
            &total_samples, &selected_features, &total_features,
            &new_responses, &new_resp_map,
            &new_comp_idx, &new_sample_idx,
            &new_type_mask, NULL /* out_missval_mask */,
            NULL /* is_sparse */));

    assert( allocated_train_data == 1 ); /* a copy is needed */
    assert( new_raw_train_data != NULL);

    if( !new_bt && total_features < bt->total_features )
        CV_ERROR( CV_StsBadArg, "Subsequent training set must have at least the same"
            " number of features" );
    if( new_resp_map->cols != 2 )
        CV_ERROR( CV_StsBadArg, "Only two-class problems are supported" );
    if( !new_bt && 
        ((CV_MAT_ELEM( *bt->class_labels, int, 0, 0 ) !=
          CV_MAT_ELEM( *new_resp_map, int, 0, 0 )) ||
         (CV_MAT_ELEM( *bt->class_labels, int, 0, 1 ) !=
          CV_MAT_ELEM( *new_resp_map, int, 0, 1 ))) )
    {
        CV_ERROR( CV_StsBadArg, "Subsequent class labels must be the same" );
    }
    if( !new_bt )
    {
        int invalid_comp_idx = 0;
        if( (!bt->comp_idx && new_comp_idx) || (bt->comp_idx && !new_comp_idx) )
            invalid_comp_idx = 1;
        else if( bt->comp_idx && new_comp_idx )
        {
            if( bt->comp_idx->cols > new_comp_idx->cols )
                invalid_comp_idx = 1;
            else
            {
                for( i = 0; i < bt->comp_idx->cols; ++i )
                {
                    if( CV_MAT_ELEM( *bt->comp_idx, int, 0, i ) !=
                        CV_MAT_ELEM( *new_comp_idx, int, 0, i ) )
                    {
                        invalid_comp_idx = 1;
                        break;
                    }
                }
            }
        }
        if( invalid_comp_idx )
            CV_ERROR( CV_StsBadArg, "Subsequent comp_idx must be the superset of the "
            "previous one" );
    }
    if( new_type_mask )
    {
        for( i = 0; i < new_type_mask->cols - 1; ++i )
            if( CV_MAT_ELEM( *new_type_mask, uchar, 0, i ) != 0 )
                CV_ERROR( CV_StsBadArg, "Only numerical features are supported" );
            if( CV_MAT_ELEM( *new_type_mask, uchar, 0, i ) != 1 )
                CV_ERROR( CV_StsBadArg, "Only categorical responses are supported" );
    }

    /* "commit" */
    bt->total_features = total_features;
    cvReleaseMat( &bt->class_labels );
    bt->class_labels = new_resp_map;
    cvReleaseMat( &bt->comp_idx );
    bt->comp_idx = new_comp_idx;

    cvFree( (void**) &ts->raw_train_data );
    ts->raw_train_data = new_raw_train_data;
    assert(ts->raw_train_data != NULL);
    CV_CALL( cvInitMatHeader( &ts->train_data,
            CV_IS_ROW_SAMPLE( flags ) ? total_samples : selected_features,
            CV_IS_ROW_SAMPLE( flags ) ? selected_features : total_samples, CV_32FC1,
            (void*) new_raw_train_data[0],
            ((char*)new_raw_train_data[1] - (char*)new_raw_train_data[0]) ));
    assert(ts->raw_train_data != NULL);
    /* the cvPrepareTrainData function always copies train data and rearranges it in 
       a such way that samples are in rows */
    /* ts->flags = flags; */
    ts->flags = CV_ROW_SAMPLE;
    cvReleaseMat( &ts->responses );
    ts->responses = new_responses;

    ts->valid = 0;

    __END__;

    if( cvGetErrStatus() < 0 )
    {
        cvFree( (void**) &new_raw_train_data );
        cvReleaseMat( &new_responses );
        cvReleaseMat( &new_resp_map );
        cvReleaseMat( &new_comp_idx );
    }
    cvReleaseMat( &new_type_mask );
    cvReleaseMat( &new_sample_idx );
}



	double
cvBtGetParamValue( CvStatModel* model,
                   const void* object,
                   int param_type,
                   int /* index */ )
{
    double val = 0;

    CV_FUNCNAME( "cvBtGetParamValue" );

    __BEGIN__;
    CvBtClassifier* bt = (CvBtClassifier*) model;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
    if( object )
        CV_ERROR( CV_StsBadArg, "Object pointer must be NULL" );

    switch( param_type )
    {
        case CV_MODEL_CLASS_NUM:
            val = (double) ((bt->class_labels) ? bt->class_labels->cols : 0);
            break;
        case CV_MODEL_FEATURE_NUM:
            val = (double) (bt->total_features);
            break;

        case CV_BT_ITER_STEP:
            val = (double) (bt->params.num_iter);
            break;
        case CV_BT_NUM_ITER:
            val = (double) ((bt->weak) ? bt->weak->total : 0);
            break;
        default:
            CV_ERROR( CV_StsBadArg, "Unsupported parameter" );
    }

    __END__;
    
    return val;
}

void
cvBtGetParamMat( CvStatModel* model,
                 const void* object,
                 int param_type,
                 int /* index */,
                 CvMat* mat )
{
    CV_FUNCNAME( "cvBtGetParamMat" );

    __BEGIN__;
    CvBtClassifier* bt = (CvBtClassifier*) model;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
    if( object )
        CV_ERROR( CV_StsBadArg, "Object pointer must be NULL" );

    mat->data.ptr = NULL;

    switch( param_type )
    {
        case CV_BT_WEIGHTS:
        {
            CvMat* weights = ((CvBoostTrainState*) bt->ts)->weights;

            if( weights )
            {
                CV_CALL( cvInitMatHeader( mat, weights->rows, weights->cols,
                    CV_MAT_TYPE( weights->type ),
                    weights->data.ptr, weights->step ));
            }
            break;
        }
        default:
            CV_ERROR( CV_StsBadArg, "Unsupported parameter" );
    }

    __END__;
}

void
cvBtSetParamValue( CvStatModel* model,
                   const void* object,
                   int param_type,
                   int /* index */,
                   double value )
{
    CV_FUNCNAME( "cvBtSetParamValue" );

    __BEGIN__;
    CvBtClassifier* bt = (CvBtClassifier*) model;
    int i;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
    if( object )
        CV_ERROR( CV_StsBadArg, "Object pointer must be NULL" );

    switch( param_type )
    {
        case CV_BT_ITER_STEP:
            i = cvRound( value );
            if( i < 0 )
                CV_ERROR( CV_StsBadArg, "Iteration step value must be non-negative" );
            bt->params.num_iter = i;
            if( bt->params.num_iter == 0 )
                icvReleaseBoostTrainStateMembers( (CvBoostTrainState*) bt->ts );
            break;
        default:
            CV_ERROR( CV_StsBadArg, "Unsupported parameter" );
    }

    __END__;
}

void
cvBtSetParamMat( CvStatModel* model,
                 const void* object,
                 int param_type,
                 int /* index */,
                 CvMat* mat )
{
    CV_FUNCNAME( "cvBtSetParamMat" );

    __BEGIN__;
    CvBtClassifier* bt = (CvBtClassifier*) model;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
    if( object )
        CV_ERROR( CV_StsBadArg, "Object pointer must be NULL" );

    switch( param_type )
    {
        case CV_BT_WEIGHTS:
        {
            CvMat** weights = &(((CvBoostTrainState*) bt->ts)->weights);

            if( !CV_IS_MAT( mat ) )
                CV_ERROR( mat ? CV_StsBadArg : CV_StsNullPtr, "Invalid matrix");
            if( CV_MAT_TYPE( mat->type ) != CV_32FC1 )
                CV_ERROR( CV_StsBadArg, "Matrix must be of 32fC1 type");
            if( mat->rows != 1 && mat->cols != 1 )
                CV_ERROR( CV_StsBadArg, "Matrix must have 1 row or 1 column");

            if( !(*weights) || (*weights)->cols != mat->rows + mat->cols - 1 )
            {
                cvReleaseMat( weights );
                CV_CALL( *weights = cvCreateMat( 1, mat->rows + mat->cols - 1,
                    CV_32FC1 ));
                ((CvBoostTrainState*) bt->ts)->valid = 0;
            }
            if( mat->rows == 1 )
            {
                CV_CALL( cvCopy( mat, *weights ) );
            }
            else
            {
                CV_CALL( cvTranspose( mat, *weights ) );
            }
            break;
        }
        break;
        default:
            CV_ERROR( CV_StsBadArg, "Unsupported parameter" );
    }

    __END__;
}




#define ICV_DEF_FIND_STUMP_THRESHOLD( suffix, type, error )                              \
int icvFindStumpThreshold_##suffix(                                              \
        uchar* data, int datastep,                                                       \
        uchar* wdata, int wstep,                                                         \
        uchar* ydata, int ystep,                                                         \
        uchar* idxdata, int idxstep, int num,                                            \
        float* lerror,                                                                   \
        float* rerror,                                                                   \
        float* threshold, float* left, float* right,                                     \
        float* sumw, float* sumwy, float* sumwyy )                                       \
{                                                                                        \
    int found = 0;                                                                       \
    float wyl  = 0.0F;                                                                   \
    float wl   = 0.0F;                                                                   \
    float wyyl = 0.0F;                                                                   \
    float wyr  = 0.0F;                                                                   \
    float wr   = 0.0F;                                                                   \
                                                                                         \
    float curleft  = 0.0F;                                                               \
    float curright = 0.0F;                                                               \
    float* prevval = NULL;                                                               \
    float* curval  = NULL;                                                               \
    float curlerror = 0.0F;                                                              \
    float currerror = 0.0F;                                                              \
    float wposl;                                                                         \
    float wposr;                                                                         \
                                                                                         \
    int i = 0;                                                                           \
    int idx = 0;                                                                         \
                                                                                         \
    wposl = wposr = 0.0F;                                                                \
    if( *sumw == FLT_MAX )                                                               \
    {                                                                                    \
        /* calculate sums */                                                             \
        float *y = NULL;                                                                 \
        float *w = NULL;                                                                 \
        float wy = 0.0F;                                                                 \
                                                                                         \
        *sumw   = 0.0F;                                                                  \
        *sumwy  = 0.0F;                                                                  \
        *sumwyy = 0.0F;                                                                  \
        for( i = 0; i < num; i++ )                                                       \
        {                                                                                \
            idx = (int) ( *((type*) (idxdata + i*idxstep)) );                            \
            w = (float*) (wdata + idx * wstep);                                          \
            *sumw += *w;                                                                 \
            y = (float*) (ydata + idx * ystep);                                          \
            wy = (*w) * (*y);                                                            \
            *sumwy += wy;                                                                \
            *sumwyy += wy * (*y);                                                        \
        }                                                                                \
    }                                                                                    \
                                                                                         \
    for( i = 0; i < num; i++ )                                                           \
    {                                                                                    \
        idx = (int) ( *((type*) (idxdata + i*idxstep)) );                                \
        curval = (float*) (data + idx * datastep);                                       \
         /* for debug purpose */                                                         \
        if( i > 0 ) assert( (*prevval) <= (*curval) );                                   \
                                                                                         \
        wyr  = *sumwy - wyl;                                                             \
        wr   = *sumw  - wl;                                                              \
                                                                                         \
        if( wl > 0.0 ) curleft = wyl / wl;                                               \
        else curleft = 0.0F;                                                             \
                                                                                         \
        if( wr > 0.0 ) curright = wyr / wr;                                              \
        else curright = 0.0F;                                                            \
                                                                                         \
        error                                                                            \
                                                                                         \
        if( curlerror + currerror < (*lerror) + (*rerror) )                              \
        {                                                                                \
            (*lerror) = curlerror;                                                       \
            (*rerror) = currerror;                                                       \
            *threshold = *curval;                                                        \
            if( i > 0 ) {                                                                \
                *threshold = 0.5F * (*threshold + *prevval);                             \
            }                                                                            \
            *left  = curleft;                                                            \
            *right = curright;                                                           \
            found = 1;                                                                   \
        }                                                                                \
                                                                                         \
        do                                                                               \
        {                                                                                \
            wl  += *((float*) (wdata + idx * wstep));                                    \
            wyl += (*((float*) (wdata + idx * wstep)))                                   \
                * (*((float*) (ydata + idx * ystep)));                                   \
            wyyl += *((float*) (wdata + idx * wstep))                                    \
                * (*((float*) (ydata + idx * ystep)))                                    \
                * (*((float*) (ydata + idx * ystep)));                                   \
        }                                                                                \
        while( (++i) < num &&                                                            \
            ( *((float*) (data + (idx =                                                  \
                (int) ( *((type*) (idxdata + i*idxstep))) ) * datastep))                 \
                == *curval ) );                                                          \
        --i;                                                                             \
        prevval = curval;                                                                \
    } /* for each value */                                                               \
                                                                                         \
    return found;                                                                        \
}

/* misclassification error
 * err = MIN( wpos, wneg );
 */
#define ICV_DEF_FIND_STUMP_THRESHOLD_MISC( suffix, type )                                \
    ICV_DEF_FIND_STUMP_THRESHOLD( misc_##suffix, type,                                   \
        wposl = 0.5F * ( wl + wyl );                                                     \
        wposr = 0.5F * ( wr + wyr );                                                     \
        curleft = 0.5F * ( 1.0F + curleft );                                             \
        curright = 0.5F * ( 1.0F + curright );                                           \
        curlerror = MIN( wposl, wl - wposl );                                            \
        currerror = MIN( wposr, wr - wposr );                                            \
    )

/* gini error
 * err = 2 * wpos * wneg /(wpos + wneg)
 */
#define ICV_DEF_FIND_STUMP_THRESHOLD_GINI( suffix, type )                                \
    ICV_DEF_FIND_STUMP_THRESHOLD( gini_##suffix, type,                                   \
        wposl = 0.5F * ( wl + wyl );                                                     \
        wposr = 0.5F * ( wr + wyr );                                                     \
        curleft = 0.5F * ( 1.0F + curleft );                                             \
        curright = 0.5F * ( 1.0F + curright );                                           \
        curlerror = 2.0F * wposl * ( 1.0F - curleft );                                   \
        currerror = 2.0F * wposr * ( 1.0F - curright );                                  \
    )

#define CV_ENTROPY_THRESHOLD FLT_MIN


#define ICV_DEF_FIND_STUMP_THRESHOLD_ENTROPY( suffix, type )                             \
    ICV_DEF_FIND_STUMP_THRESHOLD( entropy_##suffix, type,                                \
        wposl = 0.5F * ( wl + wyl );                                                     \
        wposr = 0.5F * ( wr + wyr );                                                     \
        curleft = 0.5F * ( 1.0F + curleft );                                             \
        curright = 0.5F * ( 1.0F + curright );                                           \
        curlerror = currerror = 0.0F;                                                    \
        if( curleft > CV_ENTROPY_THRESHOLD )                                             \
            curlerror -= wposl * logf( curleft );                                        \
        if( curleft < 1.0F - CV_ENTROPY_THRESHOLD )                                      \
            curlerror -= (wl - wposl) * logf( 1.0F - curleft );                          \
                                                                                         \
        if( curright > CV_ENTROPY_THRESHOLD )                                            \
            currerror -= wposr * logf( curright );                                       \
        if( curright < 1.0F - CV_ENTROPY_THRESHOLD )                                     \
            currerror -= (wr - wposr) * logf( 1.0F - curright );                         \
    )

/* least sum of squares error */
#define ICV_DEF_FIND_STUMP_THRESHOLD_SQ( suffix, type )                                  \
    ICV_DEF_FIND_STUMP_THRESHOLD( sq_##suffix, type,                                     \
        /* calculate error (sum of squares)          */                                  \
        /* err = sum( w * (y - left(rigt)Val)^2 )    */                                  \
        curlerror = wyyl + curleft * curleft * wl - 2.0F * curleft * wyl;                \
        currerror = (*sumwyy) - wyyl + curright * curright * wr - 2.0F * curright * wyr; \
    )

ICV_DEF_FIND_STUMP_THRESHOLD_MISC( 16s, short )

ICV_DEF_FIND_STUMP_THRESHOLD_MISC( 32s, int )

ICV_DEF_FIND_STUMP_THRESHOLD_MISC( 32f, float )


ICV_DEF_FIND_STUMP_THRESHOLD_GINI( 16s, short )

ICV_DEF_FIND_STUMP_THRESHOLD_GINI( 32s, int )

ICV_DEF_FIND_STUMP_THRESHOLD_GINI( 32f, float )


ICV_DEF_FIND_STUMP_THRESHOLD_ENTROPY( 16s, short )

ICV_DEF_FIND_STUMP_THRESHOLD_ENTROPY( 32s, int )

ICV_DEF_FIND_STUMP_THRESHOLD_ENTROPY( 32f, float )


ICV_DEF_FIND_STUMP_THRESHOLD_SQ( 16s, short )

ICV_DEF_FIND_STUMP_THRESHOLD_SQ( 32s, int )

ICV_DEF_FIND_STUMP_THRESHOLD_SQ( 32f, float )

typedef int (*CvFindThresholdFunc)( uchar* data, int datastep,
                                    uchar* wdata, int wstep,
                                    uchar* ydata, int ystep,
                                    uchar* idxdata, int idxstep, int num,
                                    float* lerror,
                                    float* rerror,
                                    float* threshold, float* left, float* right,
                                    float* sumw, float* sumwy, float* sumwyy );

CvFindThresholdFunc findStumpThreshold_16s[4] = {
        icvFindStumpThreshold_misc_16s,
        icvFindStumpThreshold_gini_16s,
        icvFindStumpThreshold_entropy_16s,
        icvFindStumpThreshold_sq_16s
    };

CvFindThresholdFunc findStumpThreshold_32s[4] = {
        icvFindStumpThreshold_misc_32s,
        icvFindStumpThreshold_gini_32s,
        icvFindStumpThreshold_entropy_32s,
        icvFindStumpThreshold_sq_32s
    };

CvFindThresholdFunc findStumpThreshold_32f[4] = {
        icvFindStumpThreshold_misc_32f,
        icvFindStumpThreshold_gini_32f,
        icvFindStumpThreshold_entropy_32f,
        icvFindStumpThreshold_sq_32f
    };

void cvReleaseStumpClassifier( CvClassifier** classifier )
{
    cvFree( (void**) classifier );
    *classifier = 0;
}


float cvEvalStumpClassifier( const CvClassifier* classifier,
                             const CvMat* sample, CvMat* )
{
    assert( classifier != NULL );
    assert( sample != NULL );
    assert( CV_MAT_TYPE( sample->type ) == CV_32FC1 );
    
    if( (CV_MAT_ELEM( (*sample), float, 0,
            ((CvStumpClassifier*) classifier)->compidx )) <
        ((CvStumpClassifier*) classifier)->threshold ) 
    {
        return ((CvStumpClassifier*) classifier)->left;
    }
    else
    {
        return ((CvStumpClassifier*) classifier)->right;
    }
}


CvClassifier* cvCreateStumpClassifier( CvMat* trainData,
                     int flags,
                     CvMat* trainClasses,
                     CvMat* /*typeMask*/,
                     CvMat* /*missedMeasurementsMask*/,
                     CvMat* /*compIdx*/,
                     CvMat* sampleIdx,
                     CvMat* weights,
                     CvClassifierTrainParams* trainParams
                   )
{
    CvStumpClassifier* stump = NULL;
    int m = 0; /* number of samples */
    int n = 0; /* number of components */
    uchar* data = NULL;
    int cstep   = 0;
    int sstep   = 0;
    uchar* ydata = NULL;
    int ystep    = 0;
    uchar* idxdata = NULL;
    int idxstep    = 0;
    int l = 0; /* number of indices */     
    uchar* wdata = NULL;
    int wstep    = 0;

    int* idx = NULL;
    int i = 0;
    
    float sumw   = FLT_MAX;
    float sumwy  = FLT_MAX;
    float sumwyy = FLT_MAX;

    assert( trainData != NULL );
    assert( CV_MAT_TYPE( trainData->type ) == CV_32FC1 );
    assert( trainClasses != NULL );
    assert( CV_MAT_TYPE( trainClasses->type ) == CV_32FC1 );
    //assert( missedMeasurementsMask == NULL );
    //assert( compIdx == NULL );
    assert( weights != NULL );
    assert( CV_MAT_TYPE( weights->type ) == CV_32FC1 );
    assert( trainParams != NULL );
    //typeMask; compIdx; missedMeasurementsMask;

    data = trainData->data.ptr;
    if( CV_IS_ROW_SAMPLE( flags ) )
    {
        cstep = CV_ELEM_SIZE( trainData->type );
        sstep = trainData->step;
        m = trainData->rows;
        n = trainData->cols;
    }
    else
    {
        sstep = CV_ELEM_SIZE( trainData->type );
        cstep = trainData->step;
        m = trainData->cols;
        n = trainData->rows;
    }

    //Determine how to read the labels-- whether they're
    //in a row vector or a column vector.
    ydata = trainClasses->data.ptr;
    if( trainClasses->rows == 1 )
    {
        assert( trainClasses->cols == m );
        ystep = CV_ELEM_SIZE( trainClasses->type );
    }
    else
    {
        assert( trainClasses->rows == m );
        ystep = trainClasses->step;
    }

    //Determine how to read the weights-- whether they're
    //in a row vector or a column vector
    wdata = weights->data.ptr;
    if( weights->rows == 1 )
    {
        assert( weights->cols == m );
        wstep = CV_ELEM_SIZE( weights->type );
    }
    else
    {
        assert( weights->rows == m );
        wstep = weights->step;
    }

    l = m;
    if( sampleIdx != NULL )
    {
        assert( CV_MAT_TYPE( sampleIdx->type ) == CV_32FC1 );

        idxdata = sampleIdx->data.ptr;
        if( sampleIdx->rows == 1 )
        {
            l = sampleIdx->cols;
            idxstep = CV_ELEM_SIZE( sampleIdx->type );
        }
        else
        {
            l = sampleIdx->rows;
            idxstep = sampleIdx->step;
        }
        assert( l <= m );
    }

    idx = (int*) cvAlloc( l * sizeof( int ) );
    stump = (CvStumpClassifier*) cvAlloc( sizeof( CvStumpClassifier) );

    /* START */
    memset( (void*) stump, 0, sizeof( CvStumpClassifier ) );

    stump->predict = cvEvalStumpClassifier;
    stump->update = NULL;
    //stump->save = NULL;
    stump->release = cvReleaseStumpClassifier;

    stump->lerror = FLT_MAX;
    stump->rerror = FLT_MAX;
    stump->left  = 0.0F;
    stump->right = 0.0F;

    /* copy indices */
    if( sampleIdx != NULL )
    {
        for( i = 0; i < l; i++ )
        {
            idx[i] = (int) *((float*) (idxdata + i*idxstep));
        }
    }
    else
    {
        for( i = 0; i < l; i++ )
        {
            idx[i] = i;
        }
    }

    //[IG] Added check that some feature actually gets split on
    int found = 0;

    //std::cout<<"Checking "<<n<<" features"<<std::endl;


    //was bug hunting here last. the bug is that no feature gets split on, so
    //  it defaults to feature 0, threshold 0, which does not separate our
    //  data at all. 


    for( i = 0; i < n; i++ )
    {
        CvValArray va;

        va.data = data + i * cstep;
        va.step = sstep;
        icvSortIndexedValArray_32s( idx, l, &va );


        if( findStumpThreshold_32s[(int) ((CvStumpTrainParams*) trainParams)->error]
              ( data + i * cstep, sstep,
                wdata, wstep, ydata, ystep, (uchar*) idx, sizeof( int ), l,
                &(stump->lerror), &(stump->rerror),
                &(stump->threshold), &(stump->left), &(stump->right), 
                &sumw, &sumwy, &sumwyy ) )
        {
	  //std::cout<<"Selected feature "<<i<<std::endl;
            stump->compidx = i;
	    found = 1;
        }
    } /* for each component */
    //if (found)
    //std::cout<<"At end, selected feature is: "<<stump->compidx<<std::endl;
    //else
    //std::cout<<"Did not find a feature"<<std::endl;
    assert(found);

    /* END */

    cvFree( (void**) &idx );

    if( ((CvStumpTrainParams*) trainParams)->type == CV_CLASSIFICATION_CLASS )
    {
        stump->left = 2.0F * (stump->left >= 0.5F) - 1.0F;
        stump->right = 2.0F * (stump->right >= 0.5F) - 1.0F;
    }

    return (CvClassifier*) stump;
}


CvBtClassifierTrainParams*
icvBtDefaultTrainParams( CvBtClassifierTrainParams* tp )
{
    assert( tp );
    
    tp->boost_type = CV_BT_GENTLE;
    tp->num_iter = 100;
    tp->infl_trim_rate = 0.95f;
    tp->num_splits = 2;
    
    return tp;
}


CvStatModel*
cvCreateStatModel( int flags, int header_size,
                   CvStatModelRelease release,
                   CvStatModelPredict predict,
                   CvStatModelUpdate update )
{
    CvStatModel* model = 0;

    CV_FUNCNAME( "cvCreateStatModel" );

    __BEGIN__;

    if( !release )
        CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: NULL CvStatModelRelease pointer" );

    if( header_size < (int)sizeof(CvStatModel))
        CV_ERROR( CV_StsBadSize, "INTERNAL ERROR: bad stat model header size" );

    CV_CALL( model = (CvStatModel*)cvAlloc( header_size ));
    memset( model, 0, header_size );
    model->flags = flags;
    model->header_size = header_size;
    model->release = release;
    model->predict = predict;
    model->update = update;

    __END__;

    if( cvGetErrStatus() < 0 )
        cvFree( (void**)&model );

    return model;
}


int
icvEvalCARTBOOSTRow( CvCARTBOOSTClassifier* tree, const float* sample )
{
    assert( tree != NULL );
    assert( sample != NULL );

    int idx = 0;

    do
    {
        if( sample[tree->comp_idx[idx]] < tree->threshold[idx] )
            idx = tree->left[idx];
        else
            idx = tree->right[idx];
    } while( idx > 0 );

    return -idx;
}



void
cvPreparePredictData( const CvArr* _sample, int dims_all,
                      const CvMat* comp_idx, int class_count,
                      const CvMat* prob, float** _row_sample,
                      int as_sparse )
{
    float* row_sample = 0;
    int* inverse_comp_idx = 0;
    
    CV_FUNCNAME( "cvPreparePredictData" );

    __BEGIN__;

    const CvMat* sample = (const CvMat*)_sample;
    float* sample_data;
    int sample_step;
    int is_sparse = CV_IS_SPARSE_MAT(sample);
    int d, sizes[CV_MAX_DIM];
    int i, dims_selected;
    int vec_size;

    if( !is_sparse && !CV_IS_MAT(sample) )
        CV_ERROR( !sample ? CV_StsNullPtr : CV_StsBadArg, "The sample is not a valid vector" );

    if( cvGetElemType( sample ) != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat, "Input sample must have 32fC1 type" );

    CV_CALL( d = cvGetDims( sample, sizes ));

    if( !(is_sparse && d == 1 || !is_sparse && d == 2 && (sample->rows == 1 || sample->cols == 1)) )
        CV_ERROR( CV_StsBadSize, "Input sample must be 1-dimensional vector" );
    
    if( d == 1 )
        sizes[1] = 1;

    if( sizes[0] + sizes[1] - 1 != dims_all )
        CV_ERROR( CV_StsUnmatchedSizes,
        "The sample size is different from what has been used for training" );

    if( !_row_sample )
        CV_ERROR( CV_StsNullPtr, "INTERNAL ERROR: The row_sample pointer is NULL" );

    if( comp_idx && (!CV_IS_MAT(comp_idx) || comp_idx->rows != 1 ||
        CV_MAT_TYPE(comp_idx->type) != CV_32SC1) )
        CV_ERROR( CV_StsBadArg, "INTERNAL ERROR: invalid comp_idx" );

    dims_selected = comp_idx ? comp_idx->cols : dims_all;
    
    if( prob )
    {
        if( !CV_IS_MAT(prob) )
            CV_ERROR( CV_StsBadArg, "The output matrix of probabilities is invalid" );

        if( (prob->rows != 1 && prob->cols != 1) || CV_MAT_TYPE(prob->type) != CV_32FC1 )
            CV_ERROR( CV_StsBadSize,
            "The matrix of probabilities must be 1-dimensional vector of 32fC1 type" );

        if( prob->rows + prob->cols - 1 != class_count )
            CV_ERROR( CV_StsUnmatchedSizes,
            "The vector of probabilities must contain as many elements as "
            "the number of classes in the training set" );
    }

    vec_size = !as_sparse ? dims_selected*sizeof(row_sample[0]) :
                (dims_selected + 1)*sizeof(CvSparseVecElem32f);

    if( CV_IS_MAT(sample) )
    {
        sample_data = sample->data.fl;
        sample_step = sample->step / sizeof(row_sample[0]);

        if( !comp_idx && sample_step <= 1 && !as_sparse )
            *_row_sample = sample_data;
        else
        {
            CV_CALL( row_sample = (float*)cvAlloc( vec_size ));

            if( !comp_idx )
                for( i = 0; i < dims_selected; i++ )
                    row_sample[i] = sample_data[sample_step*i];
            else
            {
                int* comp = comp_idx->data.i;
                if( !sample_step )
                    for( i = 0; i < dims_selected; i++ )
                        row_sample[i] = sample_data[comp[i]];
                else
                    for( i = 0; i < dims_selected; i++ )
                        row_sample[i] = sample_data[sample_step*comp[i]];
            }

            *_row_sample = row_sample;
        }

        if( as_sparse )
        {
            const float* src = (const float*)row_sample;
            CvSparseVecElem32f* dst = (CvSparseVecElem32f*)row_sample;

            dst[dims_selected].idx = -1;
            for( i = dims_selected - 1; i >= 0; i-- )
            {
                dst[i].idx = i;
                dst[i].val = src[i];
            }
        }
    }
    else
    {
        CvSparseNode* node;
        CvSparseMatIterator mat_iterator;
        const CvSparseMat* sparse = (const CvSparseMat*)sample;
        assert( is_sparse );

        node = cvInitSparseMatIterator( sparse, &mat_iterator );
        CV_CALL( row_sample = (float*)cvAlloc( vec_size ));

        if( comp_idx )
        {
            CV_CALL( inverse_comp_idx = (int*)cvAlloc( dims_all*sizeof(int) ));
            memset( inverse_comp_idx, -1, dims_all*sizeof(int) );
            for( i = 0; i < dims_selected; i++ )
                inverse_comp_idx[comp_idx->data.i[i]] = i;
        }
        
        if( !as_sparse )
        {
            memset( row_sample, 0, vec_size );

            for( ; node != 0; node = cvGetNextSparseNode(&mat_iterator) )
            {
                int idx = *CV_NODE_IDX( sparse, node );
                if( inverse_comp_idx )
                {
                    idx = inverse_comp_idx[idx];
                    if( idx < 0 )
                        continue;
                }
                row_sample[idx] = *(float*)CV_NODE_VAL( sparse, node );
            }
        }
        else
        {
            CvSparseVecElem32f* ptr = (CvSparseVecElem32f*)row_sample;
            
            for( ; node != 0; node = cvGetNextSparseNode(&mat_iterator) )
            {
                int idx = *CV_NODE_IDX( sparse, node );
                if( inverse_comp_idx )
                {
                    idx = inverse_comp_idx[idx];
                    if( idx < 0 )
                        continue;
                }
                ptr->idx = idx;
                ptr->val = *(float*)CV_NODE_VAL( sparse, node );
                ptr++;
            }

            qsort( row_sample, ptr - (CvSparseVecElem32f*)row_sample,
                   sizeof(ptr[0]), icvCmpSparseVecElems );
            ptr->idx = -1;
        }

        *_row_sample = row_sample;
    }

    __END__;

    if( inverse_comp_idx )
        cvFree( (void**)&inverse_comp_idx );

    if( cvGetErrStatus() < 0 && _row_sample )
    {
        cvFree( (void**)&row_sample );
        *_row_sample = 0;
    }
}


float
cvEvalBtClassifier( const CvStatModel* model, const CvMat* sample, CvMat* prob )
{
    float* row_sample = NULL;
    int label = 0;

    CV_FUNCNAME( "cvEvalBtClassifier" );

    __BEGIN__;

    float val = 0;
    CvBtClassifier* bt = (CvBtClassifier*) model;
    int i;
    CvSeqReader reader;
    CvCARTBOOSTClassifier* tree;
    int cls = 0;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");

    CV_CALL( cvPreparePredictData( sample, bt->total_features, bt->comp_idx,
        bt->class_labels->cols, prob, &row_sample ));
    
    CV_CALL( cvStartReadSeq( bt->weak, &reader ) );
    for( i = 0; i < bt->weak->total; ++i )
    {
        CV_READ_SEQ_ELEM( tree, reader );
        val += tree->val[icvEvalCARTBOOSTRow( tree, row_sample )];
    }
    cls = (val >= 0.0F);
    if( prob )
    {
        if( prob->rows == 1 )
        {
            CV_MAT_ELEM( *prob, float, 0, 0 ) = (float) (cls == 0);
            CV_MAT_ELEM( *prob, float, 0, 1 ) = (float) (cls == 1);
        }
        else
        {
            CV_MAT_ELEM( *prob, float, 0, 0 ) = (float) (cls == 0);
            CV_MAT_ELEM( *prob, float, 1, 0 ) = (float) (cls == 1);
        }
    }

    label = CV_MAT_ELEM( *bt->class_labels, int, 0, cls );

    __END__;

    if( row_sample && (void*) row_sample != (void*) sample->data.ptr )
        cvFree( (void**) &row_sample );

    return (float) label;
}


void
icvReleaseBoostTrainStateMembers( CvBoostTrainState* ts )
{
    if( ts )
    {
        cvFree( (void**) &ts->raw_train_data );
        cvReleaseMat( &ts->responses );
        cvReleaseMat( &ts->weights );
        cvReleaseMat( &ts->weak_train_resp );
        cvReleaseMat( &ts->weak_eval_resp );
        cvReleaseMat( &ts->sum_resp );
        ts->valid = 0;
    }
}


void
cvReleaseBtClassifier( CvStatModel** model )
{
    CV_FUNCNAME( "cvReleaseBtClassifier" );

    __BEGIN__;

    int i;
    CvBtClassifier* bt;
    CvSeqReader reader;
    CvCARTBOOSTClassifier* tree;
    
    if( !model )
        CV_ERROR( CV_StsNullPtr, "NULL double pointer" );
    
    bt = (CvBtClassifier*) *model;
    
    if( !bt )
        EXIT;

    if( !CV_IS_BT_CLASSIFIER(bt) )
        CV_ERROR( CV_StsBadArg, "Invalid Boosted trees model" );

    CV_CALL( cvStartReadSeq( bt->weak, &reader ) );
    for( i = 0; i < bt->weak->total; ++i )
    {
        CV_READ_SEQ_ELEM( tree, reader );
        tree->release( (CvStatModel**) &(tree) );
    }
    CV_CALL( cvReleaseMemStorage( &(bt->weak->storage) ));

    cvReleaseMat( &bt->class_labels );
    cvReleaseMat( &bt->comp_idx );

    icvReleaseBoostTrainStateMembers( (CvBoostTrainState*) bt->ts );

    cvFree( (void**) model );
    *model = NULL;

    __END__;
}



CvBtClassifier*
icvAllocBtClassifier( CvBtClassifierTrainParams* train_params )
{
  //std::cerr<<"Enter icvAllocBtClassifier"<<std::endl;
    CvBtClassifier* bt = NULL;

    CV_FUNCNAME( "icvAllocBtClassifier" );

    __BEGIN__;

    size_t data_size;
    CvStumpTrainParams* stump_params;
    CvCARTBOOSTTrainParams* weak_params;

    CvStumpType types[] = { CV_CLASSIFICATION_CLASS, CV_CLASSIFICATION,
                            CV_REGRESSION, CV_REGRESSION };
    CvStumpError errors[] = { CV_MISCLASSIFICATION, CV_GINI, CV_SQUARE, CV_SQUARE };

    if( train_params )
    {
        if( train_params->boost_type < CV_BT_DISCRETE ||
            train_params->boost_type > CV_BT_GENTLE )
            CV_ERROR( CV_StsBadArg, "Invalid boosting type" );
        if( train_params->num_iter < 0 )
            CV_ERROR( CV_StsBadArg, "Number of iterations must be non-negative" );
        if( train_params->infl_trim_rate <= 0 || train_params->infl_trim_rate > 1 )
            CV_ERROR( CV_StsBadArg, "Influence trimming rate must be in (0, 1]" );
        if( train_params->num_splits < 1 )
            CV_ERROR( CV_StsBadArg, "Number of splits must be positive" );
    }

    data_size = sizeof( *bt ) + sizeof( CvBoostTrainState );
    CV_CALL( bt = (CvBtClassifier*) cvCreateStatModel(
        CV_STAT_MODEL_MAGIC_VAL | CV_BT_CLASSIFIER_MAGIC_VAL, data_size,
        cvReleaseBtClassifier, cvEvalBtClassifier ));

    CV_CALL( bt->weak = cvCreateSeq( 0, sizeof( *(bt->weak) ),
        sizeof( CvCARTBOOSTClassifier* ),  cvCreateMemStorage() ));

    bt->ts = bt + 1;
    
    if( !train_params )
    {
        icvBtDefaultTrainParams( &bt->params );
    }
    else
    {
        bt->params = *train_params;
    }

    stump_params = &((CvBoostTrainState*) bt->ts)->stump_params;
    weak_params = &((CvBoostTrainState*) bt->ts)->weak_params;

    stump_params->type = types[bt->params.boost_type];
    stump_params->error = errors[bt->params.boost_type];

    weak_params->count = bt->params.num_splits;
    weak_params->stumpConstructor = &cvCreateStumpClassifier;
    weak_params->stumpTrainParams = (CvClassifierTrainParams*) stump_params;

    bt->get_value = &cvBtGetParamValue;
    bt->set_value = &cvBtSetParamValue;
    bt->get_mat   = &cvBtGetParamMat;
    bt->set_mat   = &cvBtSetParamMat;

    __END__;

    if( cvGetErrStatus() < 0 )
      {
	//std::cerr<<"icvAllocBtClassifier releasing classifier due to error "<<cvGetErrStatus()<<std::endl;
        cvReleaseBtClassifier( (CvStatModel**) &bt );
      }


    //std::cerr<<"Exit icvAllocBtClassifier"<<std::endl;

    return bt;
}


CvStatModel*
cvCreateBtClassifier( CvMat* train_data,
                      int flags,
                      CvMat* responses,
                      CvStatModelParams* train_params,
                      CvMat* comp_idx,
                      CvMat* sample_idx,
                      CvMat* type_mask,
                      CvMat* missval_mask)
{
  //std::cerr<<"Enter cvCreateBtClassifier"<<std::endl;
  if (sample_idx)
    assert(CV_IS_MAT_HDR(sample_idx));
  //else
    //std::cerr<<"cvCreateBtClassifier: sample_idx is NULL"<<std::endl;
  CvBtClassifier* bt = NULL;
  
  CV_FUNCNAME( "cvCreateBtClassifier" );
  
  __BEGIN__;
  
  CV_CALL( bt = icvAllocBtClassifier( (CvBtClassifierTrainParams*) train_params ));
  
  CV_CALL( cvUpdateBtClassifier( bt, train_data, flags, responses,
				 NULL /* train_params */, comp_idx, sample_idx, type_mask, missval_mask ));
  
  __END__;
  
  if( cvGetErrStatus() < 0 )
    {
      //std::cerr<<"Releasing classifier due to error"<<std::endl;
      //std::cerr<<"Error status: "<<cvGetErrStatus()<<std::endl;
      cvReleaseBtClassifier( (CvStatModel**) &bt );
    }
  
  //std::cerr<<"Exit cvCreateBtClassifier"<<std::endl;
  return (CvStatModel*) bt;
}

void
cvUpdateBtClassifier( CvBtClassifier* model,
                      CvMat* train_data,
                      int flags,
                      CvMat* responses,
                      CvStatModelParams* train_params,
                      CvMat* comp_idx,
                      CvMat* sample_idx,
                      CvMat* type_mask,
                      CvMat* missval_mask)
{
  if (sample_idx)
    assert(CV_IS_MAT_HDR(sample_idx));
  //else
    //std::cerr<<"cvUpdateBtClassifier: sample_idx is NULL"<<std::endl;

  //std::cerr<<"Enter cvUpdateBtClassifier"<<std::endl;
  
  
  CV_FUNCNAME( "cvUpdateBtClassifier" );
  __BEGIN__;
  
  CvBtClassifier* bt = (CvBtClassifier*) model;
  CvBoostTrainState* ts = (CvBoostTrainState*) bt->ts;
  
  int i, j;
  CvSeqWriter writer;
  CvCARTBOOSTClassifier* weak = NULL;
  
  if( !CV_IS_BT_CLASSIFIER( bt ) )
    CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
  if( train_params )
    CV_ERROR( CV_StsError, "Train parameters are not expected and must be NULL" );
  if( missval_mask )
    CV_ERROR( CV_StsError, "Missed values are not supported and must be NULL" );
  
  if( train_data == NULL )
    {
      /* update the model using old training data */
      if( responses || comp_idx || sample_idx || type_mask )
	CV_ERROR( CV_StsBadArg, "train_data, responses, comp_idx, sample_idx and "
		  "type_mask must be NULL simultaneously" );

      if( bt->params.num_iter <= 0 )
        {
	  //std::cerr<<"cvUpdateBtClassifier is releasing state members"<<std::endl;
	  /* release extra */
	  icvReleaseBoostTrainStateMembers( ts );
	  //std::cerr<<"Done releasing"<<std::endl;
        }
    }
  else
    {
      /* new training data */
      CV_CALL( icvBtNewTrainData( bt, train_data, flags, responses,
				  comp_idx, sample_idx, type_mask, missval_mask ) );
    }
  if( bt->params.num_iter <= 0 )
    EXIT;
  if( !ts->raw_train_data )
    CV_ERROR( CV_StsBadArg, "Training data set must be specified" );
  
  if( !ts->valid )
    {
      if( !ts->weights || ts->weights->cols != ts->responses->cols )
        {
	  cvReleaseMat( &ts->weights );
	  CV_CALL( ts->weights = cvCreateMat( 1, ts->responses->cols, CV_32FC1 ));
	  CV_CALL( cvSet( ts->weights, cvScalar( 1.0 / ts->responses->cols ) ));
        }
      if( !ts->weak_train_resp || ts->weak_train_resp->cols != ts->responses->cols )
        {
	  cvReleaseMat( &ts->weak_train_resp );
	  CV_CALL( ts->weak_train_resp=cvCreateMat( 1, ts->responses->cols, CV_32FC1 ));
	  CV_CALL( cvSet( ts->weak_train_resp, cvScalar(0) ));
        }
      if( !ts->weak_eval_resp || ts->weak_eval_resp->cols != ts->responses->cols )
        {
	  cvReleaseMat( &ts->weak_eval_resp );
	  CV_CALL( ts->weak_eval_resp = cvCreateMat( 1, ts->responses->cols, CV_32FC1));
	  CV_CALL( cvSet( ts->weak_eval_resp, cvScalar(0) ));
        }
      
      if( bt->params.boost_type == CV_BT_LOGIT )
        {
	  CvSeqReader reader;
	  
	  if( !ts->sum_resp || ts->sum_resp->cols != ts->responses->cols )
            {
	      //std::cerr<<"I am releasing something"<<std::endl;
	      cvReleaseMat( &ts->sum_resp );
	      //std::cerr<<"Yay, done"<<std::endl;
	      CV_CALL( ts->sum_resp = cvCreateMat( 1, ts->responses->cols, CV_32FC1 ));
	      CV_CALL( cvSet( ts->sum_resp, cvScalar(0) ));
            }
	  CV_CALL( cvSet( ts->sum_resp, cvScalar(0) ) );
	  CV_CALL( cvStartReadSeq( bt->weak, &reader ) );
	  if( CV_IS_ROW_SAMPLE( ts->flags ) )
            {
	      for( i = 0; i < bt->weak->total; ++i )
                {
		  CV_READ_SEQ_ELEM( weak, reader );
		  for( j = 0; j < ts->responses->cols; ++j )
                    {
		      CV_MAT_ELEM( *ts->sum_resp, float, 0, j ) += 
			weak->val[icvEvalCARTBOOSTRow( weak, ts->raw_train_data[j] )];
                    }
                }
            }
	  else
            {
	      ptrdiff_t step = (char*) ts->raw_train_data[1] -
		(char*) ts->raw_train_data[0];
	      
	      for( i = 0; i < bt->weak->total; ++i )
                {
		  CV_READ_SEQ_ELEM( weak, reader );
		  for( j = 0; j < ts->responses->cols; ++j )
                    {
		      CV_MAT_ELEM( *ts->sum_resp, float, 0, j ) += 
			weak->val[icvEvalCARTBOOSTCol( weak, ts->raw_train_data[j],
						       step )];
                    }
                }
            }
        }
      else
        {
	  //std::cerr<<"Releasing some matrix"<<std::endl;
	  cvReleaseMat( &ts->sum_resp );
	  //std::cerr<<"Releasing was successful"<<std::endl;
        }
      
      CV_CALL( icvBoostIterate( NULL, ts->responses, ts->weak_train_resp, ts->weights,
				bt->params.boost_type, ts->sum_resp ) );
      ts->valid = 1;
    }
  /* perform iteration(s) */
  CV_CALL( cvStartAppendToSeq( bt->weak, &writer ) );
  for( i = 0; i < bt->params.num_iter; ++i )
    {
      //std::cerr<<"Iteration "<<i<<std::endl;

      CvMat* sample_idx = NULL;        
      double c = 0;
      
      /* create sample subset by influence trimming */
      if( bt->params.infl_trim_rate < 1.0 )
        {
	  //std::cerr<<"About to call trim weights"<<std::endl;
	  CV_CALL( sample_idx = cvTrimWeights( ts->weights, NULL,
					       (float) bt->params.infl_trim_rate ) );
	  //std::cerr<<"Trim weights succeeded"<<std::endl;
        }
      
      /* train weak classifier */
      //std::cerr<<"A"<<std::endl;
      if (sample_idx)
	assert(CV_IS_MAT_HDR(sample_idx));
      CV_CALL( weak = (CvCARTBOOSTClassifier*) cvCreateCARTBOOSTClassifier(
									   &ts->train_data, ts->flags, ts->weak_train_resp, NULL /* type_mask */,
									   NULL /* missval_mask */, NULL /* comp_idx */, sample_idx, ts->weights,
									   (CvClassifierTrainParams*) &ts->weak_params ) );
 
      //std::cerr<<"B"<<std::endl;
     
      if( bt->params.boost_type == CV_BT_REAL )
        {
	  for( j = 0; j <= weak->count; ++j )
	    weak->val[j] = cvLogRatio( weak->val[j] );
        }

      //std::cerr<<"C"<<std::endl;
      
      if( CV_IS_ROW_SAMPLE( ts->flags ) )
        {
	  for( j = 0; j < ts->responses->cols; ++j )
            {
	      CV_MAT_ELEM( *ts->weak_eval_resp, float, 0, j ) = 
		weak->val[icvEvalCARTBOOSTRow( weak, ts->raw_train_data[j] )];
            }
        }
      else
        {
	  ptrdiff_t step = (char*)ts->raw_train_data[1] - (char*)ts->raw_train_data[0];
	  
	  for( j = 0; j < ts->responses->cols; ++j )
            {
	      CV_MAT_ELEM( *ts->weak_eval_resp, float, 0, j ) = 
		weak->val[icvEvalCARTBOOSTCol( weak, ts->raw_train_data[0]+j, step)];
            }
        }

      //std::cerr<<"D"<<std::endl;
      
      /* update boosting training state */
      CV_CALL( c = icvBoostIterate( ts->weak_eval_resp, ts->responses,
				    ts->weak_train_resp, ts->weights, bt->params.boost_type, ts->sum_resp ) );
      
      for( j = 0; j <= weak->count; ++j )
	weak->val[j] *= (float) c;
      
      //std::cerr<<"E"<<std::endl;


      CV_WRITE_SEQ_ELEM( weak, writer );
      
      /* clean up */
      //std::cerr<<"Cleaning up"<<std::endl;
      cvReleaseMat( &sample_idx );
      //std::cerr<<"Done cleaning up."<<std::endl;
    }
  

    //std::cerr<<"About to call cvEndWriteSeq"<<std::endl;
    CV_CALL( cvEndWriteSeq( &writer ) );


    //std::cerr<<"Exit update"<<std::endl;
    __END__;
}

CVAPI(float)
cvEvalWeakClassifiers( const CvBtClassifier* bt, const CvMat* sample,
                       CvMat* weak_vals, CvSlice slice, int eval_type )
{
    float val = 0;
    float* row_sample = NULL;

    CV_FUNCNAME( "cvEvalWeakClassifiers" );

    __BEGIN__;

    int total = 0;
    CvSeqReader reader;
    CvCARTBOOSTClassifier* tree;
    int i;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");
    if( slice.start_index < 0 || slice.start_index > bt->weak->total )
        CV_ERROR( CV_StsBadArg, "start index is out of range" );
    slice.end_index = MIN( slice.end_index, bt->weak->total );
    if( slice.end_index < slice.start_index )
        CV_ERROR( CV_StsBadArg, "end index is out of range" );
    total = slice.end_index - slice.start_index;
    if( total <= 0 )
        EXIT;
    if( eval_type != CV_BT_VALUE && eval_type != CV_BT_INDEX )
        CV_ERROR( CV_StsBadArg, "Invalid eval_type" );
    
    if( weak_vals && (!CV_IS_MAT( weak_vals ) ||
            ( eval_type == CV_BT_VALUE && CV_MAT_TYPE( weak_vals->type ) != CV_32FC1 ) ||
            ( eval_type == CV_BT_INDEX && CV_MAT_TYPE( weak_vals->type ) != CV_32SC1 )) )
        CV_ERROR( CV_StsBadArg, "weak_vals must be NULL or matrix of type 32fC1"
            " for values or 32fC1 for indices" );
    if( weak_vals && eval_type == CV_BT_VALUE && 
            (weak_vals->rows != 1 || (weak_vals->cols != 1 && weak_vals->cols != total)))
        CV_ERROR( CV_StsBadArg, "weak_vals matrix must be NULL, 1 by 1 or "
            "1 by <number_of_evaluated_classifiers>" );
    if( weak_vals && eval_type == CV_BT_INDEX && 
            (weak_vals->rows != 1 || weak_vals->cols != total ))
        CV_ERROR( CV_StsBadArg, "weak_vals matrix must be NULL or "
            "1 by <number_of_evaluated_classifiers>" );
    if( !weak_vals && eval_type == CV_BT_INDEX && total != 1 )
        CV_ERROR( CV_StsNullPtr, "weak_vals matrix must be specified" );

    CV_CALL( cvPreparePredictData( sample, bt->total_features, bt->comp_idx,
        bt->class_labels->cols, NULL, &row_sample ));
    
    CV_CALL( cvStartReadSeq( bt->weak, &reader ) );
    CV_CALL( cvSetSeqReaderPos( &reader, slice.start_index ) );

    val = 0;    
    if( eval_type == CV_BT_VALUE )
    {
        if( weak_vals && weak_vals->cols > 1 )
        {
            float weak_val;
            for( i = 0; i < total; ++i )
            {
                CV_READ_SEQ_ELEM( tree, reader );
                weak_val = tree->val[icvEvalCARTBOOSTRow( tree, row_sample )];
                CV_MAT_ELEM( *weak_vals, float, 0, i ) = weak_val;
                val += weak_val;
            }
        }
        else
        {
            for( i = 0; i < total; ++i )
            {
                CV_READ_SEQ_ELEM( tree, reader );
                val += tree->val[icvEvalCARTBOOSTRow( tree, row_sample )];
            }
            if( weak_vals )
            {
                CV_MAT_ELEM( *weak_vals, float, 0, 0 ) = val;
            }
        }
    }
    else
    {
        /* CV_BT_INDEX */
        if( weak_vals )
        {
            for( i = 0; i < total; ++i )
            {
                CV_READ_SEQ_ELEM( tree, reader );
                CV_MAT_ELEM( *weak_vals, int, 0, i ) = 
                    icvEvalCARTBOOSTRow( tree, row_sample );
            }
            val = (float) CV_MAT_ELEM( *weak_vals, int, 0, 0 );
        }
        else
        {
            CV_READ_SEQ_ELEM( tree, reader );
            val = (float) icvEvalCARTBOOSTRow( tree, row_sample );
        }
    }

    __END__;

    if( row_sample && (void*) row_sample != (void*) sample->data.ptr )
        cvFree( (void**) &row_sample );

    return val;
}

int
cvIsBtClassifier( const void* struct_ptr )
{
    //CV_FUNCNAME( "cvIsBtClassifier" );

    __BEGIN__;

    return CV_IS_BT_CLASSIFIER( struct_ptr );

    __END__;
}

static struct { int type; char* name; } icvBtBoostTypeMap[] =
{
    {CV_BT_DISCRETE, "Discrete AdaBoost"},
    {CV_BT_REAL,     "Real AdaBoost"},
    {CV_BT_LOGIT,    "LogitBoost"},
    {CV_BT_GENTLE,   "Gentle AdaBoost"},
    /* dummy last line */
    {-1,             NULL}
};

char* icvBoostTypeToName( int boost_type )
{
    int i;
    for( i = 0; icvBtBoostTypeMap[i].name != NULL; ++i )
    {
        if( icvBtBoostTypeMap[i].type == boost_type ) break;
    }
    return icvBtBoostTypeMap[i].name;
}



/* returns -1 if fails */
int icvNameToBoostType( const char* boost_name )
{
    int i;
    for( i = 0; icvBtBoostTypeMap[i].name != NULL; ++i )
    {
        if( !strcmp( icvBtBoostTypeMap[i].name, boost_name ) ) break;
    }

    return icvBtBoostTypeMap[i].type;
}


void*
cvReadBtClassifier( CvFileStorage* storage, CvFileNode* node )
{
    CvBtClassifier* bt = 0;
    
    CV_FUNCNAME( "cvReadBtClassifier" );

    __BEGIN__;
    const char* boost_name = NULL;
    CvBtClassifierTrainParams params;
    CvFileNode* params_fn = NULL;
    CvFileNode* trees_fn = NULL;
    void* ptr = NULL;
    CvSeqReader trees_fn_reader;
    CvSeqWriter weak_writer;
    char buf[256];
    int i, j;

    assert( storage );
    assert( node );

    CV_CALL( params_fn = cvGetFileNodeByName( storage, node, ICV_BT_PARAMS_NAME ) );
    if( !params_fn || !CV_NODE_IS_MAP( params_fn->tag ) )
        CV_ERROR( CV_StsError, "Invalid params" );

    CV_CALL( boost_name = cvReadStringByName( storage, params_fn,
        ICV_BT_BOOST_TYPE_NAME, NULL ));
    params.boost_type = icvNameToBoostType( boost_name );
    if( params.boost_type == -1 )
        CV_ERROR( CV_StsError, "Invalid params.boost_type" );
    CV_CALL( params.num_iter = cvReadIntByName( storage, params_fn,
        ICV_BT_NUM_ITER_NAME, -1 ));
    if( params.num_iter < 0 )
        CV_ERROR( CV_StsError, "Invalid params.num_iter" );
    CV_CALL( params.infl_trim_rate = cvReadRealByName( storage, params_fn,
        ICV_BT_INFL_TRIM_NAME, -1 ));
    if( params.infl_trim_rate <= 0 || params.infl_trim_rate > 1 )
        CV_ERROR( CV_StsError, "Invalid params.infl_trim_rate" );
    CV_CALL( params.num_splits = cvReadIntByName( storage, params_fn,
        ICV_BT_NUM_SPLITS_NAME, -1 ));
    if( params.num_splits <= 0 )
        CV_ERROR( CV_StsError, "Invalid params.num_splits" );
    
    CV_CALL( bt = icvAllocBtClassifier( &params ) );

    CV_CALL( ptr = cvReadByName( storage, node, ICV_BT_CLASS_LABELS_NAME ));
    if( !CV_IS_MAT( ptr ) )
    {
        cvRelease( &ptr );
        CV_ERROR( CV_StsError, "Invalid class_labels" );
    }
    bt->class_labels = (CvMat*) ptr;

    CV_CALL( bt->total_features = cvReadIntByName( storage, node,
        ICV_BT_TOTAL_FEATURES_NAME, -1 ));
    if( bt->total_features <= 0 )
        CV_ERROR( CV_StsError, "Invalid total_features" );

    //fprintf(stderr, "cvReadBtClassifier: reading indices\n");
    CV_CALL( ptr = cvReadByName( storage, node, ICV_BT_COMP_IDX_NAME ));
    if( ptr != NULL && !CV_IS_MAT( ptr ) )
    {
        cvRelease( &ptr );
        CV_ERROR( CV_StsError, "Invalid comp_idx" );
    }
    bt->comp_idx = (CvMat*) ptr;

    CV_CALL( ptr = cvReadByName( storage, node, ICV_BT_WEIGHTS_NAME ));
    if( ptr != NULL && !CV_IS_MAT( ptr ) )
    {
        cvRelease( &ptr );
        CV_ERROR( CV_StsError, "Invalid comp_idx" );
    }
    ((CvBoostTrainState*) bt->ts)->weights = (CvMat*) ptr;

    CV_CALL( trees_fn = cvGetFileNodeByName( storage, node, ICV_BT_TREES_NAME ) );
    if( !trees_fn || !CV_NODE_IS_SEQ( trees_fn->tag ) )
        CV_ERROR( CV_StsError, "Invalid trees" );
    CV_CALL( cvStartReadSeq( trees_fn->data.seq, &trees_fn_reader ));
    CV_CALL( cvStartAppendToSeq( bt->weak , &weak_writer ));
    for( i = 0; i < trees_fn->data.seq->total; ++i )
    {        
        CvFileNode* tree_fn = NULL;
        CvSeqReader tree_fn_reader;
        CvCARTBOOSTClassifier* tree = NULL;
        int next_leaf_idx = 0;

        tree_fn = (CvFileNode*) trees_fn_reader.ptr;

        if( !tree_fn || !CV_NODE_IS_SEQ( tree_fn->tag ) ||
                tree_fn->data.seq->total < 1 ||
                tree_fn->data.seq->total > bt->params.num_splits )
        {
            sprintf( buf, "Invalid tree %d", i );
            CV_ERROR( CV_StsError, buf );
        }

        CV_CALL( tree = icvAllocCARTBOOSTClassifier( bt->params.num_splits ) );
        CV_WRITE_SEQ_ELEM( tree, weak_writer );
        CV_CALL( cvStartReadSeq( tree_fn->data.seq, &tree_fn_reader ));
        for( j = 0; j < tree_fn->data.seq->total; ++j )
        {
            CvFileNode* node_fn = NULL;

            int comp_idx = -1;
            double threshold = DBL_MAX;
            int left = -1;
            int right = -1;
            double left_val = DBL_MAX;
            double right_val = DBL_MAX;

            node_fn = (CvFileNode*) tree_fn_reader.ptr;
            if( !node_fn || !CV_NODE_IS_MAP( node_fn->tag ) )
            {
                sprintf( buf, "Invalid node %d of tree %d", j, i );
                CV_ERROR( CV_StsError, buf );
            }

            CV_CALL( comp_idx = cvReadIntByName( storage, node_fn,
                ICV_BT_COMP_IDX_NAME, -1 ));
            CV_CALL( threshold = cvReadRealByName( storage, node_fn,
                ICV_BT_THRESHOLD_NAME, DBL_MAX ));

            CV_CALL( left = cvReadIntByName( storage, node_fn,
                ICV_BT_LEFT_NODE_NAME, -1 ));
            CV_CALL( left_val = cvReadRealByName( storage, node_fn,
                ICV_BT_LEFT_VAL_NAME, DBL_MAX ));

            CV_CALL( right = cvReadIntByName( storage, node_fn,
                ICV_BT_RIGHT_NODE_NAME, -1 ));
            CV_CALL( right_val = cvReadRealByName( storage, node_fn,
                ICV_BT_RIGHT_VAL_NAME, DBL_MAX ));
            
            if( comp_idx < 0 || comp_idx >= bt->total_features )
            {
                sprintf( buf, "Invalid node %d of tree %d: bad comp_idx", j, i );
                CV_ERROR( CV_StsError, buf );
            }
            if( threshold == DBL_MAX )
            {
                sprintf( buf, "Invalid node %d of tree %d: bad threshold", j, i );
                CV_ERROR( CV_StsError, buf );
            }
            if( (left == -1 && left_val == DBL_MAX) ||
                (left != -1 && (left < 1 || left >= bt->params.num_splits)) )
            {
                sprintf( buf, "Invalid node %d of tree %d: bad left branch", j, i );
                CV_ERROR( CV_StsError, buf );
            }
            if( (right == -1 && right_val == DBL_MAX) ||
                (right != -1 && (right < 1 || right >= bt->params.num_splits)) )
            {
                sprintf( buf, "Invalid node %d of tree %d: bad right branch", j, i );
                CV_ERROR( CV_StsError, buf );
            }

            tree->comp_idx[j] = comp_idx;
            tree->threshold[j] = (float)threshold;
            if( left == -1 )
            {
                if( next_leaf_idx > bt->params.num_splits )
                {
                    sprintf( buf, "Invalid tree %d: too many leaves", i );
                    CV_ERROR( CV_StsError, buf );
                }
                tree->left[j] = -next_leaf_idx;
                tree->val[next_leaf_idx++] = (float) left_val;
            }
            else
            {
                tree->left[j] = left;
            }
            if( right == -1 )
            {
                if( next_leaf_idx > bt->params.num_splits )
                {
                    sprintf( buf, "Invalid tree %d: too many leaves", i );
                    CV_ERROR( CV_StsError, buf );
                }
                tree->right[j] = -next_leaf_idx;
                tree->val[next_leaf_idx++] = (float) right_val;
            }
            else
            {
                tree->right[j] = right;
            }
            CV_NEXT_SEQ_ELEM( sizeof( *node_fn ), tree_fn_reader );
        }
        CV_NEXT_SEQ_ELEM( sizeof( *tree_fn ), trees_fn_reader );
    } /* for each tree */
    CV_CALL( cvEndWriteSeq( &weak_writer ) );

    __END__;

    if( cvGetErrStatus() < 0 )
        cvReleaseBtClassifier( (CvStatModel**) &bt );

    return bt;
}


void
cvWriteBtClassifier( CvFileStorage* storage, const char* name, const void* struct_ptr,
                     CvAttrList attributes )
{
    CV_FUNCNAME( "cvWriteBtClassifier" );

    __BEGIN__;

    int i, j;
    const CvBtClassifier* bt = (const CvBtClassifier*) struct_ptr;
    CvSeqReader reader;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");

    /* TODO: add integrity checks */

    CV_CALL( cvStartWriteStruct( storage, name, CV_NODE_MAP,
                                 cvTypeOf( struct_ptr )->type_name, attributes ));

    /* params */
    CV_CALL( cvStartWriteStruct( storage, ICV_BT_PARAMS_NAME, CV_NODE_MAP ));
    CV_CALL( cvWriteString( storage, ICV_BT_BOOST_TYPE_NAME,
                            icvBoostTypeToName( bt->params.boost_type ), 0 ));
    CV_CALL( cvWriteInt( storage, ICV_BT_NUM_ITER_NAME, bt->params.num_iter ));
    CV_CALL( cvWriteReal( storage, ICV_BT_INFL_TRIM_NAME, bt->params.infl_trim_rate ));
    CV_CALL( cvWriteInt( storage, ICV_BT_NUM_SPLITS_NAME, bt->params.num_splits ));
    CV_CALL( cvEndWriteStruct( storage )); /* params */
    CV_CALL( cvWrite( storage, ICV_BT_CLASS_LABELS_NAME, bt->class_labels ));
    CV_CALL( cvWriteInt( storage, ICV_BT_TOTAL_FEATURES_NAME, bt->total_features ));
    if( bt->comp_idx )
        CV_CALL( cvWrite( storage, ICV_BT_COMP_IDX_NAME, bt->comp_idx ));
    if( ((CvBoostTrainState*) bt->ts)->weights && ((CvBoostTrainState*) bt->ts)->valid )
        CV_CALL( cvWrite( storage, ICV_BT_WEIGHTS_NAME,
            ((CvBoostTrainState*) bt->ts)->weights ));
    
    /* write sequence of trees */
    CV_CALL( cvStartWriteStruct( storage, ICV_BT_TREES_NAME, CV_NODE_SEQ ));
    CV_CALL( cvStartReadSeq( bt->weak, &reader ));
    for( i = 0; i < bt->weak->total; ++i )
    {
        const CvCARTBOOSTClassifier* tree = NULL;
        char comment[256];

        CV_CALL( cvStartWriteStruct( storage, NULL, CV_NODE_SEQ ));
        sprintf( comment, "tree %d", i );
        CV_CALL( cvWriteComment( storage, comment, 1 ));

        CV_READ_SEQ_ELEM( tree, reader );

        for( j = 0; j < tree->count; ++j )
        {
            CV_CALL( cvStartWriteStruct( storage, NULL, CV_NODE_MAP ));
            if( j ) sprintf( comment, "node %d", j );
            else sprintf( comment, "root node %d", j );
            CV_CALL( cvWriteComment( storage, comment, 1 ));
            CV_CALL( cvWriteInt( storage, ICV_BT_COMP_IDX_NAME, tree->comp_idx[j] ));
            CV_CALL( cvWriteReal( storage, ICV_BT_THRESHOLD_NAME, tree->threshold[j] ));
            if( tree->left[j] > 0 )
            {
                CV_CALL( cvWriteInt( storage, ICV_BT_LEFT_NODE_NAME, tree->left[j] ));
            }
            else
            {
                CV_CALL( cvWriteReal( storage, ICV_BT_LEFT_VAL_NAME,
                                      tree->val[-tree->left[j]] ));
            }
            if( tree->right[j] > 0 )
            {
                CV_CALL( cvWriteInt( storage, ICV_BT_RIGHT_NODE_NAME, tree->right[j] ));
            }
            else
            {
                CV_CALL( cvWriteReal( storage, ICV_BT_RIGHT_VAL_NAME,
                                      tree->val[-tree->right[j]] ));
            }
            CV_CALL( cvEndWriteStruct( storage )); /* node */
        }

        CV_CALL( cvEndWriteStruct( storage )); /* tree */
    }

    CV_CALL( cvEndWriteStruct( storage )); /* trees */
        
    CV_CALL( cvEndWriteStruct( storage )); /* bt classifier */

    __END__;
}

void*
cvCloneBtClassifier( const void* struct_ptr )
{
    CvBtClassifier* out = NULL;
    
    CV_FUNCNAME( "cvCloneBtClassifier" );

    __BEGIN__;

    CvBtClassifier* bt = (CvBtClassifier*) struct_ptr;
    CvSeqReader reader;
    CvSeqWriter writer;
    int i, j;

    if( !CV_IS_BT_CLASSIFIER( bt ) )
        CV_ERROR( bt ? CV_StsBadArg : CV_StsNullPtr, "Invalid boosted trees model");

    CV_CALL( out = icvAllocBtClassifier( &bt->params ) );

    CV_CALL( out->class_labels = cvCloneMat( bt->class_labels ) );
    out->total_features = bt->total_features;
    if( bt->comp_idx )
        CV_CALL( bt->comp_idx = cvCloneMat( bt->comp_idx ) );
    CV_CALL( cvStartReadSeq( bt->weak, &reader ) );
    CV_CALL( cvStartAppendToSeq( out->weak, &writer ) );
    for( i = 0; i < bt->weak->total; ++i )
    {
        CvCARTBOOSTClassifier* weak = NULL;
        CvCARTBOOSTClassifier* out_weak = NULL;

        CV_READ_SEQ_ELEM( weak, reader );
        CV_CALL( out_weak = icvAllocCARTBOOSTClassifier( bt->params.num_splits ) );
        out_weak->count = weak->count;
        for( j = 0; j < weak->count; ++j )
        {
            out_weak->comp_idx[j] = weak->comp_idx[j];
            out_weak->threshold[j] = weak->threshold[j];
            out_weak->left[j] = weak->left[j];
            out_weak->right[j] = weak->right[j];
            out_weak->val[j] = weak->val[j];
        }
        out_weak->val[j] = weak->val[j];
        CV_WRITE_SEQ_ELEM( out_weak, writer );
    }
    CV_CALL( cvEndWriteSeq( &writer ) );

    /* TODO write train state clone code */

    __END__;
    if( cvGetErrStatus() < 0 && out )
        out->release( (CvStatModel**) &out );

    return out;
}


int
icvRegisterBtClassifierType()
{
    //CV_FUNCNAME( "icvRegisterBtClassifierType" );

    __BEGIN__;

    CvTypeInfo info;

    info.header_size = sizeof( info );
    info.is_instance = cvIsBtClassifier;
    info.release = (CvReleaseFunc) cvReleaseBtClassifier;
    info.read = cvReadBtClassifier;
    info.write = cvWriteBtClassifier;
    info.clone = cvCloneBtClassifier;
    info.type_name = CV_TYPE_NAME_ML_BOOSTING;
    cvRegisterType( &info );

    __END__;

    return 1;
}

static int icv_register_bt_classifier_type = icvRegisterBtClassifierType();

}
