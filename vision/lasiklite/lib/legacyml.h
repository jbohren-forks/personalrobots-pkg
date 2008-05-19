#pragma once

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

namespace legacy {

#define CV_STAT_MODEL_PARAM_FIELDS() \
	    int flags

#define CV_BT_DISCRETE   0
#define CV_BT_REAL       1
#define CV_BT_LOGIT      2
#define CV_BT_GENTLE     3

#ifndef CV_ROW_SAMPLE
#define CV_ROW_SAMPLE 1
#endif

#define CV_MAGIC_VAL 0xFFFF0000
#define CV_STAT_MODEL_MAGIC_VAL 0x77440000
#define CV_CLASSIFIER_MAGIC_VAL CV_STAT_MODEL_MAGIC_VAL
#define CV_BT_CLASSIFIER_MAGIC_VAL      0x00006789

typedef enum CvStumpType
{
	CV_CLASSIFICATION       = 0,
	CV_CLASSIFICATION_CLASS = 1,
	CV_REGRESSION           = 2
} CvStumpType;


typedef enum CvStumpError
{
	CV_MISCLASSIFICATION = 0,
	CV_GINI              = 1,
	CV_ENTROPY           = 2,
	CV_SQUARE            = 3
} CvStumpError;


typedef struct CvStumpTrainParams
{
	CV_STAT_MODEL_PARAM_FIELDS();
	CvStumpType  type;
	CvStumpError error;
} CvStumpTrainParams;


typedef struct CvStatModelParams
{
	    CV_STAT_MODEL_PARAM_FIELDS();
} CvStatModelParams;

struct CvStatModel;

typedef float (CV_CDECL *CvStatModelPredict)
    ( const struct CvStatModel* model, const CvMat* sample, CvMat* probs /* =0 */ );

typedef void (CV_CDECL *CvStatModelUpdate)
    ( struct CvStatModel* model, const CvMat* features, int sample_t_flag,
      const CvMat* responses, const CvStatModelParams* params,
      const CvMat* comp_idx /* =0 */,
      const CvMat* sample_idx /* =0 */,
      const CvMat* type_mask /* =0 */,
      const CvMat* missing_value_mask /* =0 */ );

typedef void (CV_CDECL *CvStatModelRelease)( struct CvStatModel** model );

typedef void* (CV_CDECL *CvStatModelGetObject)
    ( struct CvStatModel* model, const void* obj, int objType,
      int objIndex /* =-1 */);

typedef double (CV_CDECL *CvStatModelGetParamValue)
    ( struct CvStatModel* model, const void* obj, int paramType,
      int paramIndex /* =-1 */);

typedef void (CV_CDECL *CvStatModelSetParamValue)
    ( struct CvStatModel* model, const void* obj, int paramType,
      int paramIndex /* =-1 */,
      double paramValue /* =0 */);

typedef void (CV_CDECL *CvStatModelGetParamMat)
    ( struct CvStatModel* model, const void* obj, int paramType,
      int paramIndex, /* =-1 */
      CvMat* paramMat /* =NULL */);

typedef void (CV_CDECL *CvStatModelSetParamMat)
    ( struct CvStatModel* model, const void* obj, int paramType,
      int paramIndex, /* =-1 */
      CvMat* paramMat /* =NULL */);



#define CV_STAT_MODEL_FIELDS()                  \
	    int   flags;                                \
    int   header_size;                          \
    CvStatModelRelease          release;        \
    CvStatModelPredict          predict;        \
    CvStatModelUpdate           update;         \
    CvStatModelGetObject        get_object;     \
    CvStatModelGetParamValue    get_value;      \
    CvStatModelSetParamValue    set_value;      \
    CvStatModelGetParamMat      get_mat;        \
    CvStatModelSetParamMat      set_mat


typedef struct CvStatModel
{
    CV_STAT_MODEL_FIELDS();
} CvStatModel;


typedef CvStatModelParams CvClassifierTrainParams;

typedef CvStatModel CvClassifier; // for compatibility


typedef CvClassifier* (*CvClassifierConstructor)( CvMat*, int, CvMat*, CvMat*, CvMat*,
		CvMat*, CvMat*, CvMat*, CvClassifierTrainParams* );


typedef struct CvCARTBOOSTTrainParams
{
	CV_STAT_MODEL_PARAM_FIELDS();
	/* desired number of internal nodes */
	int count;
	CvClassifierTrainParams* stumpTrainParams;
	CvClassifierConstructor  stumpConstructor;

	/*
	 *      * Split sample indices <idx>
	 *           * on the "left" indices <left> and "right" indices <right>
	 *                * according to samples components <compidx> values and <threshold>.
	 *                     *
	 *                          * NOTE: Matrices <left> and <right> must be allocated using cvCreateMat function
	 *                               *   since they are freed using cvReleaseMat function
	 *                                    *
	 *                                         * If it is NULL then the default implementation which evaluates training
	 *                                              * samples from <trainData> passed to classifier constructor is used
	 *                                                   */

	void (*splitIdx)( int compidx, float threshold,
			CvMat* idx, CvMat** left, CvMat** right,
			void* userdata );
	void* userdata;
} CvCARTBOOSTTrainParams;


typedef struct CvBoostTrainState
{
	/* input defragmented training set */
	const float** raw_train_data; /* train_data is here */
	CvMat  train_data;            /* mat header for raw_train_data */
	int flags;
	CvMat* responses;             /* CV_32SC1. 0 or 1 values */

	CvStumpTrainParams stump_params;
	CvCARTBOOSTTrainParams weak_params;

	int valid;              /* are weak_train_resp and sum_resp valid? */
	CvMat* weights;
	CvMat* weak_train_resp; /* responses for weak classifier training */
	CvMat* weak_eval_resp;  /* last weak evaluation responses */
	CvMat* sum_resp;        /* only for CV_BT_LOGIT */
}
CvBoostTrainState;


typedef struct CvBtClassifierTrainParams
{
	CV_STAT_MODEL_PARAM_FIELDS();
	int boost_type;
	int num_iter;
	double infl_trim_rate;
	int num_splits;
}
CvBtClassifierTrainParams;

typedef struct CvBtClassifier
{
	CV_STAT_MODEL_FIELDS();
	CvBtClassifierTrainParams params;
	CvMat* class_labels;
	int total_features;
	CvSeq* weak; /* weak classifiers (CvCARTBOOSTClassifier) pointers */
	CvMat* comp_idx;
	void* ts;
}
CvBtClassifier;




CvStatModel*
cvCreateBtClassifier( CvMat* train_data,
		int flags,
		CvMat* responses,
		CvStatModelParams* train_params CV_DEFAULT(0),
		CvMat* comp_idx CV_DEFAULT(0),
		CvMat* sample_idx CV_DEFAULT(0),
		CvMat* type_mask CV_DEFAULT(0),
		CvMat* missval_mask CV_DEFAULT(0));

typedef struct CvCARTBOOSTClassifier
{
	CV_STAT_MODEL_FIELDS();
	/* number of internal tree nodes (splits) */
	int count;

	/* internal nodes (each is array of <count> elements) */
	int* comp_idx;
	float* threshold;
	int* left;
	int* right;

	/* leaves (array of <count>+1 elements) */
	float* val;
}
CvCARTBOOSTClassifier;


#define CV_IS_STAT_MODEL( model )                                          \
    ( ((model) != NULL) && ((((CvStatModel*)(model))->flags & CV_MAGIC_VAL)\
        == CV_STAT_MODEL_MAGIC_VAL ))

#define CV_IS_CLASSIFIER  CV_IS_STAT_MODEL


#define CV_IS_BT_CLASSIFIER( model )                                       \
    ( (CV_IS_CLASSIFIER( model )) && (((CvStatModel*) (model))->flags      \
        & ~CV_MAGIC_VAL) == CV_BT_CLASSIFIER_MAGIC_VAL )

#ifndef CV_IS_ROW_SAMPLE
#define CV_IS_ROW_SAMPLE( flags ) ( ( flags ) & CV_ROW_SAMPLE )
#endif

typedef struct CvSparseVecElem32f
{
	    int idx;
			    float val;
}
CvSparseVecElem32f;


typedef struct CvStumpClassifier
{
    CV_STAT_MODEL_FIELDS();
    int compidx;
    
    float lerror; /* impurity of the right node */
    float rerror; /* impurity of the left  node */
    
    float threshold;
    float left;
    float right;
} CvStumpClassifier;


void
cvPreparePredictData( const CvArr* sample, int dims_all, const CvMat* comp_idx,
		                      int class_count, const CvMat* prob, float** row_sample,
													                      int as_sparse CV_DEFAULT(0) );

CvStatModel*
cvCreateStatModel( int flags, int header_size, CvStatModelRelease release,
                   CvStatModelPredict predict, CvStatModelUpdate update CV_DEFAULT(0));


#define CV_IMPLEMENT_QSORT_EX( func_name, T, LT, user_data_type )                   \
void func_name( T *array, size_t total, user_data_type aux )                        \
{                                                                                   \
    int isort_thresh = 7;                                                           \
    T t;                                                                            \
    int sp = 0;                                                                     \
                                                                                    \
    struct                                                                          \
    {                                                                               \
        T *lb;                                                                      \
        T *ub;                                                                      \
    }                                                                               \
    stack[48];                                                                      \
                                                                                    \
    aux = aux;                                                                      \
                                                                                    \
    if( total <= 1 )                                                                \
        return;                                                                     \
                                                                                    \
    stack[0].lb = array;                                                            \
    stack[0].ub = array + (total - 1);                                              \
                                                                                    \
    while( sp >= 0 )                                                                \
    {                                                                               \
        T* left = stack[sp].lb;                                                     \
        T* right = stack[sp--].ub;                                                  \
                                                                                    \
        for(;;)                                                                     \
        {                                                                           \
            int i, n = (int)(right - left) + 1, m;                                  \
            T* ptr;                                                                 \
            T* ptr2;                                                                \
                                                                                    \
            if( n <= isort_thresh )                                                 \
            {                                                                       \
            insert_sort:                                                            \
                for( ptr = left + 1; ptr <= right; ptr++ )                          \
                {                                                                   \
                    for( ptr2 = ptr; ptr2 > left && LT(ptr2[0],ptr2[-1]); ptr2--)   \
                        CV_SWAP( ptr2[0], ptr2[-1], t );                            \
                }                                                                   \
                break;                                                              \
            }                                                                       \
            else                                                                    \
            {                                                                       \
                T* left0;                                                           \
                T* left1;                                                           \
                T* right0;                                                          \
                T* right1;                                                          \
                T* pivot;                                                           \
                T* a;                                                               \
                T* b;                                                               \
                T* c;                                                               \
                int swap_cnt = 0;                                                   \
                                                                                    \
                left0 = left;                                                       \
                right0 = right;                                                     \
                pivot = left + (n/2);                                               \
                                                                                    \
                if( n > 40 )                                                        \
                {                                                                   \
                    int d = n / 8;                                                  \
                    a = left, b = left + d, c = left + 2*d;                         \
                    left = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))     \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = pivot - d, b = pivot, c = pivot + d;                        \
                    pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                                                                                    \
                    a = right - 2*d, b = right - d, c = right;                      \
                    right = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))    \
                                      : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));    \
                }                                                                   \
                                                                                    \
                a = left, b = pivot, c = right;                                     \
                pivot = LT(*a, *b) ? (LT(*b, *c) ? b : (LT(*a, *c) ? c : a))        \
                                   : (LT(*c, *b) ? b : (LT(*a, *c) ? a : c));       \
                if( pivot != left0 )                                                \
                {                                                                   \
                    CV_SWAP( *pivot, *left0, t );                                   \
                    pivot = left0;                                                  \
                }                                                                   \
                left = left1 = left0 + 1;                                           \
                right = right1 = right0;                                            \
                                                                                    \
                for(;;)                                                             \
                {                                                                   \
                    while( left <= right && !LT(*pivot, *left) )                    \
                    {                                                               \
                        if( !LT(*left, *pivot) )                                    \
                        {                                                           \
                            if( left > left1 )                                      \
                                CV_SWAP( *left1, *left, t );                        \
                            swap_cnt = 1;                                           \
                            left1++;                                                \
                        }                                                           \
                        left++;                                                     \
                    }                                                               \
                                                                                    \
                    while( left <= right && !LT(*right, *pivot) )                   \
                    {                                                               \
                        if( !LT(*pivot, *right) )                                   \
                        {                                                           \
                            if( right < right1 )                                    \
                                CV_SWAP( *right1, *right, t );                      \
                            swap_cnt = 1;                                           \
                            right1--;                                               \
                        }                                                           \
                        right--;                                                    \
                    }                                                               \
                                                                                    \
                    if( left > right )                                              \
                        break;                                                      \
                    CV_SWAP( *left, *right, t );                                    \
                    swap_cnt = 1;                                                   \
                    left++;                                                         \
                    right--;                                                        \
                }                                                                   \
                                                                                    \
                if( swap_cnt == 0 )                                                 \
                {                                                                   \
                    left = left0, right = right0;                                   \
                    goto insert_sort;                                               \
                }                                                                   \
                                                                                    \
                n = MIN( (int)(left1 - left0), (int)(left - left1) );               \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left0[i], left[i-n], t );                              \
                                                                                    \
                n = MIN( (int)(right0 - right1), (int)(right1 - right) );           \
                for( i = 0; i < n; i++ )                                            \
                    CV_SWAP( left[i], right0[i-n+1], t );                           \
                n = (int)(left - left1);                                            \
                m = (int)(right1 - right);                                          \
                if( n > 1 )                                                         \
                {                                                                   \
                    if( m > 1 )                                                     \
                    {                                                               \
                        if( n > m )                                                 \
                        {                                                           \
                            stack[++sp].lb = left0;                                 \
                            stack[sp].ub = left0 + n - 1;                           \
                            left = right0 - m + 1, right = right0;                  \
                        }                                                           \
                        else                                                        \
                        {                                                           \
                            stack[++sp].lb = right0 - m + 1;                        \
                            stack[sp].ub = right0;                                  \
                            left = left0, right = left0 + n - 1;                    \
                        }                                                           \
                    }                                                               \
                    else                                                            \
                        left = left0, right = left0 + n - 1;                        \
                }                                                                   \
                else if( m > 1 )                                                    \
                    left = right0 - m + 1, right = right0;                          \
                else                                                                \
                    break;                                                          \
            }                                                                       \
        }                                                                           \
    }                                                                               \
}

#define CV_IMPLEMENT_QSORT( func_name, T, cmp )  \
    CV_IMPLEMENT_QSORT_EX( func_name, T, cmp, int )

typedef enum _CvModelParams_
{

/* CART parameters */
    /* Number of classes. If value is 0 then it is regression tree. */
CV_MODEL_CLASS_NUM = 2,
    /* Split rule defined in train parameters structure and used for tree creation. */
CV_MODEL_FEATURE_NUM = 5,
    /* Types array of feature vector element. If value is not zero then value is categoric and returned value describe number of possible values of it. Size of array can be obtained by CV_CART_FEATURE_NUM parameter. */
CV_MODEL_FEATURE_TYPE = 6,
    /* Feature importance array. Size of array can be obtained by CV_MODEL_FEATURE_NUM parameter. */
CV_MODEL_FEATURE_IMPORTANCE = 7
}_CvModelParams_;


/* read/write. iteration step */
#define CV_BT_ITER_STEP         300
/* read only. number of iterations */
#define CV_BT_NUM_ITER          301
/* read/write. 1 by number_of_samples 32fC1 matrix of weights */
#define CV_BT_WEIGHTS           302

void icvReleaseBoostTrainStateMembers( CvBoostTrainState* ts );
void cvUpdateBtClassifier( CvBtClassifier* model,
		CvMat* train_data,
		int flags,
		CvMat* responses,
		CvStatModelParams* train_params,
		CvMat* comp_idx,
		CvMat* sample_idx,
		CvMat* type_mask,
		CvMat* missval_mask);

#define CV_TRAIN_STATMODEL_DEFRAGMENT_TRAIN_DATA    1
#define CV_TRAIN_STATMODEL_SAMPLES_AS_ROWS          2
#define CV_TRAIN_STATMODEL_SAMPLES_AS_COLUMNS       4
#define CV_TRAIN_STATMODEL_CATEGORICAL_RESPONSE     8
#define CV_TRAIN_STATMODEL_ORDERED_RESPONSE         16
#define CV_TRAIN_STATMODEL_RESPONSES_ON_OUTPUT      32
#define CV_TRAIN_STATMODEL_ALWAYS_COPY_TRAIN_DATA   64
#define CV_TRAIN_STATMODEL_SPARSE_AS_SPARSE         128

int icvCmpSparseVecElems( const void* a, const void* b );

#define CV_BT_VALUE 0
#define CV_BT_INDEX 1


CVAPI(float)
	cvEvalWeakClassifiers( const CvBtClassifier* bt, const CvMat* sample,
			CvMat* weak_vals, CvSlice slice CV_DEFAULT(CV_WHOLE_SEQ),
			int eval_type CV_DEFAULT(CV_BT_VALUE));


#define ICV_BT_PARAMS_NAME         "params"

#define ICV_BT_BOOST_TYPE_NAME     "boost_type"
#define ICV_BT_NUM_ITER_NAME       "num_iter"
#define ICV_BT_INFL_TRIM_NAME      "infl_trim_rate"
#define ICV_BT_NUM_SPLITS_NAME     "num_splits"

#define ICV_BT_CLASS_LABELS_NAME   "class_labels"
#define ICV_BT_TOTAL_FEATURES_NAME "total_features"

#define ICV_BT_TREES_NAME          "trees"

#define ICV_BT_COMP_IDX_NAME       "comp_idx"
#define ICV_BT_WEIGHTS_NAME        "weights"

#define ICV_BT_COMP_IDX_NAME       "comp_idx"
#define ICV_BT_THRESHOLD_NAME      "threshold"
#define ICV_BT_LEFT_NODE_NAME      "left_node"
#define ICV_BT_LEFT_VAL_NAME       "left_val"
#define ICV_BT_RIGHT_NODE_NAME     "right_node"
#define ICV_BT_RIGHT_VAL_NAME      "right_val"

#define CV_TYPE_NAME_ML_BOOSTING        "opencv-ml-boost-tree"

}
