#include "Python.h"

#include "place_recognition/vocabulary_tree.h"
#include "place_recognition/sparse_stereo.h"
#include <iterator>
#include <numeric>
#include <sstream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <star_detector/detector.h>
#include <calonder_descriptor/rtree_classifier.h>
#include <calonder_descriptor/matcher.h>
#include <Cv3DPoseEstimateStereo.h>
#include <cvwimage.h>
#include <highgui.h>

using boost::format;
using namespace features;
using namespace vision;

#ifdef USE_BYTE_SIGNATURES
typedef uint8_t sig_data_t;
#else
typedef float sig_data_t;
#endif
typedef Promote<sig_data_t>::type distance_t;

static int im2arr(CvArr **dst, PyObject *src)
{
  int width, height;
  if (!PyObject_HasAttrString(src, "size") ||
      !PyObject_HasAttrString(src, "mode") ||
      !PyObject_HasAttrString(src, "tostring"))
    return 0;

  if (!PyArg_ParseTuple(PyObject_GetAttrString(src, "size"), "ii", &width, &height))
    return 0;

  char *mode = PyString_AsString(PyObject_GetAttrString(src, "mode"));
  int depth, nchannels, bps;
  nchannels = (int)strlen(mode);
  if (strcmp(mode, "F") == 0) {
    depth = IPL_DEPTH_32F;
    bps = 4;
  } else {
    depth = IPL_DEPTH_8U;
    bps = 1;
  }

  char *string;
  Py_ssize_t length;
  PyString_AsStringAndSize(PyObject_CallObject(PyObject_GetAttrString(src, "tostring"), NULL), &string, &length);

  *dst = cvCreateImageHeader(cvSize(width, height), depth, nchannels);
  cvSetData(*dst, (void*)string, nchannels * bps * width);

  return 1;
}

static const char classifier_file[] = "/u/prdata/calonder_trees/current.rtc";

typedef struct {
  PyObject_HEAD
  VocabularyTree *vt;
  RTreeClassifier *classifier;
} vocabularytree_t;

typedef struct {
  PyObject_HEAD
  sig_data_t *data;
  int size;
} signature_t;

static FeatureMatrix extract_float_features(PyObject *self, PyObject *pim,
                                            PyObject *descriptors)
{
  RTreeClassifier *classifier = ((vocabularytree_t*)self)->classifier;
  unsigned int dimension = classifier->classes();

  FeatureMatrix image_features;

  if (descriptors == NULL) {
    // Prepare keypoint detector, classifier
    // TODO: use FAST instead?
    StarDetector detector(cvSize(640, 480), 5, 10.0);
    std::vector<Keypoint> pts;

    // Compute features and their descriptors for each object (image)

    CvArr *cva;
    if (!im2arr(&cva, pim)) assert(0);
    CvArr *local = cvCreateImage(cvGetSize(cva), IPL_DEPTH_8U, 1);
    cvCopy(cva, local);
    cv::WImageBuffer1_b left( (IplImage*)local );

    // Find keypoints
    detector.DetectPoints(left.Ipl(), std::back_inserter(pts));
    printf("[Star detector gave %d points, dimension %d]\n", pts.size(), dimension);

    // Compute descriptors
    image_features.resize(pts.size(), dimension);
    float* sig = image_features.data();
    
    BOOST_FOREACH( const Keypoint& pt, pts ) {
      // Signature
      cv::WImageView1_b view = extractPatch(left.Ipl(), pt);
      classifier->getSignature(view.Ipl(), sig);

      sig += dimension;
    }
  } else {
    if (sizeof(float) != sizeof(sig_data_t)) {
      printf("ERROR: We have problems in extract_float_features!\n");
      abort();
    }
    
    unsigned int size = PySequence_Size(descriptors);
    image_features.resize(size, dimension);
    float *dst = image_features.data();
    PyObject *iterator = PyObject_GetIter(descriptors);
    PyObject *d;
    assert(iterator != NULL);
    while ((d = PyIter_Next(iterator)) != NULL) {
      signature_t *sig = (signature_t*)d;
      memcpy((char*)dst, (char*)sig->data, dimension * sizeof(float));
      dst += dimension;
    }
  }

  return image_features;
}


#ifdef USE_BYTE_SIGNATURES
static sig_data_t* extract_features(PyObject *self, PyObject *pim,
                                    unsigned int *num_features, PyObject *descriptors)
{
  RTreeClassifier *classifier = ((vocabularytree_t*)self)->classifier;
  unsigned int dimension = classifier->classes();

  sig_data_t *image_features;

  if (descriptors == NULL) {
    // Prepare keypoint detector, classifier
    // TODO: use FAST instead?
    StarDetector detector(cvSize(640, 480), 5, 10.0);
    std::vector<Keypoint> pts;

    // Compute features and their descriptors for each object (image)

    CvArr *cva;
    if (!im2arr(&cva, pim)) assert(0);
    CvArr *local = cvCreateImage(cvGetSize(cva), IPL_DEPTH_8U, 1);
    cvCopy(cva, local);
    cv::WImageBuffer1_b left( (IplImage*)local );

    // Find keypoints
    detector.DetectPoints(left.Ipl(), std::back_inserter(pts));
    printf("[Star detector gave %d points, dimension %d]\n", pts.size(), dimension);

    // Compute descriptors
    *num_features = pts.size();
    posix_memalign((void**)&image_features, 16, pts.size()*dimension*sizeof(sig_data_t));
    sig_data_t* sig = image_features;

    BOOST_FOREACH( const Keypoint& pt, pts ) {
      // Signature
      cv::WImageView1_b view = extractPatch(left.Ipl(), pt);
      classifier->getSignature(view.Ipl(), sig);

      sig += dimension;
    }
  } else {
    unsigned int size = PySequence_Size(descriptors);
    *num_features = size;
    posix_memalign((void**)&image_features, 16, size*dimension*sizeof(sig_data_t));
    sig_data_t *dst = image_features;
    PyObject *iterator = PyObject_GetIter(descriptors);
    PyObject *d;
    assert(iterator != NULL);
    while ((d = PyIter_Next(iterator)) != NULL) {
      signature_t *sig = (signature_t*)d;
      memcpy((char*)dst, (char*)sig->data, dimension * sizeof(sig_data_t));
      dst += dimension;
    }
  }

  return image_features;
}
#else
static inline FeatureMatrix extract_features(PyObject *self, PyObject *pim, PyObject *descriptors)
{
  return extract_float_features(self, pim, descriptors);
}
#endif

// NOTE: Uses float signatures, but self-contained. Should be able to get away with this
//       even if calonder_descriptor bindings use byte signatures.
PyObject *vtbuild(PyObject *self, PyObject *args)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;

  PyObject *training_images;
  unsigned int k = 5;
  unsigned int levels = 4;
  int keep_training_images = 1;

  if (!PyArg_ParseTuple(args, "O|iii", &training_images, &k, &levels, &keep_training_images))
    return NULL;

  srand(0);

  PyObject *iterator = PyObject_GetIter(training_images);
  PyObject *pil_im;
  if (iterator == NULL)
      return NULL;
  std::vector<FeatureMatrix> fl;  // feature list
  std::vector<unsigned int> objs;
  int obj = 0;
  int rows = 0;
  while ((pil_im = PyIter_Next(iterator)) != NULL) {
    FeatureMatrix f = extract_float_features(self, pil_im, NULL);
    rows += f.rows();
    for (int i = 0; i < f.rows(); i++)
      objs.push_back(obj);
    fl.push_back(f);
    obj++;
  }

  int dimension = fl[0].cols();

  // Concatenate all feature matices vertically
  FeatureMatrix features((int)rows, (int)dimension);
  int current_row = 0;
  for (unsigned int i = 0; i < fl.size(); ++i) {
    features.block(current_row, 0, fl[i].rows(), dimension) = fl[i];
    current_row += fl[i].rows();
  }

  //
  // Train vocabulary tree
  vt->build(features, objs, k, levels, keep_training_images);
  Py_RETURN_NONE;
}

PyObject *vtadd(PyObject *self, PyObject *args)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;
  PyObject *pil, *descriptors = NULL;
  if (!PyArg_ParseTuple(args, "O|O", &pil, &descriptors))
    return NULL;
#ifdef USE_BYTE_SIGNATURES
  unsigned int num_features = 0;
  sig_data_t *features = extract_features(self, pil, &num_features, descriptors);
  vt->insert(features, num_features);
  free(features);
#else
  vt->insert(extract_features(self, pil, descriptors));
#endif
  Py_RETURN_NONE;
}

PyObject *vtsave(PyObject *self, PyObject *args)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename))
    return NULL;
  vt->save(filename);
  Py_RETURN_NONE;
}

PyObject *vttopN(PyObject *self, PyObject *args)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;

  unsigned int N_show = 10;
  PyObject *query_image;
  PyObject *descriptors;
  if (!PyArg_ParseTuple(args, "OOi", &query_image, &descriptors, &N_show))
    return NULL;

  std::vector<VocabularyTree::Match> matches;
  matches.reserve(N_show);
  
#ifdef USE_BYTE_SIGNATURES
  unsigned int num_features = 0;
  sig_data_t *features = extract_features(self, query_image, &num_features, descriptors);
  vt->find(features, num_features, N_show, std::back_inserter(matches));
  free(features);
#else
  FeatureMatrix features = extract_features(self, query_image, descriptors);
  vt->find(features, N_show, std::back_inserter(matches));
#endif

  unsigned int N = ((vocabularytree_t*)self)->vt->databaseSize();
  PyObject *l = PyList_New(N);
  for (unsigned j = 0; j < N; ++j)
    PyList_SetItem(l, j, PyFloat_FromDouble(0.0));

  for (unsigned int j = 0; j < matches.size(); ++j) {
    unsigned int match_id = matches[j].id;
    assert(match_id < N);
    PyList_SetItem(l, match_id, PyFloat_FromDouble(matches[j].score));
  }
  

  return l;
}

/* Method table */
static PyMethodDef vocabularytree_methods[] = {
  {"save", vtsave, METH_VARARGS},
  {"build", vtbuild, METH_VARARGS},
  {"add", vtadd, METH_VARARGS},
  //{"query", vtquery, METH_VARARGS},
  {"topN", vttopN, METH_VARARGS},
  {NULL, NULL},
};

static PyObject *
vocabularytree_GetAttr(PyObject *self, char *attrname)
{
    return Py_FindMethod(vocabularytree_methods, self, attrname);
}

static void
vocabularytree_dealloc(PyObject *self)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;
  delete vt;
  PyObject_Del(self);
}

static PyTypeObject vocabularytree_Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,
    "VocabularyTree",
    sizeof(vocabularytree_t),
    0,
    (destructor)vocabularytree_dealloc,
    0,
    (getattrfunc)vocabularytree_GetAttr,
    0,
    0,
    0, // repr
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,

    0,

    Py_TPFLAGS_CHECKTYPES,

    0,
    0,
    0,
    0

    /* the rest are NULLs */
};

PyObject *mkvocabularytree(PyObject *self, PyObject *args)
{
  vocabularytree_t *object = PyObject_NEW(vocabularytree_t, &vocabularytree_Type);
  object->vt = new VocabularyTree();
  object->classifier = new RTreeClassifier(true);
  object->classifier->read(classifier_file);

  return (PyObject*)object;
}


PyObject *mkload(PyObject *self, PyObject *args)
{
  char *filename;
  if (!PyArg_ParseTuple(args, "s", &filename))
    return NULL;

  vocabularytree_t *object = PyObject_NEW(vocabularytree_t, &vocabularytree_Type);
  object->vt = new VocabularyTree();
  object->classifier = new RTreeClassifier(true);
  object->classifier->read(classifier_file);
  object->vt->load(filename);

  return (PyObject*)object;
}

static PyMethodDef methods[] = {
  {"vocabularytree", mkvocabularytree, METH_VARARGS},
  {"load", mkload, METH_VARARGS},
  {NULL, NULL},
};

extern "C" void initplace_recognition()
{
    PyObject *m, *d;

    m = Py_InitModule("place_recognition", methods);
    d = PyModule_GetDict(m);
}


// Don't need this anymore?
/*
PyObject *vtquery(PyObject *self, PyObject *args)
{
  VocabularyTree *vt = ((vocabularytree_t*)self)->vt;

  PyObject *query_images;
  if (!PyArg_ParseTuple(args, "O", &query_images))
    return NULL;

  // Prepare keypoint detector, classifier
  // TODO: change to FAST? do some of this elsewhere?
  StarDetector detector(cvSize(640, 480), 7, 10.0);
  std::vector<Keypoint> pts;
  RTreeClassifier classifier(true);
  classifier.read(classifier_file);
  unsigned int dimension = classifier.classes();

  // Compute features and their descriptors for each object (image)
  std::vector<float*> buffers;
  std::vector<size_t> buffer_sizes;
  std::vector<unsigned int> objs;
  // Save features for each image
  size_t num_objs = PySequence_Size(query_images);
  std::vector< std::vector<float*> > image_features(num_objs);
  int obj = 0;
  PyObject *iterator = PyObject_GetIter(query_images);
  PyObject *pil_im;
  if (iterator == NULL)
      return NULL;
  while ((pil_im = PyIter_Next(iterator)) != NULL) {
    CvArr *cva;
    if (!im2arr(&cva, pil_im)) return NULL;
    CvArr *local = cvCreateImage(cvGetSize(cva), IPL_DEPTH_8U, 1);
    cvCopy(cva, local);
    // Load left/right images
    cv::WImageBuffer1_b left( (IplImage*)local );

    // Find keypoints in left image
    pts.resize(0);
    detector.DetectPoints(left.Ipl(), std::back_inserter(pts));

    // Compute descriptors, disparities
    float* sig_buffer = Eigen::ei_aligned_malloc<float>(dimension * pts.size());
    buffers.push_back(sig_buffer);
    buffer_sizes.push_back(pts.size());
    float* sig = sig_buffer;
    BOOST_FOREACH( const Keypoint& pt, pts ) {
      // Signature
      cv::WImageView1_b view = extractPatch(left.Ipl(), pt);
      classifier.getSignature(view.Ipl(), sig);
      objs.push_back(obj);

      // Disparity
      image_features[obj].push_back(sig);

      sig += dimension;
    }
    ++obj;
  }
  size_t num_features = std::accumulate(buffer_sizes.begin(), buffer_sizes.end(), 0);  
  //printf("%u features\n", num_features);

  // Copy into single Eigen matrix
  FeatureMatrix features((int)num_features, (int)dimension);
  size_t current_row = 0;
  for (unsigned int i = 0; i < buffers.size(); ++i) {
    features.block(current_row, 0, buffer_sizes[i], dimension) =
      Eigen::Map<FeatureMatrix>(buffers[i], buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    free(buffers[i]);
  }
  buffers.clear();

  // 
  // Validation
  int training_correct = 0;
  unsigned int N = ((vocabularytree_t*)self)->size;
  unsigned int N_show = 10;
  std::vector<VocabularyTree::Match> matches;
  matches.reserve(N);
  
  PyObject *r = PyList_New(num_objs);
  for (unsigned i = 0; i < num_objs; ++i) {
    PyObject *l = PyList_New(num_objs);
    PyList_SetItem(r, i, l);
    for (unsigned j = 0; j < num_objs; ++j)
      PyList_SetItem(l, j, PyFloat_FromDouble(0.0));
  }

  // Validate on training images
  current_row = 0;
  for (unsigned int i = 0; i < num_objs; ++i) {
    FeatureMatrix query = features.block(current_row, 0, buffer_sizes[i], dimension);
    current_row += buffer_sizes[i];
    printf("Training image %u, %u features\n", i, query.rows());

    // Find top N matches
    matches.resize(0);
    vt->find(query, N, std::back_inserter(matches));
    if (matches[0].id == i) ++training_correct;

    // Set up matcher for signatures in query image
    BruteForceMatcher<float, int> matcher(dimension);
    BOOST_FOREACH( float* sig, image_features[i] )
      matcher.addSignature(sig, 0);

    for (unsigned int j = 0; j < N_show; ++j) {
      unsigned int match_id = matches[j].id;
      PyList_SetItem(PyList_GetItem(r, i), match_id, PyFloat_FromDouble(matches[j].score));
    }
  }
  return r;
}
*/
