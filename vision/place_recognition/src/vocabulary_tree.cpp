#include "place_recognition/vocabulary_tree.h"
#include "place_recognition/kmeans.h"
#include <boost/foreach.hpp>
#include <boost/accumulators/statistics/p_square_quantile.hpp>
#include <limits>
#ifdef USE_BYTE_SIGNATURES
#include <cstdlib> // posix_memalign
#include "calonder_descriptor/randomized_tree.h" // quantizeVector
#include "calonder_descriptor/basic_math.h"
#endif

namespace vision {

// TODO: currently assume objs contains indices in [0, nObjs)
// TODO: choose between calculating database vectors or clearing objects
void VocabularyTree::build( const FeatureMatrix& features,
                            const std::vector<unsigned int>& objs,
                            unsigned int k, unsigned int levels,
                            bool keep_training_images)
{
  dim_ = features.cols();
  k_ = k;
  levels_ = levels;

#ifdef USE_BYTE_SIGNATURES
  // step 0: determine bounds to use for rescaling signatures
  // estimate 5% and 95% quantiles
  using namespace boost::accumulators;
  typedef accumulator_set<float, boost::accumulators::features<tag::p_square_quantile> > accumulator_t;
  accumulator_t acc05(quantile_probability = 0.05);
  accumulator_t acc95(quantile_probability = 0.95);
  for (const float *i = features.data(),
         *ie = features.data() + features.rows()*features.cols();
       i != ie; ++i) {
    acc05(*i);
    acc95(*i);
  }
  bounds_[0] = p_square_quantile(acc05);
  bounds_[1] = p_square_quantile(acc95);
  printf("Using bounds [%f, %f]\n", bounds_[0], bounds_[1]);
#endif
  
  // step 1: tree construction
  printf("Building tree...\n");
  std::vector<unsigned int> input(features.rows());
  for (unsigned int i = 0; i < input.size(); ++i) input[i] = i;
  constructVocabulary(features, input);
  
  // step 2: add objects
  printf("Adding objects...\n");
  for (int i = 0; i < features.rows(); ++i)
    addFeature(root_, objs[i], features.row(i));

  // step 3: assign weight
  printf("Assigning weights...\n");
  db_vectors_.resize( *std::max_element(objs.begin(), objs.end()) + 1 );
  assignWeights();
  
  // step 4: calculate database vectors
  if (keep_training_images) {
    printf("Calculating database vectors...\n");
    calculateDatabaseVectors();
  }
  else
    clearDatabase();
}

void VocabularyTree::clearDatabaseAux(Node* node)
{
  node->inverted_file.clear();
  BOOST_FOREACH( Node* child, node->children )
    clearDatabaseAux(child);
}

void VocabularyTree::clearDatabase()
{
  db_vectors_.clear();
  clearDatabaseAux(root_);
}

#ifdef USE_BYTE_SIGNATURES
unsigned int VocabularyTree::insert(const uint8_t* image_features,
                                    unsigned int num_features)
{
  unsigned int id = db_vectors_.size();
  db_vectors_.resize(id + 1);
  for (unsigned int i = 0; i < num_features; ++i)
    addFeatureToDatabaseVector(root_, id, image_features + i*dim_);
  normalizeL1(db_vectors_[id]);
  
  return id;
}
#else
unsigned int VocabularyTree::insert(const FeatureMatrix& image_features)
{
  unsigned int id = db_vectors_.size();
  db_vectors_.resize(id + 1);
  for (int i = 0; i < image_features.rows(); ++i)
    addFeatureToDatabaseVector(root_, id, image_features.row(i));
    
  normalizeL1(db_vectors_[id]);
  
  return id;
}
#endif

// TODO: check if already an existing vocabulary?
void VocabularyTree::constructVocabulary(const FeatureMatrix& features,
                                         const std::vector<unsigned int>& input)
{
  root_ = pool_.construct();
  constructVocabularyAux(features, input, root_, 0);
}

// TODO: input can be partitioned (sorted?) in place, passed as iterator range
void VocabularyTree::constructVocabularyAux(const FeatureMatrix& features,
                                            const std::vector<unsigned int>& input,
                                            Node* node, unsigned int level)
{
  if (level >= levels_ || input.size() == 0)
    return; // leaf node

  ++level;

  // TODO: these allocations could be done once
  std::vector<int> membership(input.size());
  FeatureMatrix centroids((int)k_, (int)dim_);

  // Compute child centers using kmeans
  if (level <= 2) {
    printf("\tRunning kmeans, level %u, %u features... ", level, input.size());
    fflush(stdout);
  }
  int num_clusters = kmeans(features, input, membership, centroids, k_);
  if (level <= 2)
    printf("done\n");

  // Partition features among children
  std::vector< std::vector<unsigned int> > children_input(num_clusters);
  for(unsigned int i = 0; i < input.size(); ++i)
    children_input[ membership[i] ].push_back(input[i]);

  // Recursively cluster children
  node->children.resize(num_clusters);
  for(int i = 0; i < num_clusters; ++i)
  {
    node->children[i] = pool_.construct();
#ifdef USE_BYTE_SIGNATURES
    uint8_t** child_centroid = &node->children[i]->centroid;
    posix_memalign(reinterpret_cast<void**>(child_centroid), 16, dim_);
    features::RandomizedTree::quantizeVector(const_cast<float*>(centroids.data() + i*dim_),
                                             dim_, QUANTIZE_N, bounds_, *child_centroid);
#else
    node->children[i]->centroid.set( centroids.row(i) );
#endif
    constructVocabularyAux(features, children_input[i], node->children[i], level);
  }
}

#ifdef USE_BYTE_SIGNATURES
VocabularyTree::Node*
VocabularyTree::findNearestChild(Node* node, const uint8_t* feature)
{
  Node* nearest = NULL;
  int d_min = std::numeric_limits<int>::max();
  for (std::vector<Node*>::iterator i = node->children.begin(),
         ie = node->children.end(); i != ie; ++i) {
    int distance = features::L2Distance(dim_, feature, (*i)->centroid);
    if (distance < d_min) {
      nearest = *i;
      d_min = distance;
    }
  }
  return nearest;
}
#else
VocabularyTree::Node*
VocabularyTree::findNearestChild(Node* node, const FeatureMatrix::RowXpr& feature)
{
  Node* nearest = NULL;
  float d_min = std::numeric_limits<float>::max();
  for (std::vector<Node*>::iterator i = node->children.begin(),
         ie = node->children.end(); i != ie; ++i) {
    float distance = distanceL2(feature.transpose(), (*i)->centroid);
    if (distance < d_min) {
      nearest = *i;
      d_min = distance;
    }
  }
  return nearest;
}
#endif

// TODO: next 3 functions basically the same, would be nice to abstract
//       this a bit
#ifdef USE_BYTE_SIGNATURES
void VocabularyTree::addFeature(Node* node, unsigned int object_id,
                                const FeatureMatrix::RowXpr& feature)
{
  // Quantize feature
  // TODO: don't allocate new uchar array every time
  float* f = &const_cast<FeatureMatrix::RowXpr&>(feature)(0);
  uint8_t* qf = NULL;
  posix_memalign(reinterpret_cast<void**>(&qf), 16, dim_);
  features::RandomizedTree::quantizeVector(f, dim_, QUANTIZE_N, bounds_, qf);
  addFeature(node, object_id, qf);
}

void VocabularyTree::addFeature(Node* node, unsigned int object_id,
                                const uint8_t* feature)
{
  if ( node->children.empty() ) {
    // leaf node: increment entry for obj in inverted file
    node->inverted_file[object_id]++;
  }
  else {
    Node* nearest = findNearestChild(node, feature);
    addFeature(nearest, object_id, feature);
  }
}
#else
void VocabularyTree::addFeature(Node* node, unsigned int object_id,
                                const FeatureMatrix::RowXpr& feature)
{
  if ( node->children.empty() ) {
    // leaf node: increment entry for obj in inverted file
    node->inverted_file[object_id]++;
  }
  else {
    Node* nearest = findNearestChild(node, feature);
    addFeature(nearest, object_id, feature);
  }
}
#endif

#ifdef USE_BYTE_SIGNATURES
void VocabularyTree::addFeatureToQuery(Node* node, ImageVector& vec,
                                       const uint8_t* feature)
{
  if (node->weight > 0.0f)
    vec[node] += node->weight;
  if ( !node->children.empty() ) {
    Node* nearest = findNearestChild(node, feature);
    addFeatureToQuery(nearest, vec, feature);
  }
}

void VocabularyTree::addFeatureToDatabaseVector(Node* node, unsigned int object_id,
                                                const uint8_t* feature)
{
  if (node->weight > 0.0f)
    db_vectors_[object_id][node] += node->weight;
  if ( node->children.empty() ) {
    node->inverted_file[object_id]++;
  }
  else {
    Node* nearest = findNearestChild(node, feature);
    addFeatureToDatabaseVector(nearest, object_id, feature);
  }
}
#else
void VocabularyTree::addFeatureToQuery(Node* node, ImageVector& vec,
                                       const FeatureMatrix::RowXpr& feature)
{
  if (node->weight > 0.0f)
    vec[node] += node->weight;
  if ( !node->children.empty() ) {
    Node* nearest = findNearestChild(node, feature);
    addFeatureToQuery(nearest, vec, feature);
  }
}

void VocabularyTree::addFeatureToDatabaseVector(Node* node, unsigned int object_id,
                                                const FeatureMatrix::RowXpr& feature)
{
  if (node->weight > 0.0f)
    db_vectors_[object_id][node] += node->weight;
  if ( node->children.empty() ) {
    node->inverted_file[object_id]++;
  }
  else {
    Node* nearest = findNearestChild(node, feature);
    addFeatureToDatabaseVector(nearest, object_id, feature);
  }
}
#endif

// TODO: could perhaps be optimized more, restructured
// TODO: block longer lists?
void VocabularyTree::recursiveTfIdfWeighting(Node* node, IdOutputIterator output)
{
  if ( node->children.empty() )
  {
    node->weight = tfIdfWeight(node->inverted_file.size());

    // pass object ids to parent
    for (InvertedFile::iterator i = node->inverted_file.begin(),
           ie = node->inverted_file.end(); i != ie; ++i)
      *output = i->first;
  }
  else
  {
    // recursively build id set
    std::vector<unsigned int> id_set, child_id_set, union_id_set;
    for (std::vector<Node*>::iterator i = node->children.begin(),
           ie = node->children.end(); i != ie; ++i) {
      recursiveTfIdfWeighting(*i, std::back_inserter(child_id_set));
      std::set_union(id_set.begin(), id_set.end(),
                     child_id_set.begin(), child_id_set.end(),
                     std::back_inserter(union_id_set));
      id_set.swap(union_id_set);
      child_id_set.resize(0);
      union_id_set.resize(0);
    }
    
    node->weight = tfIdfWeight(id_set.size());
    
    // pass object ids to parent
    if (node != root_)
      std::copy(id_set.begin(), id_set.end(), output);
  }
}

void VocabularyTree::assignWeights()
{
  std::vector<unsigned int> dummy;
  recursiveTfIdfWeighting(root_, std::back_inserter(dummy));
}

void VocabularyTree::updateFiles(Node* node, InvertedFile& file, InvertedFile& parent_file)
{
  if (node->weight == 0.0f) return;
  for (InvertedFile::iterator i = file.begin(), ie = file.end(); i != ie; ++i) {
    unsigned int id = i->first;
    unsigned int frequency = i->second;
    parent_file[id] += frequency;
    db_vectors_[id][node] += node->weight * frequency;
  }
}

// TODO: somewhat wasteful of memory
void VocabularyTree::calculateDatabaseVectorsAux(Node* node,
                                                 InvertedFile& parent_file)
{
  if ( node->children.empty() )
  {
    updateFiles(node, node->inverted_file, parent_file);
  }
  else
  {
    InvertedFile virtual_file;
    for (std::vector<Node*>::iterator i = node->children.begin(),
           ie = node->children.end(); i != ie; ++i) {
      calculateDatabaseVectorsAux(*i, virtual_file);
    }
    updateFiles(node, virtual_file, parent_file);
  }
}

void VocabularyTree::calculateDatabaseVectors()
{
  InvertedFile dummy;
  calculateDatabaseVectorsAux(root_, dummy);

  BOOST_FOREACH( ImageVector& vec, db_vectors_ ) {
    normalizeL1(vec);
  }
}

// TODO: fix indentation, ugh
// TODO: change to uint8_t centroids, don't bother saying centroid size
void VocabularyTree::saveAux(Node* node, FILE* out, std::string indentation)
{
  if (node == root_)
    fprintf(out, "%sCentroid[0]: \n", indentation.c_str());
  else {
    fprintf(out, "%sCentroid[%d]: ", indentation.c_str(), dim_);
    for (int i = 0; i < dim_; ++i)
      fprintf(out, "%f ", node->centroid[i]);
    fprintf(out, "\n");
  }

  fprintf(out, "%sWeight: %f\n", indentation.c_str(), node->weight);
  
  fprintf(out, "%sInverted file[%u]: ", indentation.c_str(), node->inverted_file.size());
  for (InvertedFile::iterator i = node->inverted_file.begin(),
         ie = node->inverted_file.end(); i != ie; ++i)
    fprintf(out, "(%u,%u) ", i->first, i->second);
  fprintf(out, "\n");
  
  fprintf(out, "%sChildren: %u\n", indentation.c_str(), node->children.size());
  //indentation += "    ";
  BOOST_FOREACH( Node* child, node->children )
    saveAux(child, out, indentation);
}

void VocabularyTree::save(const std::string& file)
{
  FILE* out = fopen(file.c_str(), "w");
  fprintf(out, "Branching factor: %u\n", k_);
  fprintf(out, "Levels: %u\n", levels_);
  fprintf(out, "Dimension: %u\n", dim_);
  fprintf(out, "Database images: %u\n", db_vectors_.size());
  saveAux(root_, out);
}

void VocabularyTree::loadAux(Node* node, FILE* in, unsigned int indent_level)
{
  int centroid_size = 0;
  //fseek(in, indentation, SEEK_CUR);
  fscanf(in, "Centroid[%d]: ", &centroid_size);
  if (centroid_size != 0) {
#ifdef USE_BYTE_SIGNATURES
    posix_memalign((void**)&node->centroid, 16, centroid_size);
#else
    node->centroid.resize(centroid_size);
#endif
  }
  for (int i = 0; i < centroid_size; ++i) {
    fscanf(in, "%f ", &node->centroid[i]);
  }
  fscanf(in, "\n");

  //fseek(in, indentation, SEEK_CUR);
  fscanf(in, "Weight: %f\n", &node->weight);
  
  unsigned int inverted_file_size;
  //fseek(in, indentation, SEEK_CUR);
  fscanf(in, "Inverted file[%u]: ", &inverted_file_size);
  for (unsigned int i = 0; i < inverted_file_size; ++i) {
    unsigned int id, frequency;
    fscanf(in, "(%u, %u) ", &id, &frequency);
    node->inverted_file.insert(node->inverted_file.end(),
                               InvertedFile::value_type(id, frequency));
  }
  fscanf(in, "\n");

  unsigned int num_children;
  //fseek(in, indentation, SEEK_CUR);
  fscanf(in, "Children: %u\n", &num_children);
  node->children.reserve(num_children);
  for (unsigned int i = 0; i < num_children; ++i) {
    Node* child = pool_.construct();
    node->children.push_back(child);
    loadAux(child, in, indent_level + 1);
  }
}

void VocabularyTree::load(const std::string& file)
{
  unsigned int db_size;
  FILE* in = fopen(file.c_str(), "r");
  fscanf(in, "Branching factor: %u\n", &k_);
  fscanf(in, "Levels: %u\n", &levels_);
  fscanf(in, "Dimension: %u\n", &dim_);
  fscanf(in, "Database images: %u\n", &db_size);
  root_ = pool_.construct();
  loadAux(root_, in);
  if (db_size > 0) {
    db_vectors_.resize(db_size);
    calculateDatabaseVectors();
  }
}

} // namespace vision
