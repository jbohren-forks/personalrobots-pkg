#include <place_recognition/vocabulary_tree.h>
#include <cstdio>
#include <fstream>

using namespace vision;

static const char sig_file[] = "signatures.dat";
static const char obj_file[] = "objects.dat";
//static const char vocab_file[] = "holidays.tree";
static const char vocab_file[] = "wg_indoor1.tree";
static const unsigned int DIMENSION = 176;

int main(int argc, char** argv)
{
  std::ifstream sig_is(sig_file, std::ios::binary);
  std::ifstream obj_is(obj_file, std::ios::binary);

  // Get number of descriptors
  sig_is.seekg(0, std::ios::end);
  int length = sig_is.tellg();
  int num_sigs = length / (DIMENSION * sizeof(float));
  sig_is.seekg(0, std::ios::beg);
  /*
  int num_sigs = 100000;
  int length = num_sigs * DIMENSION * sizeof(float);
  */
  printf("%d descriptors\n", num_sigs);

  // Read in descriptors
  FeatureMatrix features(num_sigs, (int)DIMENSION);
  sig_is.read((char*)features.data(), length);
  printf("Done reading in descriptors\n");

  // Read in objs vector
  std::vector<unsigned int> objs(num_sigs);
  obj_is.read((char*)&objs[0], num_sigs * sizeof(unsigned int));

  // Create tree
  VocabularyTree tree;
  tree.build(features, objs, 10, 5, false);
  //tree.build(features, objs, 5, 4, false);
  tree.save(vocab_file);
  
  return 0;
}
