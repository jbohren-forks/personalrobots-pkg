#include "place_recognition/vocabulary_tree.h"

using namespace vision;

int main(int argc, char** argv)
{
  VocabularyTree tree;
  tree.load("test.tree");
  tree.save("test_copy.tree");

  return 0;
}
