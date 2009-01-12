#include "place_recognition/vocabulary_tree.h"
#include <cstdio>

using namespace vision;

int main(int argc, char** argv)
{
  if (argc < 3) {
    printf("Usage: %s test.tree test_empty.tree\n", argv[0]);
    return 0;
  }
  
  VocabularyTree tree;
  tree.load(argv[1]);
  tree.clearDatabase();
  tree.save(argv[2]);

  return 0;
}
