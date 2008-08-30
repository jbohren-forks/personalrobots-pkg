#ifndef __BINARY_TREE_H__
#define __BINARY_TREE_H__

#include <utility>
#include <string>

using namespace std;

/**
   @brief A simple, templated binary tree implementation
   @tparam T The tag type associated with the tree

   @attention Deletes all children trees on destruction
 */
template <class T>
class BinaryTree {
public:
  /**
     @param atag A tag to be associated with this tree
   */
  BinaryTree(const T& atag) :
    lTree(NULL),
    rTree(NULL),
    tag(atag)
  {
  };
  
  virtual ~BinaryTree() {
    if (lTree != NULL)
      delete lTree;
    if (rTree != NULL)
      delete rTree;
  }

  /**
     @param atag The tag to be set
   */
  void setTag(const T& atag) { tag = atag; }

  /**
     @return The tree's tag
   */
  T getTag() const { return tag; }

  /**
     @brief Finds a subtree with the given tag
     @param ref The searched-for tag
   */
  BinaryTree<T>* find(const T& ref) {
    if (this->tag == ref) 
      return this;
    else {
      if (lTree != NULL) return lTree->find(ref);
      if (rTree != NULL) return rTree->find(ref);
      return NULL;
    }
  }

  /**
     @brief Sets the 0-child
     @param tree The 0-child tree
   */
  void addChild0(BinaryTree<T>* tree) { lTree = tree; }
  /**
     @brief Sets the 1-child
     @param tree The 1-child tree
   */
  void addChild1(BinaryTree<T>* tree) { rTree = tree; }

  /**
     @return A pair of subtrees
   */
  pair<BinaryTree<T>*, BinaryTree<T>*> children() {
    pair<BinaryTree<T>*, BinaryTree<T>*> ret(lTree, rTree);
    return ret;
  }

  /**
     @return A pair of subtrees
   */
  pair<const BinaryTree<T>*, const BinaryTree<T>*> const_children() const {
    pair<const BinaryTree<T>*, const BinaryTree<T>*> ret(lTree, rTree);
    return ret;
  }

  /**
     @brief Recursively deletes the given tree and subtrees
     @param tree The base tree to be deleted
   */
  static void deleteTree(BinaryTree<T>* tree) {
    if (tree != NULL) 
      delete tree;
    else 
      return;

    pair<BinaryTree<T>*, BinaryTree<T>*> children = 
      tree->children();

    deleteTree(children.first);
    deleteTree(children.second);
  }

  /**
     @brief Applies the given function to the tags of this 
     tree and all subtrees
     @tparam IterFunction The type of function object to be applied
     @param func The function (object) to be applied
   */
  template <class IterFunction>
  void iter(IterFunction& func) {
    T cnode = getTag();

    func(cnode);
    
    pair<BinaryTree<T>*, BinaryTree<T>*> children = this->children();
    
    if (children.first != NULL)
      children.first->iter(func);

    if (children.second != NULL)
      children.second->iter(func);
  }

private:
  BinaryTree<T> *lTree;
  BinaryTree<T> *rTree;
  T tag;
};

#endif
