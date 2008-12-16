#if !defined(TRAINBASEH)
#define TRAINBASEH

#include <string>
#include "calonder_descriptor/rtree_classifier.h"
#include "calonder_descriptor/rng.h"
#include "calonder_descriptor/randomized_tree.h"


class TrainBase
{
public:

   void view(std::string img_url, int num_pts); //Just view an image with star points on it

   float train(std::string img_url,
               int num_trees,
               int depth,
               int views,
               int base_sz, 
               float phi_minmax, 
               float theta_minmax, 
               float lambda_plusminus);            
private:
   std::string findNonexistingFilename(std::string base, std::string ext);
};

#endif
