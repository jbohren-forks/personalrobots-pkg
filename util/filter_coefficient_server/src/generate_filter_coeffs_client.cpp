#include "ros/node.h"
#include "filters/transfer_function.h"
#include <string>
#include <vector>
#include "filter_coefficient_server/Filter.h"

class GenFilter : public ros::node
{
public:
  GenFilter() : ros::node("generate_filter_coeffs_client")
  {
  }
  bool call_add(std::string name, std::vector<std::string> args, std::vector<double> &a, std::vector<double> &b)
  {
    filter_coefficient_server::Filter::request  req;
    filter_coefficient_server::Filter::response res;
    req.name = name;
    req.args = args;
    if (ros::service::call("generate_filter_coeffs", req, res))
    {
      for(uint32_t i=0; i<res.a.size();i++)
      { 
        a.push_back(res.a[i]);
        b.push_back(res.b[i]);
      }
      return true;
    }
    else
    {
      a.push_back(0);
      b.push_back(0);
      return false;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  if (argc < 3)
  {
    printf("usage: generate_filter_coeffs_client name args\n");
    printf("usage: generate_filter_coeffs_client butter 2 .1\n");
    printf("usage: generate_filter_coeffs_client butter 2 .1 high\n");
    printf("usage: generate_filter_coeffs_client butter 2 .1 .5 stop\n");
    return 1;
  }
  GenFilter a;
  std::vector<double> tf_a;
  std::vector<double> tf_b;
  std::vector<std::string> args;
  for(int i=2; i<argc; i++)
  {
    args.push_back(argv[i]);
  }
  
  if (a.call_add(argv[1], args, tf_a, tf_b))
  {
    for(uint32_t i=0; i<tf_a.size();i++)
    {
      printf("a[%d]:%f b[%d]:%f \n",i,tf_a[i],i,tf_b[i]);
    }
  }
  else
    printf("an error occurred\n");
  ros::fini();
  return 0;
}

