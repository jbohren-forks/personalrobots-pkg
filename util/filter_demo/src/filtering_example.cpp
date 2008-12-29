#include <math.h>
#include <string>
#include <vector>

#include "ros/node.h"
#include "filters/transfer_function.h"
#include "filter_coefficient_server/Filter.h"



class ExampleFilter : public ros::node
{
public:
  ExampleFilter() : ros::node("filtering_example")
  {
  }
  bool call_srv(std::string name, std::vector<std::string> args, std::vector<double> &a, std::vector<double> &b, std::vector<std::vector<double> > &in, std::vector<std::vector<double> > &out)
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
      
      std::vector<double> temp;
      temp.resize(2);    
      //create a filter using the coeffs
      filters::TransferFunctionFilter<double> filter(b,a,1);

      //pass in a simple sinewave
      for(uint32_t i=0; i<100; i++)
      {       
        temp[0]=sin(2*M_PI*i/10);
        temp[1]=cos(2*M_PI*i/20);
        in.push_back(temp);
        filter.update(&temp);
        out.push_back(temp);
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
  ExampleFilter a;
  std::vector<double> tf_a;
  std::vector<double> tf_b;
  std::vector<std::vector<double> > input;
  std::vector<std::vector<double> > output;
  std::vector<std::string> args;
  for(int i=2; i<argc; i++)
  {
    args.push_back(argv[i]);
  }

  if (a.call_srv(argv[1], args, tf_a, tf_b, input, output))
  {
    for(uint32_t i=0; i<tf_a.size();i++)
    {
      printf("a[%d]:%f b[%d]:%f \n",i,tf_a[i],i,tf_b[i]);
    }
  }
  else
    printf("The filter name does not exist or the wrong arguments were sent\n");
  ros::fini();
  return 0;
}

