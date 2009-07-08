#include "lifelong_mapping/database.h"

#include <std_msgs/Float32.h>
#include <cstdio>

int main(int argc, char** argv)
{
  std_msgs::Float32 msg;
  msg.data = 42.0;

  lifelong_mapping::Database db("database");
  db.insert(0, msg);

  std_msgs::Float32 msg2;
  db.get(0, msg2);

  printf("Data = %f\n", msg2.data);

  return 0;
}
