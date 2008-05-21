#include <cstring>
#include <cstdio>
#include <stdint.h>
#include "ros/time.h"

int main(int argc, char **argv)
{
  if (argc <= 1)
  {
    printf("\nusage: verify_bag_integity BAGFILE\n\n");
    return 1;
  }
  FILE *f = fopen(argv[1], "r");
  if (!f)
  {
    printf("couldn't open [%s]\n", argv[1]);
    return 2;
  }
  char topic_name_buf[4096];
  fgets(topic_name_buf, 4096, f);
  topic_name_buf[strlen(topic_name_buf)-1] = 0; // get rid of the newline
  printf("topic: %s\n", topic_name_buf);
  int num_messages = 0;
  uint32_t secs(0), nsecs(0), message_len;
  ros::Time start;
  uint32_t largest(0), smallest(1234567890);
  while (!feof(f))
  {
    fread(&secs, 4, 1, f);
    fread(&nsecs, 4, 1, f);
    if (num_messages == 0)
      start = ros::Time(secs, nsecs);
    fread(&message_len, 4, 1, f);
    if (feof(f))
      break;
    if (message_len > largest) largest = message_len;
    if (message_len < smallest) smallest = message_len;
    if (fseek(f, message_len, SEEK_CUR))
      break;
    num_messages++;
  }
  fclose(f);
  ros::Duration elapsed = start - ros::Time(secs, nsecs);
  printf("message count: %d\ntime span: %f seconds\n", 
         num_messages, elapsed.to_double());
  printf("largest message: %d bytes\nsmallest message: %d bytes\n",
         largest, smallest);

  return 0;
}
