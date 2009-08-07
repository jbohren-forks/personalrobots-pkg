string joint_states_topic
string channel_name
string[] joint_names   # Names of the joints we're waiting to settle
float64[] tolerances   # Tolerance on each of the specified joints
duration padding       # Required time padding on either side of the chosen element
uint32 min_samples     # The minimum # of samples needed in the interval
uint32 cache_size      # The size of our cache when searching for an interval