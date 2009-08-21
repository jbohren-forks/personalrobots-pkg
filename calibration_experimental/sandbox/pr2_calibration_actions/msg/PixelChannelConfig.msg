string pixel_topic    # Topic on which to listen to ImagePointStamped
string image_topic    # Topic on which to listen to the source images
string channel_name   # User defined name that gets added to the result
float64 tolerance     # Tolerance on the pixel location
duration padding      # Required time padding on either side of the chosen element
uint32 min_samples    # The minimum # of samples needed in the interval
uint32 cache_size     # The size of our cache when searching for an interval