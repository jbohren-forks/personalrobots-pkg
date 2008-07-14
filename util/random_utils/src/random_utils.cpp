#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <climits>
#include <rosthread/mutex.h>
#include <random_utils/random_utils.h>

/* mutex for random number generation locking */
static ros::thread::mutex rMutex;

void random_utils::init(void)
{
  rMutex.lock();
  srandom(time(NULL));
  rMutex.unlock();
}

void random_utils::init(rngState *state)
{
  FILE        *fp = fopen("/dev/urandom", "r");    
  unsigned int s;

  if(fp != NULL)
  {
    fread(&s, sizeof(unsigned int), 1, fp);
    fclose(fp);
  }
  else
    s = (unsigned int) time(NULL);
  state->seed = s;
  state->gaussian.valid = false;
}

double random_utils::uniform(double lower_bound, double upper_bound)
{
  rMutex.lock();
  double r = (double)random();
  rMutex.unlock();
  return (upper_bound - lower_bound) 
         * r / ((double)(RAND_MAX) + 1.0)
         + lower_bound;
}

double random_utils::uniform(rngState *state, double lower_bound, 
                             double upper_bound)
{
  return (upper_bound - lower_bound) 
         * (double)rand_r(&state->seed) / ((double)(RAND_MAX) + 1.0)
         + lower_bound;     
}

int random_utils::uniformInt(int lower_bound, int upper_bound)
{
  return (int)random_utils::uniform((double)lower_bound, 
                                    (double)(upper_bound + 1));
}

int random_utils::uniformInt(rngState *state, int lower_bound, int upper_bound)
{
  return (int)random_utils::uniform(state, (double)lower_bound, 
                                    (double)(upper_bound + 1));
}

double random_utils::gaussian(double mean, double stddev)
{
  double x1, x2, w;
  do
  {
    x1 = random_utils::uniform(-1, 1);
    x2 = random_utils::uniform(-1, 1);
    w = x1*x1 + x2*x2;
  } while (w >= 1.0 || w == 0.0);
  w = sqrt(-2.0 * log(w) / w);
  return x1 * w * stddev + mean;
}

double random_utils::gaussian(rngState *state, double mean, double stddev)
{
  if (state->gaussian.valid)
  {
    double r = state->gaussian.last * stddev + mean;
    state->gaussian.valid = false;
    return r;
  }
  else
  {	
    double x1, x2, w;
    do
    {
      x1 = random_utils::uniform(-1, 1);
      x2 = random_utils::uniform(-1, 1);
      w = x1*x1 + x2*x2;
    } while (w >= 1.0 || w == 0.0);
    w = sqrt(-2.0 * log(w) / w);
    state->gaussian.valid = true;
    state->gaussian.last  = x2 * w;
    return x1 * stddev * w + mean;
  }
}

double random_utils::bounded_gaussian(double mean, double stddev, 
                                      double max_stddev)
{
  double sample, max_s = max_stddev * stddev;
  do
  {
    sample = random_utils::gaussian(mean, stddev);
  } while (fabs(sample - mean) > max_s);
  return sample;
}

double random_utils::bounded_gaussian(rngState *state, double mean, 
                                      double stddev, double max_stddev)
{
  double sample, max_s = max_stddev * stddev;
  do
  {
    sample = random_utils::gaussian(state, mean, stddev);
  } while (fabs(sample - mean) > max_s);
  return sample;
}

