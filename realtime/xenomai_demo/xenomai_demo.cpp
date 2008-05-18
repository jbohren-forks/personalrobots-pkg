#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>

RT_TASK demo_task;

#include "ros/node.h"
#include "std_msgs/MsgString.h"

/* NOTE: error handling omitted. */

float period = 1; //period in milliseconds 

float elapsed = 0.0;
long elapsed_ticks = 0;
double max_elapsed = 0.0;
double min_elapsed = period * 2;
double total_elapsed = 0.0;
long int num_overruns = 0;
long int num_ticks = 0;

void demo(void *arg)
{
	RTIME now, previous;

	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period)
	 */
	rt_task_set_periodic(NULL, TM_NOW, period * 1000000);
	previous = rt_timer_read();

	while (1) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();
		elapsed_ticks = (long)(now - previous);
		previous = now;

		elapsed = elapsed_ticks / 1000000.0;
		if(elapsed > max_elapsed)
			max_elapsed = elapsed;
		if(elapsed < min_elapsed)
			min_elapsed = elapsed;
		if(elapsed_ticks > 1.1 * period * 1000000)
			num_overruns++;
		num_ticks++;
		total_elapsed += elapsed;
		}
}

void cleanup(int sig)
{
	printf("Goodbye.\n");
	rt_task_delete(&demo_task);
	//exit(0);
}

class XenomaiTest : public ros::node
{
public:
 MsgString msg;
 
 XenomaiTest() : ros::node("XenomaiTest")
 {
   advertise<MsgString>("status");
 }

 void update(char *s)
 {
  msg.data = string(s);
  publish("status", msg);
 }
};
  
int main(int argc, char* argv[])
{
	char msg[1024];
	//signal(SIGTERM, cleanup);
	//signal(SIGINT, cleanup);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_timer_set_mode(TM_ONESHOT);
	//rt_timer_set_mode(900000);
	
	/*
	 * Arguments: &task,
	 *            name,
	 *            stack size (0=default),
	 *            priority,
	 *            mode (FPU, start suspended, ...)
	 */
	rt_task_create(&demo_task, "trivial", 0, 99, XNFPU);

	/*
	 * Arguments: &task,
	 *            task function,
	 *            function argument
	 */
	rt_task_start(&demo_task, &demo, NULL);

	ros::init(argc, argv);
	XenomaiTest x;
	while(x.ok()){
	  usleep(100000);
	  snprintf(msg, sizeof(msg), "(milliseconds) min offset: %f \t max offset: %f \t average offset:%f \t overruns: %ld/%ld\n", min_elapsed - period, max_elapsed - period, total_elapsed / num_ticks - period, num_overruns, num_ticks);
	  printf(msg);
	  x.update(msg);
	}
	cleanup(0);	
}
