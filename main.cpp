/*
 * main.cpp
 *
 *  Created on: Nov 3, 2017
 *      Author: wangbo
 */


#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "utility.h"
#include "scheduler.h"


void loop( void );
void fast_loop( void );
void slow_loop( void );
void super_slow_loop( void );

// main loop scheduler
static AP_Scheduler scheduler;

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called
  (in 10ms units) and the maximum time they are expected to take (in
  microseconds)
 */
static const AP_Scheduler::Task scheduler_tasks[] = {
//    { update_GPS,            2,     900 },
//    { update_navigation,     10,    500 },
//    { medium_loop,           2,     700 },
//    { update_altitude,      10,    1000 },
//    { fifty_hz_loop,         2,     950 },
//    { run_nav_updates,      10,     800 },
//    { slow_loop,            10,     500 },
//    { gcs_check_input,       2,     700 },
//    { gcs_send_heartbeat,  100,     700 },
//    { gcs_data_stream_send,  2,    1500 },
//    { gcs_send_deferred,     2,    1200 },
//    { compass_accumulate,    2,     700 },
//    { barometer_accumulate,  2,     900 },
//    { super_slow_loop,     100,    1100 },
//    { perf_update,        1000,     500 }
      { slow_loop,            100,     500 },
      { super_slow_loop,     1000,    1100 }
};

#define LINUX_OS

#ifdef  LINUX_OS
//#define MAINTASK_TICK_TIME_MS 20
#define MAINTASK_TICK_TIME_MS 10//这个设置为10ms主要是为了跟sim_aircraft的仿真频率一致，其实20ms（50hz就够）
//#define MAINTASK_TICK_TIME_MS 1000
int seconds=0;
int mseconds=MAINTASK_TICK_TIME_MS*(1e3);/*每个tick为20毫秒，也就是20000微秒*/
struct timeval maintask_tick;
#endif

uint32_t maintast_cnt;
uint32_t loop_cnt;

int main()
{

    printf("hello wangbo\n");

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
    //fast_loop_finish_timer = gettimeofday_us();
    printf(" sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]) = %d\n",sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));

    while (1)
    {
#ifdef LINUX_OS
        //20170919为了跟sim_multicopter的动力模型计算频率一致，我们现在的循环频率是100hz
        maintask_tick.tv_sec = seconds;
        maintask_tick.tv_usec = mseconds;
        select(0, NULL, NULL, NULL, &maintask_tick);
#endif

        maintast_cnt++;

        if(maintast_cnt > 100)
        {
            //printf("hello wangbo while loop\n");
            maintast_cnt = 0;
        }


        /*
         * 如果这个while(1)的循环周期是10ms那么
         * 这个loop循环中所有的函数都执行一边（或者说运行最多函数时），所需要的时间应该是小于10ms的
         */
        //copter.loop();
        loop();
    }

    return 0;
}
//uint32_t fast_loop_finish_timer;
void loop()
{
    loop_cnt++;

    //uint32_t timer = micros();
    uint32_t timer = gettimeofday_us();

    // Execute the fast loop
    //        // ---------------------
    fast_loop();

    // tell the scheduler one tick has passed
    scheduler.tick();

    //fast_loop_finish_timer = gettimeofday_us();


    //uint16_t dt = timer - fast_loop_finish_timer;
    //uint16_t dt = fast_loop_finish_timer - timer;
    //uint32_t loop_us = 10000;//10ms
    uint32_t loop_us = 9000;//10ms
    //uint32_t time_available= timer + loop_us - gettimeofday_us();
    uint32_t time_available = loop_us - ( gettimeofday_us() - timer );

    if(loop_cnt > 100)
    {
        //printf("time_available = %d \n",time_available);
        loop_cnt = 0;
    }


    scheduler.run(time_available > loop_us ? 0u : time_available);

}

void fast_loop( void )
{
//    for(int i=0;i<10000;i++)
//    {
//
//    }
}


void slow_loop( void )
{
    printf("hello slow_loop\n");
}

void super_slow_loop( void )
{
    printf("hello super_slow_loop\n");
}

