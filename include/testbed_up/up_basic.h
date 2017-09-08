/*
 * File:   main.hpp
 * Author: Bara Emran
 * Created on September 9, 2017, 12:44 PM
 */

#ifndef UP_BASIC
#define UP_BASIC

/*****************************************************************************************
Header files
******************************************************************************************/
#include <signal.h>       // signal ctrl+c
#include <stdio.h>        // printf
#include <cstdlib>
#include <unistd.h>      // usleep
#include <pthread.h>   // create thread

#include "lib/Encoder.h"         // Encoder class
#include "lib/libusdusb4.h"    // USB4 lib
#include "lib/SamplingTime.h"    // samplig time

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>   // encoder msg

/*****************************************************************************************
Global variables
******************************************************************************************/
#define _SENSOR_FREQ 500        // Sensor thread frequency in Hz
#define _ROS_FREQ 250                // ROS thread frequency in Hz
#define RSign +1.0
#define PSign +1.0
#define YSign -1.0
#define PI 3.14159

using namespace std;
bool _CloseRequested = false;
pthread_t _Thread_Sensors;
pthread_t _Thread_RosNode;

/*****************************************************************************************
Define structures
******************************************************************************************/
struct dataStruct {
        float encoderes[3];
        int yaw_fold;
        int argc;
        char** argv;
};

/*****************************************************************************************
Functions prototype
******************************************************************************************/
void *sensorsThread(void *data);
void *rosNodeThread(void *data);
void ctrlCHandler(int signal);
float unfoldYaw (float yaw_new, float yaw_old, int *n);

#endif // UP_BASIC

