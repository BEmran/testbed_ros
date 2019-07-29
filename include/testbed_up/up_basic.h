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
#include <math.h>

#include "../../testbed_up/include/lib/Encoder.h"         // Encoder class
#include "../../testbed_up/include/lib/libusdusb4.h"      // USB4 lib
#include "../../testbed_up/include/lib/TimeSampling.h"    // samplig time
#include "../../testbed_up/include/testbed_up/ros_node.h" // ros node

/*****************************************************************************************
Global variables
******************************************************************************************/
#define _ROSNODE_FREQ 200           // ROS thread frequency in Hz
#define _SENSORS_FREQ 500           // Sensor thread frequency in Hz
#define _CONTROL_FREQ 200           // Control thread frequency in Hz
#define RSign +1.0
#define PSign +1.0
#define YSign -1.0
#define PI M_PI

using namespace std;
bool _CloseRequested = false;
//pthread_t _Thread_Rosnode;
pthread_t _Thread_Sensors;
pthread_t _Thread_Control;
/*****************************************************************************************
Define structures
******************************************************************************************/
struct dataStruct {
  bool is_sensors_ready;
  bool is_control_ready;
  bool is_mainfun_ready;
  float du[4];                // output PWM signal
  float encoderes[3];
  float record[10];           // stored data to print each samplig time
  int yaw_fold;
  RosNode *rosnode;
  FILE *file;
  int argc;
  char** argv;
};
/*****************************************************************************************
Functions prototype
******************************************************************************************/
dataStruct* initialize(int &argc, char **argv);
void main_loop(dataStruct* data);
void *rosNodeThread(void *data);
void *sensorsThread(void *data);
void *controlThread(void *data);
void ctrlCHandler(int signal);
float unfoldYaw (float yaw_new, float yaw_old, int* n);
void printRecord(FILE* file, float data[]);
void control(dataStruct* data, float dt);
/**************************************************************************************************
 initialize: Common initialize function for all programes
 **************************************************************************************************/
dataStruct* initialize(int &argc, char **argv){

  // conncet ctrl+c Handler
  signal(SIGINT, ctrlCHandler);

  // Define main variables --------------------------------------------------
  dataStruct *data;
  data->is_control_ready = false;
  data->is_sensors_ready = false;
  data->is_mainfun_ready = false;
  data->yaw_fold = 0;
  // initialize record data with zeros
  for (int i = 0; i < 10; ++i) {
    data->record[i] = 0.0;
  }

  // Start threads ----------------------------------------------------------
  //pthread_create(&_Thread_RosNode, NULL, rosNodeThread, (void *) data);
  pthread_create(&_Thread_Control, NULL, controlThread, (void *) data);
  pthread_create(&_Thread_Sensors, NULL, sensorsThread, (void *) data);

  // Create new record file -------------------------------------------------
  char file_name[64];
  int fileNumber = 0;
  // find avaliable file name & number
  do {
    sprintf(file_name, "~/testbed_data_%.2d.csv", fileNumber++);
  } while (access(file_name, F_OK) == 0);
  // open avaliable file
  data->file = fopen(file_name, "w");
  // check file
  if (data->file == NULL) {
    printf("Error creating file!\n");
    exit(1);
  }
  // Display Information for user
  printf("A file successfully created to record testbed data\n");
  printf("Record file path and name: %s\n",file_name);

  // Record starting time of test -------------------------------------------
  time_t rawtime;
  time (&rawtime);
  struct tm * timeinfo = localtime (&rawtime);
  fprintf(data->file,"Current local time and date: %s", asctime(timeinfo));

  // Print data header
  fprintf(data->file,"time,"
                     "enc0,enc1,enc2,"
                     "ur,up,uw,uz,"
                     "d0,d1\n");

  // Initialize ROS ----------------------------------------------------------------------
  while(!data->is_sensors_ready);                 // wait for sensor thread to be ready
  string name = "testbed_up";                     // define ros node name
  ros::init(data->argc, data->argv, name);        // initialize ros
  ros::NodeHandle nh;                             // define ros handle
  data->rosnode = new RosNode (nh, name);         // define RosNode object

  // Wait for user to be ready ----------------------------------------------
  while(!data->is_control_ready || !data->is_sensors_ready);
  int x = 0;
  while (x == 0) {
    printf("Enter 1 to run control\n");
    cin >> x;
    sleep(1);
  }
  data->is_mainfun_ready = true;

  return data;
}
/**************************************************************************************************
 main_loop: runnin main loop for all programes
 **************************************************************************************************/
void main_loop(dataStruct* data){
  ros::Rate loop_rate(_ROSNODE_FREQ);             // define ros frequency
  while (ros::ok() && !_CloseRequested)
  {
    // Prepare encoderes msg -------------------------------------------------------------
    data->rosnode->publishAllMsgs(data->encoderes, data->du);
    ros::spinOnce();
    loop_rate.sleep();
  }


}
/**************************************************************************************************
 ctrlCHandler: Detect ctrl+c to quit program
 **************************************************************************************************/
void ctrlCHandler(int signal) {
  _CloseRequested = true;
  printf("Ctrl+c have been detected\n");
}
/**************************************************************************************************
 sensorsThread: read encoderes
 **************************************************************************************************/
void *sensorsThread(void *data) {
  // Initialize thread ------------------------------------------------------------------
  printf("Sensors thread: Starts\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize USB4 sensor ----------------------------------
  printf("Initialize USB4 Sensor\n");
  short deviceCount = 0;
  int iResult = USB4_Initialize(&deviceCount); // Initialize the USB4 driver
  if (iResult != USB4_SUCCESS) {
    ROS_INFO("USB4 can't be initialized, USB4 error #%d\n", iResult);
    exit(EXIT_FAILURE);
  } else
    ROS_INFO("USB4 is initialized\n");

  //-------------------------------------- Configure encoders -----------------------------------
  Encoder En0 (0, 0, 10000, QUAD_X4);
  Encoder En1 (0, 1, 10000, QUAD_X4);
  Encoder En2 (0, 2, 10000, QUAD_X4);

  // Announce thread is ready
  my_data->is_sensors_ready = true;
  printf("Sensors thread: Ready\n");

  // Main loop ----------------------------------------------------------------------------------
  TimeSampling ts(_SENSORS_FREQ);
  float dt, dtsumm = 0;
  while (!_CloseRequested) {
    // calculate sampling time
    dt = ts.updateTs();

    // Read Sensor -----------------------------------------
    my_data->encoderes[0] = En0.getAngleRad() * RSign;
    my_data->encoderes[1] = En1.getAngleRad() * PSign;
    my_data->encoderes[2]  = unfoldYaw(En2.getAngleRad() * YSign, my_data->encoderes[2],
        &my_data->yaw_fold);

    // Update record values
    my_data->record[1] = my_data->encoderes[0];
    my_data->record[2] = my_data->encoderes[1];
    my_data->record[3] = my_data->encoderes[2];

    // Record data in a file
    printRecord(my_data->file, my_data->record);

    // Display info for user every 5 second
    dtsumm+= dt;
    if (dtsumm > 5) {
      dtsumm = 0;
      printf("Sensors thread: Running with %4d Hz\n", int(1 / dt));
    }
  }

  //---------------------------------------- Exit procedure ----------------------------------------
  printf("Sensors thread: Exit\n");
  pthread_exit(NULL);
}
/*****************************************************************************************
 rosNodeThread: ROS Node thread
 *****************************************************************************************/
void *rosNodeThread(void *data) {
  // Initialize thread ------------------------------------------------------------------
  printf("ROSNode thread: Starts\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize ROS ----------------------------------------------------------------------
  while(!my_data->is_sensors_ready);              // wait for sensor thread to be ready
  string name = "testbed_up";                     // define ros node name
  ros::init(my_data->argc, my_data->argv, name);  // initialize ros
  ros::NodeHandle nh;                             // define ros handle
  my_data->rosnode = new RosNode (nh, name);      // define RosNode object
  ros::Rate loop_rate(_ROSNODE_FREQ);             // define ros frequency

  // Announce thread is ready
  printf("ROSNode thread: Ready\n");

  // Main loop ----------------------------------------------------------------------------------
  while (ros::ok() && !_CloseRequested)
  {
    // Prepare encoderes msg -------------------------------------------------------------
    my_data->rosnode->publishAllMsgs(my_data->encoderes, my_data->du);
    ros::spinOnce();
    loop_rate.sleep();
  }

  //---------------------------------------- Exit procedure ---------------------------------------
  ctrlCHandler(0);
  printf("ROSNode thread: Exit\n");
  pthread_exit(NULL);
}
/**************************************************************************************************
 controlThread: Perform control loop and send PWM output
**************************************************************************************************/
void *controlThread(void *data) {
  // Initialize thread ------------------------------------------------------------------
  printf("Control thread: Starts\n");

  // Initialize mapping data
  struct dataStruct *my_data;
  my_data = (struct dataStruct *) data;

  // Initialize sampling time
  TimeSampling ts(_CONTROL_FREQ);

  // Initialize PWM
  my_data->du[0] = 0.0;
  my_data->du[1] = 0.0;
  my_data->du[2] = 0.0;
  my_data->du[3] = 0.0;

  // Announce thread is ready
  my_data->is_control_ready = true;
  printf("Control thread: Ready\n");

  // Wait for main function to be ready
  while(!my_data->is_mainfun_ready);

  // Main loop ----------------------------------------------------------------------------------
  float dt, dtsumm = 0;
  while (!_CloseRequested) {

    // calculate sampling time
    dt = ts.updateTs();

    // Run control function when sampling is not big
    if (dt < 0.02) {
      control(my_data,dt);
    } else {
      printf("Control thread: Sampling time is too big = %f\n",dt);
      //TODO: stop control thread for huge sampling time
    }

    // Send data to motor
    //du2motor(my_data->pwm, my_data->du, my_data->pwm_offset, my_data->du_min, my_data->du_max);

    // Display info for user every 5 second
    dtsumm += dt;
    if (dtsumm > 5.0) {
      dtsumm = 0;
      printf("Control thread: Running with %4d Hz\n", int(1 / dt));
    }
  }

  // Exit procedure -----------------------------------------------------------------------------
  ctrlCHandler(0);
  printf("Control thread: Exit\n");
  pthread_exit(NULL);
}
/*****************************************************************************************
 unfoldYaw: Unfold yaw angle
 *****************************************************************************************/
float unfoldYaw (float encoder_new, float yaw_old, int* n){
  float encoder_old = yaw_old - *n * 2 * PI; // get old encoder data before folding
  if ((encoder_new-encoder_old)>6.0)         // if encoder data jumped from - to +
    *n = *n - 1;
  if ((encoder_new-encoder_old)<-6.0)	       // if encoder data jumbed from + to -
    *n = *n + 1;
  return encoder_new + *n * 2 * PI;	         // fold encoder data
}
/**************************************************************************************************
printRecord: print recorded data in a file
**************************************************************************************************/
void printRecord(FILE* file, float data[]){
  int size = 10;          // record data 0-9
  // Timing data
  struct timeval tv;

  // Record data header
  char buf[1024];
  char *pos = buf;

  // Calculate delta time
  gettimeofday(&tv, NULL);
  long tmp = 1000000L * tv.tv_sec + tv.tv_usec;
  data[0] = tmp / 1000000.0;

  // get data stored in data array
  for (int i = 0; i < size; ++i) {
    pos += sprintf(pos, "%+9.3f, ", data[i]);
  }

  // print collected data
  fprintf(file, "%s\n", buf);
}
#endif // UP_BASIC

