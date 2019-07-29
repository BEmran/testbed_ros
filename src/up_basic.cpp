/*
 * File:   main.cpp
 * Author: Bara Emran
 * Created on September 9, 2017, 12:44 PM
 */
#include "../../testbed_up/include/testbed_up/up_basic.h" // basic header
/*****************************************************************************************
main: Run main function
 ****************************************************************************************/

int main(int argc, char** argv)
{
  // Welcome msg -------------------------------------------------------------------------
  printf("Start Program...\n");

  // Common initialization ---------------------------------------------------------------
  struct dataStruct* data = initialize(argc, argv);

  // Main loop ---------------------------------------------------------------------------
  main_loop(data);

  // Exit procedure ----------------------------------------------------------------------
  pthread_cancel(_Thread_Sensors);
  //pthread_cancel(_Thread_RosNode);
  pthread_cancel(_Thread_Control);
  printf("Close program\n");
  return 0;
}

void control(dataStruct* data, float dt){

}
