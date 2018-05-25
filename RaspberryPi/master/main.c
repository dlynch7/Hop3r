 /******************************************************************************
* Kinematic library adapted from my MATLAB kinematic library:
* https://github.com/dlynch7/Hop3r/tree/master/MATLAB/Kinematic
* Begun 4-4-2018
******************************************************************************/

/*
* CAN interface adapted from cantest.c and candump.c, from www.skpang.co.uk
* Dan Lynch
* 3-27-2018
*
* Periodic threading code from http://2net.co.uk/tutorial/periodic_threads
* Added 3-28-2018
******************************************************************************/

/*
 *
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <errno.h>          /* Error number definitions */
#include <fcntl.h>          /* File control definitions */
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>          /* Standard input/optput definitions */
#include <stdlib.h>
#include <string.h>         /* String function definitions */
#include <sys/ioctl.h>
#include <sys/types.h>	    // needed for getpid()
#include <termios.h>        /* POSIX terminal control definitions */
#include <time.h>
#include <unistd.h>         /* UNIX standard function definitions */
#include <wiringPi.h>

#include "can_io.h"
#include "circ_buffer.h"
#include "kinematic.h"
#include "linux-can-utils/lib.h"
#include "per_threads.h"
#include "serial_interface.h"
#include "safety.h"

#define CONTROL_PERIOD_US 2000
#define CAN_READ_PERIOD_US 1
#define UART_PERIOD_US 2000

#define INBUFLENGTH (sizeof(inbuf)/sizeof(inbuf[0]))

#define PI 3.14159

void *Control_thread();
void *CAN_read_thread();
void *UART_thread();

uint8_t Control_thread_begin; // thread must wait for begin = 1
uint8_t CAN_read_thread_begin; // thread must wait for begin = 1
uint8_t UART_thread_begin; // thread must wait for begin = 1

uint8_t control_complete;

int serial_port;
char inbuf[100] = "";
char writemsg[10] = {};

can_input_struct dataFromCAN;

int refTraj[BUFLEN] = {};
float qaTraj[BUFLEN][3] = {};

int main(void) {
  pthread_t thread1, thread2, thread3;
  int rc1, rc2, rc3;
  int readTrajCount = 0;
  int writePermission = 0;
  int runPermission = 0;
  int startwait;

  CAN_read_thread_begin = 0; // reads from CAN bus cannot commence
  UART_thread_begin = 0; // reading and writing over UART cannot commence
  Control_thread_begin = 0; // control computations and writes to CAN cannot commence

  control_complete = 0;

  if(initSocketCAN()) {
    fprintf(stderr,"Failed to initialize SocketCAN interface.\n");
    return 1;
  }
  printf("Initialized SocketCAN interface.\n");

  safety_init();

  printf("Buffer read index: %d\n",get_read_index());
  printf("Buffer write index: %d\n",get_write_index());

  printf("This is the main function.\n");

  /////////////////////////////////////////////
  // POSIX serial interface:
  //
  // open the serial port: COMMENT IN AFTER HERE
  serial_port = open_port();
  printf("serial_port = %d\n",serial_port);

  config_port(serial_port);
  dprintf(serial_port,"%d\n",BUFLEN);

  // read(serial_port, inbuf,INBUFLENGTH);
  // sscanf(inbuf,"%d\n",&runPermission);
  // printf("runPermission = %d\n",runPermission);
  // if (runPermission != 1) {
  //   printf("Client denied permission to run.\n");
  //   return 1;
  // }

  // // receive current profile from client PC:
  // for (readTrajCount = 0; readTrajCount < BUFLEN; readTrajCount++) {
  //   read(serial_port, inbuf,INBUFLENGTH);
  //   sscanf(inbuf,"%f %f %f\n",&qaTraj[readTrajCount][0],&qaTraj[readTrajCount][1],&qaTraj[readTrajCount][2]);
  //   printf("%d: %5.3f\t%5.3f\t%5.3f\n",readTrajCount,qaTraj[readTrajCount][0],qaTraj[readTrajCount][1],qaTraj[readTrajCount][2]);
  //   // sscanf(inbuf,"%d\n",&refTraj[readTrajCount]);
  //   // printf("%d %d\n",readTrajCount,refTraj[readTrajCount]);
  // }
  //
  // printf("Done receiving\n");
  // // ask client PC for permission to write
  // read(serial_port, inbuf,INBUFLENGTH);
  // sscanf(inbuf,"%d\n",&writePermission);
  // printf("writePermission = %d\n",writePermission);
  // if (writePermission != 1) {
  //   printf("Client denied permission to write.\n");
  //   return 1;
  // } // COMMENT IN ABOVE HERE
  //////////////////////////////////////////////////////////////////////////////

  printf("Status of data_buf: read = %d, write = %d, empty = %d, full = %d\n",\
  get_read_index(),get_write_index(),buffer_empty(),buffer_full());

  /****************************************************************************
	*	Create three independent threads:
  *   Control_thread
  *   CAN_read_thread
  *   UART_thread
	****************************************************************************/
  if (setup_periodic()) {
    fprintf(stderr, "Failed to setup periodic threads.\n");
    return 1;
  }

  if ( (rc1=pthread_create(&thread1,NULL,&CAN_read_thread,NULL)) ) {
		fprintf(stderr,"Thread creation failed: %d\n", rc1);
	}
	if ( (rc2=pthread_create(&thread2,NULL,&UART_thread,NULL)) ) {
		fprintf(stderr,"Thread creation failed: %d\n", rc2);
	}
  if ( (rc3=pthread_create(&thread3,NULL,&Control_thread,NULL)) ) {
    fprintf(stderr,"Thread creation failed: %d\n", rc3);
  }

  printf("From main process ID: %d\n", ((int)getpid()));

  /****************************************************************************
  * Main loop
  ****************************************************************************/
  printf("Running...\n");
  run_program = 1;

  CAN_read_thread_begin = 1;
  Control_thread_begin = 1; // controls and writes to CAN bus can commence
  startwait = millis();
  while ((millis() - startwait) < 100); // delay to avoid emptying buffer early
  UART_thread_begin = 1; // reading and writing can commence

  /****************************************************************************
  *	Wait until threads are complete before main continues. Unless we
  *	wait, we run the risk of executing an exit which will terminate
  *	the process and all threads before the threads have completed.
  ****************************************************************************/
  pthread_join(thread1,NULL); // wait for Control_thread to complete
  pthread_join(thread2,NULL); // wait for UART_thread to complete
  pthread_join(thread3,NULL); // wait for CAN_read_thread to complete

  if (kill_motors()) {
    fprintf(stderr,"Unable to kill motors!\n");
  } else {
    printf("Killed motors.\n");
  }

  close(s); // close the CAN socket

  printf("Done writing to and reading from data_buf.\n");
  printf("Status of data_buf: read = %d, write = %d, empty = %d, full = %d\n",\
  get_read_index(),get_write_index(),buffer_empty(),buffer_full());

  printf("From main process ID: %d\n", ((int)getpid()));

  exit(EXIT_SUCCESS);
  return 0;
}

//*****************************************************************************
//
// CAN_read_thread:
//
// Reads from the CAN bus (blocking read), parses received CAN frames, and puts
// parsed data into a global struct (dataFromCAN), shared with Control_thread.
//
//
// This is a periodic thread with period defined by CAN_READ_PERIOD_US.
//
//*****************************************************************************
void *CAN_read_thread() {
  uint16_t read_count = 0;
  struct periodic_info info;

  while(!CAN_read_thread_begin) {;} // wait
  make_periodic(CAN_READ_PERIOD_US, &info); // period (first argument) in microseconds
  while ((run_program) && (!control_complete)) {
  // while ((!control_complete)) {
    readCAN(&dataFromCAN);

    wait_period(&info);
  }

  printf("CAN read thread has completed.\n");
  return NULL;
}

//*****************************************************************************
//
// Control_thread:
//
// Reads from a global struct (dataFromCAN), shared with CAN_read_thread.
// Calculates control inputs (commanded motor torques or motor positions) and
// writes them to the CAN bus. Also stores info from dataFromCAN and control
// data to a circular buffer shared with UART_thread.
//
// This is a periodic thread with period defined by CONTROL_PERIOD_US.
//
//*****************************************************************************

void *Control_thread() {
  uint16_t k = 0;
  double trqArr[3] = {0.0, 1.0, -2.0};
  double posArr[3] = {-150.0,-45.0,-135.0};

  struct periodic_info info;

  float qa[3] = {-1.6845,-2.6214,-1.4571}; // in degrees: -96.5, -150.2, -83.5
  // // -152.2, -170.2, -27.7 (deg) or -2.6564, -2.9706, -0.4835 (rad)
  float qu[6];
  float footPose[3] = {};
  double wrench[3] = {0,-70,0};
  double torques[3];

  /****************************************************************************
  * Wait for permission to begin,
  * then send/receive via CAN and put relevant data into circular buffer.
  ****************************************************************************/
  control_complete = 0;
  while(!Control_thread_begin) {;}
  make_periodic(CONTROL_PERIOD_US, &info); // period (first argument) in microseconds

  while ((run_program) && (k < BUFLEN)) {
  // while ((k < BUFLEN)) {
    // get shared data:
    pthread_mutex_lock(&mutex1);
    qa[0] = (double) 0.000555556*PI*(dataFromCAN.qa_act[0] - 2700);
    qa[1] = (double) 0.000555556*PI*(dataFromCAN.qa_act[1] - 2700);
    qa[2] = (double) 0.000555556*PI*(dataFromCAN.qa_act[2] - 2700);
    pthread_mutex_unlock(&mutex1);
    printf("%d\t%d\t%d\n",dataFromCAN.qa_act[0],dataFromCAN.qa_act[1],dataFromCAN.qa_act[2]);
    printf("qa = %f,\t%f,\t%f\n",qa[0],qa[1],qa[2]);

    clock_t tic2 = clock();
    geomFK(qa,qu,footPose,1);
    subchainIK(qaTraj[k],qu,footPose);
    if (wrench2torques(qa, qu, torques, wrench)) {
      fprintf(stderr,"wrench2torques failed.\n");
    }
    clock_t toc2 = clock();
    printf("Calculating took %f seconds\n", (double)(toc2 - tic2) / CLOCKS_PER_SEC);

    // write to the CAN bus:
    writePosToCAN(posArr); // this function handles the mutex

    // put stuff in the circular buffer:
    pthread_mutex_lock(&mutex1);
    buffer_write(qa[0],qa[1],qa[2]);
    pthread_mutex_unlock(&mutex1);
    ++k;
    wait_period(&info);
  }
  printf("Control thread has completed.\n");
  control_complete = 1;
  return NULL;
}

//*****************************************************************************
//
// UART_thread
//
// Description
//
//*****************************************************************************
void *UART_thread() {
  uint16_t j = 0;
  float bufferval[3];
  struct periodic_info info;

  /****************************************************************************
  * Wait for permission to begin,
  * then get relevant data from circular buffer and send via UART
  ****************************************************************************/

  while(!UART_thread_begin) {;}
  make_periodic(UART_PERIOD_US, &info); // period (1st argument) in microseconds
  while ((run_program) && (j < BUFLEN)) {
  // while ((j < BUFLEN)) {
    pthread_mutex_lock(&mutex1);
    buffer_read(bufferval);
    pthread_mutex_unlock(&mutex1);

    dprintf(serial_port,"%5.3f %5.3f %5.3f\n",bufferval[0],bufferval[1],bufferval[2]);
    printf("UART thread: %d: %5.3f %5.3f %5.3f\n",j,bufferval[0],bufferval[1],bufferval[2]);
    ++j;

    wait_period(&info);
  }
  printf("UART thread has completed.\n");
  return NULL;
}
