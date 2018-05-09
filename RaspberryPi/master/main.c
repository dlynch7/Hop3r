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
// #include <poll.h>
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
#include <wiringSerial.h>

#include "linux-can-utils/lib.h"
#include "circ_buffer.h"
#include "kinematic.h"
#include "per_threads.h"
#include "serial_interface.h"

#define CAN_PERIOD_US 2000
#define UART_PERIOD_US 2000

#define INBUFLENGTH (sizeof(inbuf)/sizeof(inbuf[0]))

#define PI 3.14159

void *CAN_thread();
void *UART_thread();
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
uint8_t CAN_thread_begin; // read and write threads must wait for begin = 1
uint8_t UART_thread_begin; // read and write threads must wait for begin = 1
int serial_port;
char inbuf[100] = "";
char writemsg[10] = {};
int16_t amp = 0;
int amptemp = 0;

int refTraj[BUFLEN] = {};
float qaTraj[BUFLEN][3] = {};

int main(void) {
  int rc1, rc2;
  int readTrajCount = 0;
  int writePermission = 0;
  int runPermission = 0;
  int startwait;

  CAN_thread_begin = 0; // reading and writing cannot commence
  UART_thread_begin = 0; // reading and writing cannot commence

  printf("Buffer read index: %d\n",get_read_index());
  printf("Buffer write index: %d\n",get_write_index());

  printf("This is the main function.\n");

  /////////////////////////////////////////////
  // POSIX serial interface:
  //
  // open the serial port:
  serial_port = open_port();
  printf("serial_port = %d\n",serial_port);
  //
  config_port(serial_port);
  //
  dprintf(serial_port,"%d\n",BUFLEN);
  //
  read(serial_port, inbuf,INBUFLENGTH);
  sscanf(inbuf,"%d\n",&runPermission);
  printf("runPermission = %d\n",runPermission);
  if (runPermission != 1) {
    printf("Client denied permission to run.\n");
    return 1;
  }
  //
  // receive current profile from client PC:
  for (readTrajCount = 0; readTrajCount < BUFLEN; readTrajCount++) {
    read(serial_port, inbuf,INBUFLENGTH);
    sscanf(inbuf,"%f %f %f\n",&qaTraj[readTrajCount][0],&qaTraj[readTrajCount][1],&qaTraj[readTrajCount][2]);
    printf("%d: %5.3f\t%5.3f\t%5.3f\n",readTrajCount,qaTraj[readTrajCount][0],qaTraj[readTrajCount][1],qaTraj[readTrajCount][2]);
    // sscanf(inbuf,"%d\n",&refTraj[readTrajCount]);
    // printf("%d %d\n",readTrajCount,refTraj[readTrajCount]);
  }
  //
  printf("Done receiving\n");
  // usleep(250000);
  // tcflush(serial_port, TCIOFLUSH);
  // dprintf(serial_port,"%d\n",BUFLEN);
  // ask client PC for permission to write
  read(serial_port, inbuf,INBUFLENGTH);
  sscanf(inbuf,"%d\n",&writePermission);
  printf("writePermission = %d\n",writePermission);
  if (writePermission != 1) {
    printf("Client denied permission to write.\n");
    return 1;
  }

  // usleep(250000);
  // tcflush(serial_port, TCIOFLUSH);

  // int j = 0;
  // for (j = 0; j < BUFLEN;j++) {
  //   dprintf(serial_port,"%d\n",j);
  // }

  // usleep(250000);
  // tcflush(serial_port, TCIOFLUSH);
  //
  //////////////////////////////////////////////////////////////////////////////

  printf("Status of data_buf: read = %d, write = %d, empty = %d, full = %d\n",\
  get_read_index(),get_write_index(),buffer_empty(),buffer_full());

  /****************************************************************************
	*	Create two independent threads.
  * CAN_thread will write to a buffer.
  * UART_thread will read from the buffer.
	****************************************************************************/
  if (setup_periodic()) {
    fprintf(stderr, "Failed to setup periodic threads.\n");
    return 1;
  }

  pthread_t thread1, thread2;
  if ( (rc1=pthread_create(&thread1,NULL,&CAN_thread,NULL)) ) {
		printf("Thread creation failed: %d\n", rc1);
	}
	if ( (rc2=pthread_create(&thread2,NULL,&UART_thread,NULL)) ) {
		printf("Thread creation failed: %d\n", rc2);
	}

  printf("From main process ID: %d\n", ((int)getpid()));
  CAN_thread_begin = 1; // reading and writing can commence
  startwait = millis();
  while ((millis() - startwait) < 100); // delay
  UART_thread_begin = 1; // reading and writing can commence

  /****************************************************************************
  * Main loop
  ****************************************************************************/
  printf("Running...");

  /****************************************************************************
  *	Wait until threads are complete before main continues. Unless we
  *	wait, we run the risk of executing an exit which will terminate
  *	the process and all threads before the threads have completed.
  ****************************************************************************/
  pthread_join(thread1,NULL);
  pthread_join(thread2,NULL);

  printf("Done writing to and reading from data_buf.\n");
  printf("Status of data_buf: read = %d, write = %d, empty = %d, full = %d\n",\
  get_read_index(),get_write_index(),buffer_empty(),buffer_full());

  printf("From main process ID: %d\n", ((int)getpid()));

  exit(EXIT_SUCCESS);
  return 0;
}

void *CAN_thread() {
  uint16_t k;

  int s; // can raw socket
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;

  struct periodic_info info;

  float qa[3] = {-1.6845,-2.6214,-1.4571}; // in degrees: -96.5, -150.2, -83.5
  // -152.2, -170.2, -27.7 (deg) or -2.6564, -2.9706, -0.4835 (rad)
  float qu[6];
  float footPose[3] = {};
  double wrench[3] = {1,1,1};
  // double twist[3] = {1,1,1};
  double torques[3];
  uint8_t didw2tSucceed = 0;

  /****************************************************************************
  * Set up CAN raw socket
  ****************************************************************************/
  // if you don't have access to the CAN bus, comment out from the next line:
  pthread_mutex_lock(&mutex1);

  printf("Beginning CAN socket setup:\n");

  /* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("\tsocket");
    printf("\tsocket error\n");
    // TO-DO: ERROR HANDLING
    return NULL;
	}
  printf("\tsocket open complete\n");

  addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, "can0");
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("\tSIOCGIFINDEX");
    // TO-DO: ERROR HANDLING
    return NULL;
	}
  printf("\tioctl complete\n");
	addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("\tbind");
    printf("\tbind error\n");
    // TO-DO: ERROR HANDLING
    return NULL;
	}
  printf("\tbind complete\n");

  printf("\tsocket: %d\n",s);
  printf("\tsizeof(frame): %d\n",sizeof(frame));

  printf("CAN socket set up complete!\n");

  /* parse bogus CAN frame */
	if (parse_canframe("00003001#0000000000000000", &frame)){
		fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		fprintf(stderr, "Try: <can_id>#{R|data}\n");
		fprintf(stderr, "can_id can have 3 (SFF) or 8 (EFF) hex chars\n");
		fprintf(stderr, "data has 0 to 8 hex-values that can (optionally)");
		fprintf(stderr, " be seperated by '.'\n\n");
		fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / ");
		fprintf(stderr, "5AA# /\n     1F334455#1122334455667788 / 123#R ");
		fprintf(stderr, "for remote transmission request.\n\n");
		return NULL;
	}
  // if you don't have access to the CAN bus, comment out up to the line above.
  // Make sure both mutex lock and unlock are either both commented out or
  // neither commented out.
  pthread_mutex_unlock(&mutex1);

  /****************************************************************************
  * Wait for permission to begin,
  * then send/receive via CAN and put relevant data into circular buffer.
  ****************************************************************************/

  while(!CAN_thread_begin) {;}
  make_periodic(CAN_PERIOD_US, &info); // period (first argument) in microseconds
  for (k = 0; k < BUFLEN;) {
    // read from the CAN bus:
    pthread_mutex_lock(&mutex1);
    // temporary kinematics testing location:

    clock_t tic2 = clock();
    // qa = qaTraj[k];
    geomFK(qaTraj[k],qu,footPose,1);
    subchainIK(qaTraj[k],qu,footPose);
    didw2tSucceed = wrench2torques(qa, qu, torques, wrench);
    clock_t toc2 = clock();

    printf("Did w2t succeed? Yes (0) / No(1): %d\n",didw2tSucceed);
    printf("Calculating took %f seconds\n", (double)(toc2 - tic2) / CLOCKS_PER_SEC);
    printf("torques = [%6.3f, %6.3f, %6.3f]\n",torques[0],torques[1],torques[2]);
    pthread_mutex_unlock(&mutex1);

    // write to the CAN bus:
    frame.data[0] = 0b00101011;
    frame.data[1] = (refTraj[k] & 0x00FF);
    frame.data[2] = (refTraj[k] & 0xFF00) >> 8;
    frame.data[3] = (refTraj[k] & 0x00FF);
    frame.data[4] = (refTraj[k] & 0xFF00) >> 8;
    frame.data[5] = (refTraj[k] & 0x00FF);
    frame.data[6] = (refTraj[k] & 0xFF00) >> 8;
    pthread_mutex_lock(&mutex1);
  	if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
  		perror("write");
  		// return NULL;
  	}
    /* get interface name of the received CAN frame */
    ifr.ifr_ifindex = addr.can_ifindex;
    ioctl(s, SIOCGIFNAME, &ifr);
    pthread_mutex_unlock(&mutex1);
    // put stuff in the circular buffer:
    pthread_mutex_lock(&mutex1);
    printf("CAN thread: %5.3f %5.3f %5.3f\n",qaTraj[k][0],qaTraj[k][1],qaTraj[k][2]);
    buffer_write(qaTraj[k][0],qaTraj[k][1],qaTraj[k][2]);
    pthread_mutex_unlock(&mutex1);
    ++k;
    wait_period(&info);
  }
  close(s); // close the CAN socket
  printf("Write thread has completed.\n");
  return NULL;
}

void *UART_thread() {
  uint16_t j;
  float bufferval[3];
  struct periodic_info info;

  /****************************************************************************
  * Wait for permission to begin,
  * then get relevant data from circular buffer and send via UART
  ****************************************************************************/

  while(!UART_thread_begin) {;}
  make_periodic(UART_PERIOD_US, &info); // period (1st argument) in microseconds
  for (j = 0; j < BUFLEN;) {
    pthread_mutex_lock(&mutex1);
    buffer_read(bufferval);
    // fflush(stdout);
    // sprintf(writemsg,"%d %d %d\n",bufferval[0],bufferval[1],bufferval[2]);
    // dprintf(serial_port,"%d\n",j);
    dprintf(serial_port,"%5.3f %5.3f %5.3f\n",bufferval[0],bufferval[1],bufferval[2]);
    printf("UART thread: %d: %5.3f %5.3f %5.3f\n",j,bufferval[0],bufferval[1],bufferval[2]);
    // serialPuts(serial_port, writemsg);
    ++j;
    pthread_mutex_unlock(&mutex1);

    wait_period(&info);
  }
  printf("Read thread has completed.\n");
  return NULL;
}
