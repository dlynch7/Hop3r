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

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>	// needed for getpid()
#include <time.h>
#include <unistd.h>			// needed for getpid()
#include <wiringPi.h>
#include <wiringSerial.h>

#include "linux-can-utils/lib.h"
#include "circ_buffer.h"
#include "per_threads.h"

#define CAN_PERIOD_US 5000
#define UART_PERIOD_US 50000

void *CAN_thread();
void *UART_thread();
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
uint8_t CAN_thread_begin; // read and write threads must wait for begin = 1
uint8_t UART_thread_begin; // read and write threads must wait for begin = 1
int serial_port;
char writemsg[10] = {};

// static int make_periodic(int unsigned period, struct periodic_info *info)
// {
// 	static int next_sig;
// 	int ret;
// 	unsigned int ns;
// 	unsigned int sec;
// 	struct sigevent sigev;
// 	timer_t timer_id;
// 	struct itimerspec itval;
//
// 	/* Initialise next_sig first time through. We can't use static
// 	   initialisation because SIGRTMIN is a function call, not a constant */
// 	if (next_sig == 0)
// 		next_sig = SIGRTMIN;
// 	/* Check that we have not run out of signals */
// 	if (next_sig > SIGRTMAX)
// 		return -1;
// 	info->sig = next_sig;
// 	next_sig++;
// 	/* Create the signal mask that will be used in wait_period */
// 	sigemptyset(&(info->alarm_sig));
// 	sigaddset(&(info->alarm_sig), info->sig);
//
// 	/* Create a timer that will generate the signal we have chosen */
// 	sigev.sigev_notify = SIGEV_SIGNAL;
// 	sigev.sigev_signo = info->sig;
// 	sigev.sigev_value.sival_ptr = (void *)&timer_id;
// 	ret = timer_create(CLOCK_MONOTONIC, &sigev, &timer_id);
// 	if (ret == -1)
// 		return ret;
//
// 	/* Make the timer periodic */
// 	sec = period / 1000000;
// 	ns = (period - (sec * 1000000)) * 1000;
// 	itval.it_interval.tv_sec = sec;
// 	itval.it_interval.tv_nsec = ns;
// 	itval.it_value.tv_sec = sec;
// 	itval.it_value.tv_nsec = ns;
// 	ret = timer_settime(timer_id, 0, &itval, NULL);
// 	return ret;
// }
//
// static void wait_period(struct periodic_info *info)
// {
// 	int sig;
// 	sigwait(&(info->alarm_sig), &sig);
// }

int main(void) {
  int rc1, rc2;
  uint8_t writePermission = 0;

  if (setup_periodic()) {
    fprintf(stderr, "Failed to setup periodic threads.\n");
    return 1;
  }
  // sigset_t alarm_sig;
	// int i;
  //
	// printf("Periodic threads using POSIX timers\n");
  //
	// /* Block all real time signals so they can be used for the timers.
	//    Note: this has to be done in main() before any threads are created
	//    so they all inherit the same mask. Doing it later is subject to
	//    race conditions */
	// sigemptyset(&alarm_sig);
	// for (i = SIGRTMIN; i <= SIGRTMAX; i++)
	// 	sigaddset(&alarm_sig, i);
	// sigprocmask(SIG_BLOCK, &alarm_sig, NULL);

  CAN_thread_begin = 0; // reading and writing cannot commence
  UART_thread_begin = 0; // reading and writing cannot commence

  printf("This is the main function.\n");

  if ((serial_port = serialOpen("/dev/ttyS0", 115200)) < 0) // open serial port
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}

	if (wiringPiSetup() == -1) {
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
	sprintf(writemsg,"%d\r\n",BUFLEN);
	serialPuts(serial_port, writemsg);

	writePermission = serialGetchar(serial_port);
	printf("writePermission: %c\r\n",writePermission);

	if (writePermission != '1') {
		printf("Write permission denied by client.\r\n");
		return 1;
	}

  printf("Status of data_buf: read = %d, write = %d, empty = %d, full = %d\n",\
  get_read_index(),get_write_index(),buffer_empty(),buffer_full());

  /****************************************************************************
	*	Create two independent threads.
  * CAN_thread will write to a buffer.
  * UART_thread will read from the buffer.
	****************************************************************************/

  // // initialized with default attributes:
  // ret = pthread_attr_getschedparam (&tattr, &param);
  // // safe to get existing cheduling param:
  // ret = pthread_attr_getschedparam (&tattr, &param);
  // // set the priority; others are unchanged:
  // param.sched_priority = newprio;
  // // setting the new scheduling param:
  // ret = pthread_attr_setschedparam (&tattr, &param);


	pthread_t thread1, thread2;
  if ( (rc1=pthread_create(&thread1,NULL,&CAN_thread,NULL)) ) {
		printf("Thread creation failed: %d\n", rc1);
	}
	if ( (rc2=pthread_create(&thread2,NULL,&UART_thread,NULL)) ) {
		printf("Thread creation failed: %d\n", rc2);
	}

  printf("From main process ID: %d\n", ((int)getpid()));
  CAN_thread_begin = 1; // reading and writing can commence
  // some kind of delay here?
  UART_thread_begin = 1; // reading and writing can commence

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
  // unsigned int nextWriteTime;

  int s; // can raw socket
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;

  struct periodic_info info;

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
    // return 1;
	}
  printf("\tsocket open complete\n");

  addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, "can0");
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("\tSIOCGIFINDEX");
    printf("\tSIOCGIFINDEX\n");
    // TO-DO: ERROR HANDLING
    // return 1;
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
		// return 1;
	}
  printf("\tbind complete\n");

  printf("\tsocket: %d\n",s);
  printf("\tsizeof(frame): %d\n",sizeof(frame));

  printf("CAN socket set up complete!\n");

  /* parse bogus CAN frame */
  // char data[]
	if (parse_canframe("000#00000000", &frame)){
		fprintf(stderr, "\nWrong CAN-frame format!\n\n");
		fprintf(stderr, "Try: <can_id>#{R|data}\n");
		fprintf(stderr, "can_id can have 3 (SFF) or 8 (EFF) hex chars\n");
		fprintf(stderr, "data has 0 to 8 hex-values that can (optionally)");
		fprintf(stderr, " be seperated by '.'\n\n");
		fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / ");
		fprintf(stderr, "5AA# /\n     1F334455#1122334455667788 / 123#R ");
		fprintf(stderr, "for remote transmission request.\n\n");
		// return 1;
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
  // nextWriteTime = millis() + periodms_write;
  for (k = 0; k < BUFLEN;) {
    // if (millis() > nextWriteTime) {
      // read from the CAN bus:
      // printf("next line: read CAN socket\n");
      /* send frame */
      frame.data[0] = (k & 0x00FF);
      frame.data[1] = (k & 0x0F00) >> 8;
      pthread_mutex_lock(&mutex1);
    	if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
    		perror("write");
        printf("write error: nbytes = %d\n",nbytes);
    		// return 1;
    	}
      /* get interface name of the received CAN frame */
      ifr.ifr_ifindex = addr.can_ifindex;
      ioctl(s, SIOCGIFNAME, &ifr);
      printf("Received a CAN frame from interface %s\n", ifr.ifr_name);
      pthread_mutex_unlock(&mutex1);
      // put stuff in the circular buffer:
      pthread_mutex_lock(&mutex1);
      buffer_write(k);
      printf("Wrote %d to data_buf[%d]: read = %d\twrite = %d\tempty = %d\tfull = %d\n",\
      k,get_write_index()-1,get_read_index(),get_write_index(),buffer_empty(),buffer_full());
      // nextWriteTime += periodms_write;
      ++k;
      pthread_mutex_unlock(&mutex1);
    // }
    wait_period(&info);
  }
  close(s); // close the CAN socket
  printf("Write thread has completed.\n");

  return NULL;
}

void *UART_thread() {
  uint16_t j;
  uint16_t bufferval;
  // unsigned int nextReadTime;
  struct periodic_info info;

  /****************************************************************************
  * Wait for permission to begin,
  * then get relevant data from circular buffer and send via UART
  ****************************************************************************/

  while(!UART_thread_begin) {;}
  // nextReadTime = millis() + periodms_read;
  make_periodic(UART_PERIOD_US, &info); // period (1st argument) in microseconds
  for (j = 0; j < BUFLEN;) {
    // if (millis() > nextReadTime) {
      pthread_mutex_lock(&mutex1);
      // if (get_write_index() >= get_read_index()) {
        bufferval = buffer_read();
        printf("data_buf[%d] = %d\tread = %d\twrite = %d\tempty = %d\tfull = %d\n",\
        j,bufferval,get_read_index(),get_write_index(),buffer_empty(),buffer_full());

        fflush(stdout);
        sprintf(writemsg,"%d\r\n",bufferval);
        serialPuts(serial_port, writemsg);
        // nextReadTime += periodms_read;
        ++j;
      // }
      pthread_mutex_unlock(&mutex1);
    // }

    // delay(3);

    // while(serialDataAvail(serial_port)) {
    //   printf(" -> %3d", serialGetchar(serial_port));
    //   fflush(stdout);
    // }
    wait_period(&info);
  }
  printf("Read thread has completed.\n");

  return NULL;
}
