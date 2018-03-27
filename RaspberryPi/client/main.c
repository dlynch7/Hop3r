/*
* CAN interface adapted from cantest.c, from www.skpang.co.uk
* Multithreading by yours truly.
* Dan Lynch
* 3-27-2018
*/

/*
 *  $Id$
 */

/*
 * cansend.c - simple command line tool to send CAN-frames via CAN_RAW sockets
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

/* 20-05-15 Modified to sent a message out in code rather then from parameter.

	www.skpang.co.uk
*/

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>
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

void *CAN_thread();
void *UART_thread();
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
uint8_t begin; // read and write threads must wait for begin = 1
int serial_port;
uint16_t periodms_write = 1; // 1 = 1 kHz, 1000 = 1 Hz
uint16_t periodms_read = 10; // 1 = 1 kHz, 1000 = 1 Hz
char writemsg[10] = {};

uint8_t main(void) {
  int rc1, rc2;
  uint8_t writePermission = 0;

  begin = 0; // reading and writing cannot commence

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
  begin = 1; // reading and writing can commence

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
  uint16_t i;
  unsigned int nextWriteTime;

  /****************************************************************************
  * Set up CAN raw socket
  ****************************************************************************/
  // if you don't have access to the CAN bus, comment out from the next line:
  pthread_mutex_lock(&mutex1);
  int s; // can raw socket
  int nbytes;
  struct sockaddr_can addr;
  struct can_frame frame;
  struct ifreq ifr;

  /* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
    // TO-DO: ERROR HANDLING
    // return 1;
	}

  addr.can_family = AF_CAN;

	strcpy(ifr.ifr_name, "can0");
	if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
		perror("SIOCGIFINDEX");
    // TO-DO: ERROR HANDLING
    // return 1;
	}
	addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
    // TO-DO: ERROR HANDLING
		// return 1;
	}

  printf("CAN socket set up successfully!\n");
  pthread_mutex_unlock(&mutex1);
  // if you don't have access to the CAN bus, comment out up to the line above.
  // Make sure both mutex lock and unlock are either both commented out or
  // neither commented out.

  /****************************************************************************
  * Wait for permission to begin,
  * then send/receive via CAN and put relevant data into circular buffer.
  ****************************************************************************/

  while(!begin) {;}
  nextWriteTime = millis() + periodms_write;
  for (i = 0; i < BUFLEN;) {
    if (millis() > nextWriteTime) {
      pthread_mutex_lock(&mutex1);
      buffer_write(i);
      printf("Wrote %d to data_buf[%d]: read = %d\twrite = %d\tempty = %d\tfull = %d\n",\
      i,get_write_index()-1,get_read_index(),get_write_index(),buffer_empty(),buffer_full());
      nextWriteTime += periodms_write;
      ++i;
      pthread_mutex_unlock(&mutex1);
    }
  }
  printf("Write thread has completed.\n");
}

void *UART_thread() {
  uint16_t j;
  uint16_t bufferval;
  unsigned int nextReadTime;

  /****************************************************************************
  * Wait for permission to begin,
  * then get relevant data from circular buffer and send via UART
  ****************************************************************************/

  while(!begin) {;}
  nextReadTime = millis() + periodms_read;
  for (j = 0; j < BUFLEN;) {
    if (millis() > nextReadTime) {
      pthread_mutex_lock(&mutex1);
      bufferval = buffer_read();
      printf("data_buf[%d] = %d\tread = %d\twrite = %d\tempty = %d\tfull = %d\n",\
      j,bufferval,get_read_index(),get_write_index(),buffer_empty(),buffer_full());

      fflush(stdout);
      sprintf(writemsg,"%d\r\n",bufferval);
      serialPuts(serial_port, writemsg);
      nextReadTime += periodms_read;
      ++j;
      pthread_mutex_unlock(&mutex1);
    }

    // delay(3);

    // while(serialDataAvail(serial_port)) {
    //   printf(" -> %3d", serialGetchar(serial_port));
    //   fflush(stdout);
    // }
  }
  printf("Read thread has completed.\n");
}
