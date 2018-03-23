#include <errno.h>
#include <pthread.h>
// #include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>	// needed for getpid()
#include <unistd.h>			// needed for getpid()
#include <wiringPi.h>
#include <wiringSerial.h>
#include "circ_buffer.h"

void *write_thread();
void *read_thread();
pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
uint8_t begin; // read and write threads must wait for begin = 1
int serial_port;
uint16_t periodms_write = 1; // 1 = 1 kHz, 1000 = 1 Hz
uint16_t periodms_read = 10; // 1 = 1 kHz, 1000 = 1 Hz
char writemsg[10] = {};

uint8_t main(void) {
  int rc1, rc2;
  // pthread_attr_t tattr;
  // int ret;
  // int newprio = 20;
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
  * One thread will write to a buffer.
  * The other thread will read from the buffer.
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
  if ( (rc1=pthread_create(&thread1,NULL,&write_thread,NULL)) ) {
		printf("Thread creation failed: %d\n", rc1);
	}
	if ( (rc2=pthread_create(&thread2,NULL,&read_thread,NULL)) ) {
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

void *write_thread() {
  uint16_t i;
  unsigned int nextWriteTime;

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

void *read_thread() {
  uint16_t j;
  uint16_t bufferval;
  unsigned int nextReadTime;

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
