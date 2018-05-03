/*
 serial_interface.c
 Dan Lynch
 Begun May 2 2018

 implements a serial interface on port '/dev/ttyS0'

 Reference: http://mirror.datenwolf.net/serial/

 */

#include <stdio.h>    /* Standard input/output definitions */
#include <stdint.h>   /* Integer types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */

#include "serial_interface.h"

/*
 * 'open_port()' - Open serial port '/dev/ttyS0'.
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(void)
{
  int fd; /* File descriptor for the port */

  fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)
  { //Could not open the port.
    perror("open_port: Unable to open /dev/ttyS0 - ");
  }
  else {
    fcntl(fd, F_SETFL, 0); // configure for blocking read
    // wait, then flush the OS's buffer on the serial port:
    usleep(250000);
    tcflush(fd, TCIOFLUSH);
  }
  return (fd);
}

void config_port(int fd) {
  struct termios options;

  tcgetattr(fd, &options); // Get the current options for the port

  // Set the baud rates to 19200
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode

  tcsetattr(fd, TCSANOW, &options); // Set the new options for the port
}

int serial_write(int fd, char *buf, uint16_t length) {
  uint16_t n;
  char err_msg[50];

  n = write(fd, buf, length);
  if (n < 0) {
    sprintf(err_msg,"write() of %d bytes failed!\n",length);
    perror(err_msg);
  }

  return n;

}
