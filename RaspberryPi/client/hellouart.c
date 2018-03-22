// to compile: gcc -o hellouart hellouart.c -lwiringPi

// https://github.com/WiringPi/WiringPi/blob/master/examples/serialTest.c

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int main (void) {
	int serial_port;
	int count;
	uint16_t len = 256;
	unsigned int nextTime;
	uint8_t writePermission = 0;
	uint16_t periodms = 1;

	char writemsg[10] = {};

	if ((serial_port = serialOpen("/dev/ttyS0", 115200)) < 0) // open serial port
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}

	if (wiringPiSetup() == -1) {
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
	sprintf(writemsg,"%d\r\n",len);
	serialPuts(serial_port, writemsg);

	writePermission = serialGetchar(serial_port);
	printf("writePermission: %c\r\n",writePermission);

	if (writePermission != '1') {
		printf("Write permission denied by client.\r\n");
		return 1;
	}

	if (writePermission=='1') {
		nextTime = millis() + periodms;
		for (count = 0; count < len;) {
			if (millis() > nextTime) {
				printf("\nOut: %3d: ", count);
				fflush(stdout);
				sprintf(writemsg,"%d\r\n",count);
				serialPuts(serial_port, writemsg);
				nextTime += periodms;
				++count;
			}

			delay(3);

			while(serialDataAvail(serial_port)) {
				printf(" -> %3d", serialGetchar(serial_port));
				fflush(stdout);
			}
		}
	}
	printf("\n");

	return 0;
}
