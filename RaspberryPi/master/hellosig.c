#include <wiringPi.h>
#include <stdlib.h>

#include <signal.h>
#include <unistd.h>

void alarmWakeup(int sig_num);


int main(int argc, char *argv[])
{
    unsigned int j;

    wiringPiSetupPhys();//use the physical pin numbers on the P1 connector


    pinMode(40, OUTPUT);
    pinMode(38, OUTPUT);

    signal(SIGALRM, alarmWakeup);
    ualarm(5000, 5000);


    while(1)
    {
        digitalWrite(40, HIGH); //pin 40 high
        for(j=0; j<1000000; j++);//do something
        digitalWrite(40, LOW);  //pin 40 low
        for(j=0; j<1000000; j++);//do something
    }

    return 0;

}//int main(int argc, char *argv[])


void alarmWakeup(int sig_num)
{
    unsigned int i;

    if(sig_num == SIGALRM)
    {
        digitalWrite(38, HIGH); //pin 38 high
        for(i=0; i<65535; i++); //do something
        digitalWrite(38, LOW);  //pin 38 low
    }

}
