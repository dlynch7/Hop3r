#include "safety.h"

void safety_init(void) {
  wiringPiSetup();
  pinMode(COPLEY_EN, OUTPUT); // COPLEY_EN is wiringPi pin 26 is physical pin 32
  digitalWrite(COPLEY_EN, 0); // setting this pin low enables the Copleys
  signal(SIGINT, &trap);
  printf("Safety enabled");
}

// when CTRL+C pressed, kill motors
void trap(int signal) {
  run_program = 0;
  // exit(EXIT_SUCCESS);
}

int kill_motors(void) { //
  double qa_trq_kill[3] = {0.0, 0.0, 0.0};

  if (writeTrqToCAN(qa_trq_kill)) {
    fprintf(stderr,"Unable to write KILL torques to CAN.\n");
    return 1;
  }

  digitalWrite(COPLEY_EN,1); // setting this pin high disables the Copleys

  return 0;
}
