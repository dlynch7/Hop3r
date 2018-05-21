#include "can_io.h"

int initSocketCAN(void) { // set up CAN raw socket
  printf("Beginning CAN socket setup:\n");

  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
  	perror("\tsocket");
    printf("\tsocket error\n");
    return 1;
  }
  printf("\tsocket open complete\n");

  addr.can_family = AF_CAN;

  strcpy(ifr.ifr_name, "can0");
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
  	perror("\tSIOCGIFINDEX");
    return 1;
  }
  printf("\tioctl complete\n");
  addr.can_ifindex = ifr.ifr_ifindex;


  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
  	perror("\tbind");
    printf("\tbind error\n");
    return 1;
  }
  printf("\tbind complete\n");

  printf("\tsocket: %d\n",s);
  printf("\tsizeof(writeFrame): %d\n",sizeof(writeFrame));

  printf("CAN socket set up complete!\n");

  /* parse bogus CAN frame */
  if (parse_canframe("00004001#0000000000000000", &writeFrame)){
  	fprintf(stderr, "\nWrong CAN-frame format!\n\n");
  	fprintf(stderr, "Try: <can_id>#{R|data}\n");
  	fprintf(stderr, "can_id can have 3 (SFF) or 8 (EFF) hex chars\n");
  	fprintf(stderr, "data has 0 to 8 hex-values that can (optionally)");
  	fprintf(stderr, " be seperated by '.'\n\n");
  	fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / ");
  	fprintf(stderr, "5AA# /\n     1F334455#1122334455667788 / 123#R ");
  	fprintf(stderr, "for remote transmission request.\n\n");
  	return 1;
  }
  printf("CAN writeFrame ID: %X\n",writeFrame.can_id);

  return 0;
}

// get data from CAN, parse it, and put it into a struct of type can_input_struct:
int readCAN(can_input_struct *ptr) {
  nbytesR = read(s, &readFrame, sizeof(readFrame));
  printf("Read %d bytes:\n", nbytesR);
  printf("\tframe.can_id  = %X\n",readFrame.can_id);
  printf("\tmasked ID  = %X\n",readFrame.can_id & 0x0FFFFFFF);

  switch (readFrame.can_id & 0x0FFFFFFF) {
    // if the received CAN frame ID indicates a joint position:
    case MOTOR_1_POS_CAN_ID:
    {
      ptr->qa_act[0] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case MOTOR_2_POS_CAN_ID:
    {
      ptr->qa_act[1] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case MOTOR_3_POS_CAN_ID:
    {
      ptr->qa_act[2] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    // if the received CAN frame ID indicates a motor current:
    case MOTOR_1_CUR_CAN_ID:
    {
      ptr->ia[0] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case MOTOR_2_CUR_CAN_ID:
    {
      ptr->ia[1] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case MOTOR_3_CUR_CAN_ID:
    {
      ptr->ia[2] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    // if the received CAN frame ID indicates a boom angle:
    case BOOM_ROLL_CAN_ID:
    {
      ptr->boom[0] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case BOOM_PITCH_CAN_ID:
    {
      ptr->boom[1] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    case BOOM_YAW_CAN_ID:
    {
      ptr->boom[2] = ((readFrame.data[1] << 8) | readFrame.data[0]);
      break;
    }
    // if the received CAN frame ID indicates IMU/FZ information:
    case IMU_FZ_CAN_ID:
    {
      ptr->accel = ((readFrame.data[1] << 8) | readFrame.data[0]);
      // optionally expand for accelerations in 3 axes
      ptr->fz = ((readFrame.data[3] << 8) | readFrame.data[2]);
      break;
    }
    default: // CAN frame does not match any known IDs
      fprintf(stderr,"The received CAN frame does not match any known IDs.\n");
      return 1;
  }
  return 0;
}

// write 3 reference joint positions to CAN:
int writePosToCAN(double *pos_deg_arr) {
  int16_t qa_deg10[3];

  // *pos_deg_arr is a pointer to an array of 3 joint positions, represented as doubles.
  // The motor controllers expect reference positions in 1/10ths of a degree,
  // represented as int16_t's, so we must multiply the elements in pos_deg_arr
  // by 10, and then cast to int16_t:
  qa_deg10[0] = ((int) (10*pos_deg_arr[0]));
  qa_deg10[1] = ((int) (10*pos_deg_arr[1]));
  qa_deg10[2] = ((int) (10*pos_deg_arr[2]));

  writeFrame.can_id = MOTOR_CMD_ID;
  // set mode to position control and enable motors:
  writeFrame.data[0] = (MODE_POS_CTRL | MOTOR_3_EN | MOTOR_2_EN | MOTOR_1_EN);
  // set data bytes to reference positions:
  writeFrame.data[1] = (qa_deg10[0] & 0x00FF);
  writeFrame.data[2] = (qa_deg10[0] & 0xFF00) >> 8;
  writeFrame.data[3] = (qa_deg10[1] & 0x00FF);
  writeFrame.data[4] = (qa_deg10[1] & 0xFF00) >> 8;
  writeFrame.data[5] = (qa_deg10[2] & 0x00FF);
  writeFrame.data[6] = (qa_deg10[2] & 0xFF00) >> 8;

  pthread_mutex_lock(&mutex1);
  if ((nbytesW = write(s, &writeFrame, sizeof(writeFrame))) != sizeof(writeFrame)) {
    perror("write");
    pthread_mutex_unlock(&mutex1); // still have to unlock the mutex!
    return 1;
  }
  pthread_mutex_unlock(&mutex1);

  return 0;
}

// write 3 reference joint torques to CAN:
int writeTrqToCAN(double *trq_Nm_arr) {
  int16_t qa_trq_mNm[3];

  // *trq_Nm_arr is a pointer to an array of 3 joint positions, represented as doubles.
  // The motor controllers expect reference torques in mNm,
  // represented as int16_t's, so we must multiply the elements in trq_Nm_arr
  // by 1000, and then cast to int16_t:
  qa_trq_mNm[0] = ((int) (1000*trq_Nm_arr[0]));
  qa_trq_mNm[1] = ((int) (1000*trq_Nm_arr[1]));
  qa_trq_mNm[2] = ((int) (1000*trq_Nm_arr[2]));

  writeFrame.can_id = MOTOR_CMD_ID;
  // set mode to torque control and enable motors:
  writeFrame.data[0] = (MODE_TRQ_CTRL | MOTOR_3_EN | MOTOR_2_EN | MOTOR_1_EN);
  // set data bytes to reference torques:
  writeFrame.data[1] = (qa_trq_mNm[0] & 0x00FF);
  writeFrame.data[2] = (qa_trq_mNm[0] & 0xFF00) >> 8;
  writeFrame.data[3] = (qa_trq_mNm[1] & 0x00FF);
  writeFrame.data[4] = (qa_trq_mNm[1] & 0xFF00) >> 8;
  writeFrame.data[5] = (qa_trq_mNm[2] & 0x00FF);
  writeFrame.data[6] = (qa_trq_mNm[2] & 0xFF00) >> 8;

  pthread_mutex_lock(&mutex1);
  if ((nbytesW = write(s, &writeFrame, sizeof(writeFrame))) != sizeof(writeFrame)) {
    perror("write");
    pthread_mutex_unlock(&mutex1); // still have to unlock the mutex!
    return 1;
  }
  pthread_mutex_unlock(&mutex1);

  return 0;
}
