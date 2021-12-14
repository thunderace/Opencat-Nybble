#ifndef __MPU_H__
#define __MPU_H__

#include <protothreads.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#define PACKET_SIZE 42
#define OVERFLOW_THRESHOLD 128
//#if OVERFLOW_THRESHOLD>1024-1024%PACKET_SIZE-1   // when using (1024-1024%PACKET_SIZE) as the overflow resetThreshold, the packet buffer may be broken
// and the reading will be unpredictable. it should be replaced with previous reading to avoid jumping
#define FIX_OVERFLOW
//#endif
#define HISTORY 2
int8_t lag = 0;
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprLag[HISTORY][3];
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
//bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[PACKET_SIZE]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

// ================================================================
// ===               INTERRUPT_PIN DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void getFIFO() {//get FIFO only without further processing
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
}

void getYPR() {//get YPR angles from FIFO data, takes time
  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) ;
  if (mpuInterrupt || fifoCount >= packetSize)
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    //PTL(fifoCount);
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount > OVERFLOW_THRESHOLD) { //1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // otherwise, check for DMP data ready interrupt (this should happen frequently)

      // -- RzLi --
#ifdef FIX_OVERFLOW
      lag = (lag - 1 + HISTORY) % HISTORY;
#endif
      // --
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      getFIFO();

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef MPU_YAW180
      ypr[2] = -ypr[2];
      ypr[1] = -ypr[1];
#endif //!MPU_YAW180
#endif //!OUTPUT_READABLE_YAWPITCHROLL
      for (byte g = 1; g < 3; g++)
        ypr[g] *= degPerRad;        //ypr converted to degree

      // overflow is detected after the ypr is read. it's necessary to keep a lag record of previous reading.  -- RzLi --
#ifdef FIX_OVERFLOW
      for (byte g = 1; g < 3; g++) {
        yprLag[lag][g] = ypr[g];
        ypr[g] = yprLag[(lag - 1 + HISTORY) % HISTORY][g] ;
      }
      lag = (lag + 1) % HISTORY;
#endif //!FIX_OVERFLOW

      PT(ypr[0]);
      PTF("\t");
      PT(ypr[1]);
      PTF("\t");
      PTL(ypr[2]);
    }
  }
}
#if 0
void checkBodyMotion()  {
  //if (!dmpReady) return;
  getYPR();
  // --
  //deal with accidents
  if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//wait until stable
    if (!hold)
      for (byte w = 0; w < 50; w++) {
        getYPR();
        delay(10);
      }
    if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//check again
      if (!hold) {
        token = T_SKILL;
        if (fabs(ypr[2]) > LARGE_ROLL) {
          strcpy(newCmd, "rc");
          newCmdIdx = 4;
        }
        //        else {
        //          strcpy(newCmd, ypr[1] < LARGE_PITCH ? "lifted" : "dropped");
        //          newCmdIdx = 1;
        //        }
      }
      hold = 10;
    }
  }

  // recover
  else if (hold) {
    if (hold == 1) {
      token = T_SKILL;
      strcpy(newCmd, "balance");
      newCmdIdx = 1;
    }
    hold --;
    if (!hold) {
      char temp[CMD_LEN];
      strcpy(temp, newCmd);
      strcpy(newCmd, lastCmd);
      strcpy(lastCmd, temp);
      newCmdIdx = 1;
      meow();
    }
  }
  //calculate deviation
  for (byte i = 0; i < 2; i++) {
    RollPitchDeviation[i] = ypr[2 - i] - motion.expectedRollPitch[i]; //all in degrees
    //PTL(RollPitchDeviation[i]);
    RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0);//filter out small angles
  }

  //PTL(jointIdx);
}
#endif
int checkBodyMotionThread(struct pt* pt)  {
  static byte hold = 0;
  PT_BEGIN(pt);

  // Loop forever
  for(;;) {
      getYPR();
      // --
      //deal with accidents
      if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//wait until stable
        if (!hold) {
            getYPR();
            PT_SLEEP(pt, 10);
        }
        if (fabs(ypr[1]) > LARGE_PITCH || fabs(ypr[2]) > LARGE_ROLL) {//check again
          if (!hold) {
            token = 'k';
            if (fabs(ypr[2]) > LARGE_ROLL) {
              strcpy(newCmd, "rc");
              newCmdIdx = 4;
            }
            else {
              strcpy(newCmd, ypr[1] < LARGE_PITCH ? "lifted" : "dropped");
              newCmdIdx = 1;
            }
          }
          hold = 10;
        }
      }

      // recover
      else if (hold) {
        if (hold == 1) {
          token = 'k';
          strcpy(newCmd, "balance");
          newCmdIdx = 1;
        }
        hold --;
        if (!hold) {
          char temp[CMD_LEN];
          strcpy(temp, newCmd);
          strcpy(newCmd, lastCmd);
          strcpy(lastCmd, temp);
          newCmdIdx = 1;
          meow();
        }
      }
      //calculate deviation
      for (byte i = 0; i < 2; i++) {
        RollPitchDeviation[i] = ypr[2 - i] - motion.expectedRollPitch[i]; //all in degrees
        //PTL(RollPitchDeviation[i]);
        RollPitchDeviation[i] = sign(ypr[2 - i]) * max(fabs(RollPitchDeviation[i]) - levelTolerance[i], 0);//filter out small angles
      }
  PT_SLEEP(pt, 5);
  }
  PT_END(pt);
}


pt ptBodyMotion;

bool mpuSetup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  PTLF("Connect MPU6050");
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  delay(500);
  // verify connection
  PTLF("Test connection");
  PTL(mpu.testConnection() ? F("MPU successful") : F("MPU failed"));//sometimes it shows "failed" but is ok to bypass.

  // load and configure the DMP
  do {
    PTLF("Initialize DMP");
    devStatus = mpu.dmpInitialize();
    delay(500);
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setZAccelOffset(EEPROMReadInt(MPUCALIB + 4));
    mpu.setXGyroOffset(EEPROMReadInt(MPUCALIB + 6));
    mpu.setYGyroOffset(EEPROMReadInt(MPUCALIB + 8));
    mpu.setZGyroOffset(EEPROMReadInt(MPUCALIB + 10));
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      PTLF("Enable DMP");
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      PTLF("Enable interrupt");
      attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      PTLF("DMP ready!");
      //dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      PTLF("DMP failed (code ");
      PT(devStatus);
      PTLF(")");
      PTL();
    }
  } while (devStatus);
  PT_INIT(&ptBodyMotion);  
}
#endif //!__MPU_H__