/*
    Skill class holds only the lookup information of joint angles.
    One frame of joint angles defines a static posture, while a series of frames defines a periodic motion, usually a gait.
    Skills are instantiated as either:
      instinct  (trained by Rongzhong Li, saved in external i2c EERPOM) or
      newbility (taught by other users, saved in PROGMEM)
    A well-tuned (finalized) newbility can also be saved in external i2c EEPROM. Remember that EEPROM has very limited (1,000,000) write cycles!

    SkillList (inherit from QList class) holds a mixture of instincts and newbilities.
    It also provides a dict(key) function to return the pointer to the skill.
    Initialization information(individual skill name, address) for SkillList is stored in on-board EEPROM

    Behavior list (inherit from QList class) holds a time dependent sequence of multiple skills, triggered by certain perceptions.
    It defines the order, speed, repetition and interval of skills。
    (Behavior list is yet to be implemented)

    Motion class uses the lookup information of a Skill to construct a Motion object that holds the actual angle array.
    It also implements the reading and writing functions in specific storage locations.
    Considering Arduino's limited SRAM, you should create only one Motion object and update it when loading new skills.

    instinct(external EEPROM) \
                                -- skill that contains only lookup information
    newbility(progmem)        /

    Skill list: skill1, skill2, skill3,...
                              |
                              v
                           motion that holds actual joint angle array in SRAM

    Behavior list: skill3(speed, repetition and interval), skill1(speed, repetition and interval), ...

    **
    Updates: One Skill object in the SkillList takes around 20 bytes in SRAM. It takes 200+ bytes for 15+ skills.
    On a tiny atmega328 chip with only 2KB SRAM, I'm implementing the Skills and SkillList in the on-board EEPROM。
    Now the skill list starts from on-board EEPROM address SKILLS.
    Format:
    1 byte skill_1 nameLength + char string name1 + 1 char skillType1 + 1 int address1,
    1 byte skill_2 nameLength + char string name2 + 1 char skillType2 + 1 int address2,
    ...
    The iterator can traverse the list with the string length of each skill name.

    The Skill and SkillList classes are obsolete in the atmega328 implementation but are still included in this header file.
    **

  Rongzhong Li
  January 2021

  Copyright (c) 2021 Petoi LLC.

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

*/
//postures and movements trained by RongzhongLi
//#include "InstinctBittle.h" //activate the correct header file according to your model
#include "InstinctNybble.h"

#ifdef ESP8266
#ifndef min
#define min(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#endif
#endif

//#define NyBoard_V0_1
//#define NyBoard_V0_2
//#define NyBoard_V1_0
#define Arduino_Board
//#define DEVELOPER
#ifdef DEVELOPER
#include <MemoryFree.h> //http://playground.arduino.cc/Code/AvailableMemory
#include <QList.h> //https://github.com/SloCompTech/QList
#endif

// credit to Adafruit PWM servo driver library
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
//#include <avr/eeprom.h> // doesn't work. <EEPROM.h> works

//abbreviations
#define PT(s) Serial.print(s)  //makes life easier
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))//trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

//board configuration
#define INTERRUPT 0
#define IR_RECEIVER 4 // Signal Pin of IR receiver to Arduino Digital Pin 4
#define BUZZER 5
#define GYRO
#define ULTRA_SOUND
int lightLag = 0;

void beep(int8_t note, float duration = 10, int pause = 0, byte repeat = 1 ) {
  if (note == 0) {//rest note
    analogWrite(BUZZER, 0);
    delay(duration);
    return;
  }
  int freq = 220 * pow(1.059463, note - 1); // 1.059463 comes from https://en.wikipedia.org/wiki/Twelfth_root_of_two
  float period = 1000000.0 / freq;
  for (byte r = 0; r < repeat; r++) {
    for (float t = 0; t < duration * 1000; t += period) {
      analogWrite(BUZZER, 150);      // Almost any value can be used except 0 and 255
      // experiment to get the best tone
      delayMicroseconds(period / 2);        // rise for half period
      analogWrite(BUZZER, 0);       // 0 turns it off
      delayMicroseconds(period / 2);        // down for half period
    }
    delay(pause);
  }
}
void playMelody() {
  static int8_t melody[] = {8, 13, 15,//10, 13, 8,  0,  5,  8,  3,  5, 8, // tone
                   8, 8, 4 //32, 32, 8, 32, 32, 32, 32, 32, 8, //relative duration, 8 means 1/8 note length
                  };
  static byte len = sizeof(melody)/2;
  for (byte i = 0; i < len; i++)
    beep(melody[i], 1000 / melody[len + i], 100);
}

void meow(int repeat = 0, int pause = 200, int startF = 50,  int endF = 200, int increment = 5) {
  for (int r = 0; r < repeat + 1; r++) {
    for (int amp = startF; amp <= endF; amp += increment) {
      analogWrite(BUZZER, amp);
      delay(15); // wait for 15 milliseconds to allow the buzzer to vibrate
    }
    delay(100 + 500 / increment);
    analogWrite(BUZZER, 0);
    if (repeat)delay(pause);
  }
}

//token list
#define T_ABORT     'a'
#define T_BEEP      'b'
#define T_CALIBRATE 'c'
#define T_REST      'd'
#define T_GYRO      'g'
#define T_HELP      'h'
#define T_INDEXED   'i'
#define T_JOINTS    'j'
#define T_SKILL     'k'
#define T_LISTED    'l'
#define T_MOVE      'm'
#define T_SIMULTANEOUS_MOVE 'M'
#define T_MELODY    'o'
#define T_PAUSE     'p'
#define T_RAMP      'r'
#define T_SAVE      's'
#define T_MEOW      'u'
#define T_UNDEFINED 'w'
#define T_XLEG      'x'

//abbreviation //gait/posture/function names
#define K00 "d"       //rest and shutdown all servos 
#define K01 "F"       //forward
#define K02 "g"       //turn off gyro feedback to boost speed

#define K10 "L"       //left
#define K11 "balance" //neutral stand up posture
#define K12 "R"       //right

#define K20 "p"       //pause motion and shut off all servos 
#define K21 "B"       //backward
#define K22 "c"       //calibration mode with IMU turned off

#define K30 "vt"      //stepping
#define K31 "cr"      //crawl
#define K32 "wk"      //walk

#define K40 "tr"      //trot
#define K41 "sit"     //sit
#define K42 "str"     //stretch

#define K50 "hi"      //greeting
#define K51 "pu"      //push up
#define K52 "pee"     //standng with three legs

#ifdef NYBBLE
#define K60 "lu"      //look up
#define K61 "buttUp"    //butt up
#else //BITTLE
#define K60 "ck"      //check around
#define K61 "pd"      //play dead
#endif
#define K62 "zero"    //zero position


String gait = "wk";
char direct = 'F';
int8_t tStep = 1;

/* Joint Idx : 
  Head Pan  0
  Head Tilt 1
  Tail Pan  2
  N/A       3
  N/A       4
  N/A       5
  N/A       6
  N/A       7
  Shoulder Front Left 8
  Shoulder Front Right  9
  Shoulder Rear Right 10
  Shoulder Rear Left  11
  Knee Front Left 12
  Knee Front Right  13
  Knee Rear Right 14
  Knee Rear Left  15
  Tail Tilt - (Not present on the original nybble
 */


#ifdef NyBoard_V0_1
byte pins[] = {7, 0, 8, 15,
               6, 1, 14, 9,
               5, 2, 13, 10,
               4, 3, 12, 11
              };
#define BATT A0 //voltage detector
#define LOW_BATT 440
#define DEVICE_ADDRESS 0x50     //I2C Address of eeprom chip         
#define BAUD_RATE 57600

#elif defined NyBoard_V0_2
byte pins[] = {4, 3, 11, 12,
               5, 2, 13, 10,
               6, 1, 14, 9,
               7, 0, 15, 8
              };
#define BATT A0
#define LOW_BATT 440
#define DEVICE_ADDRESS 0x50
#define BAUD_RATE 57600

#elif defined NyBoard_V1_0
byte pins[] = {12, 11, 3, 4,
               13, 10, 5, 2,
               14, 9, 6, 1,
               15, 8, 7, 0
              };
#define BATT A7
#define LOW_BATT 640
#define DEVICE_ADDRESS 0x54
#define BAUD_RATE 115200
#elif defined Arduino_Board
byte pins[] = {8, 9, 10, 11,
               15, 14, 13, 12,
               0, 2, 6, 4,
               1, 3, 7, 5
              };
#ifdef ESP8266
#define BATT D3
#else
#define BATT A0
#endif
#define LOW_BATT 640
#define DEVICE_ADDRESS 0x54
#define BAUD_RATE 115200
#endif

#ifdef NYBBLE
#define HEAD
#define TAIL
#ifdef Arduino_Board
#define TAIL_TILT
#endif //!Arduino_Board
#define X_LEG
#define WALKING_DOF 8
#else
#ifdef BITTLE
#define HEAD
#define LL_LEG
#define WALKING_DOF 8
#define P1S
#endif //!BITTLE
#endif //!NYBBLE

//on-board EEPROM addresses
#define CALIB 0              // 16 byte array
#define MPUCALIB 16           

#define ADAPT_PARAM 160          // 16 x NUM_ADAPT_PARAM byte array
#define NUM_ADAPT_PARAM  2    // number of parameters for adaption

//servo constants
#define DOF 16
#define PWM_FACTOR 1
#define MG92B_MIN 170*PWM_FACTOR
#define MG92B_MAX 550*PWM_FACTOR
#define MG92B_RANGE 150

#define MG90D_MIN 158*PWM_FACTOR //so mg92b and mg90 are not centered at the same signal
#define MG90D_MAX 515*PWM_FACTOR
#define MG90D_RANGE 150 // $$AL$$ in degrees???

#define MG90S_MIN 158*PWM_FACTOR  // $$AL$$ TODO update
#define MG90S_MAX 620*PWM_FACTOR  // $$AL$$ TODO update
#define MG90S_RANGE 150           // $$AL$$ TODO update

#define P1S_MIN 180*PWM_FACTOR
#define P1S_MAX 620*PWM_FACTOR
#define P1S_RANGE 250


// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#ifdef P1S
#define SERVOMIN  P1S_MIN // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  P1S_MAX // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_ANG_RANGE P1S_RANGE
#else
#ifdef MG90S
#define SERVOMIN  MG90S_MIN 
#define SERVOMAX  MG90S_MAX 
#define SERVO_ANG_RANGE MG90S_RANGE
#else
#define SERVOMIN  MG92B_MIN 
#define SERVOMAX  MG92B_MAX 
#define SERVO_ANG_RANGE MG92B_RANGE
#endif
#endif
#define PWM_RANGE (SERVOMAX - SERVOMIN)

float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;
#ifdef BITTLE
int8_t middleShifts[] = {0, 15, 0, 0,
                         -45, -45, -45, -45,
                         45, 45, -45, -45,
                         -75, -75, -75, -75
                        };
#else
int8_t middleShifts[] = {0, 15, 0, 0,
                         -45, -45, -45, -45,
                         0, 0, 0, 0,
                         0, 0, 0, 0
                        };
#endif
#ifdef Arduino_Board
int8_t rotationDirections[] = {1, -1, 1, 1, //$$AL$$ TODO : update
                               1, -1, -1, 1,
                               -1, 1, 1, -1,
                               -1, 1, 1, -1
                              };
#else
int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };
#endif //!Arduino_Board                              
byte servoAngleRanges[] =  {SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE,
                            SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE,
                            SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE,
                            SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE, SERVO_ANG_RANGE,
                           };
float pulsePerDegree[DOF] = {};
int8_t servoCalibs[DOF] = {};
int currentAng[DOF] = {};
float currentAdjust[DOF] = {};
int calibratedDuty0[DOF] = {};

//control related variables
char token;
char lastToken;
#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
byte hold = 0;
int8_t offsetLR = 0;
bool checkGyro = true;
int8_t skipGyro = 2;

#define COUNT_DOWN 60

uint8_t timer = 0;
//#define SKIP 1
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstMotionJoint;
byte jointIdx = 0;

//bool Xconfig = false;
unsigned long usedTime = 0;

//--------------------

//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);
#ifdef ESP8266
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
#else  
  EEPROM.update(p_address, lowByte);
  EEPROM.update(p_address + 1, highByte);
#endif
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

#define WIRE_BUFFER 30 //Arduino wire allows 32 byte buffer, with 2 byte for address.
#define WIRE_LIMIT 16 //That leaves 30 bytes for data. use 16 to balance each writes
#define PAGE_LIMIT 32 //AT24C32D 32-byte Page Write Mode. Partial Page Writes Allowed
#define EEPROM_SIZE (65536/8)

class Motion {
  public:
    byte pins[DOF];           //the mapping between PCB pins and the joint indexes
    int8_t period;            //the period of a skill. 1 for posture, >1 for gait, <-1 for behavior
    int expectedRollPitch[2]; //expected body orientation (roll, pitch)
    byte angleDataRatio;      //divide large angles by 1 or 2. if the max angle of a skill is >128, all the angls will be divided by 2
    byte loopCycle[3];        //the looping section of a behavior (starting row, ending row, repeating cycles)
    char* dutyAngles;         //the data array for skill angles and parameters
    Motion() {
      period = 0;
      expectedRollPitch[0] = 0;
      expectedRollPitch[1] = 0;
      dutyAngles = NULL;
    }

    const char *lookupAddressByName(char* skillName) {
/*
 *   const char* skillNameWithType[]={"bdI","biI","bkI","bkLI","bkRI","crFI","crLI","crRI","lyI","trFI","trLI","trRI","vtI","wkFI","wkLI","wkRI","balanceI","buttUpI","calibI","droppedI","liftedI","luI","restI","sitI","sleepI","strI","zeroN","hiI","peeI","puI","rcI","standI",};
  const char* progmemPointer[] = {bd, bi, bk, bkL, bkR, crF, crL, crR, ly, trF, trL, trR, vt, wkF, wkL, wkR, balance, buttUp, calib, dropped, lifted, lu, rest, sit, sleep, str, zero, hi, pee, pu, rc, stand, };
 */
      byte skillNameLength = strlen(skillName);
      for (byte i = 0; i < NUM_SKILLS; i++) {
        if (!strncmp(skillNameWithType[i], skillName, skillNameLength)) {
          return progmemPointer[i];
        }
      }
      return NULL;
    }

    void loadBySkillName(char* skillName) {
      const char *skillAddr = lookupAddressByName(skillName);
      if (skillAddr == NULL)
        return;
      delete[] dutyAngles;
#ifdef DEVELOPER
      PTF("free memory: ");
      PTL(freeMemory());
#endif
      period = skillAddr[0];
      expectedRollPitch[0] = (int8_t)skillAddr[1];
      expectedRollPitch[1] = (int8_t)skillAddr[2];
      angleDataRatio = skillAddr[3];
      byte skillHeader = SKILL_HEADER_LENGTH;
      byte frameSize;
      if (period < -1) {
        frameSize = 20;
        for (byte i = 0; i < 3; i++)
          loopCycle[i] = skillAddr[skillHeader + i];
        skillHeader += 3;
      } else {
        frameSize = period > 1 ? WALKING_DOF : 16;
      }
      int len = abs(period) * frameSize;
      dutyAngles = new char[len];
      for (int k = 0; k < len; k++) {
        dutyAngles[k] = skillAddr[skillHeader + k];
      }
#ifdef DEVELOPER
      PTF("free memory: ");
      PTL(freeMemory());
#endif
    }
    void info() {
      PTL("period: " + String(period) + ",\tdelayBetweenFrames: " + ",\texpected (pitch,roll): (" + expectedRollPitch[0]*degPerRad + "," + expectedRollPitch[1]*degPerRad + ")");
      for (int k = 0; k < period * (period > 1 ? WALKING_DOF : 16); k++) {
        PT(String((int8_t)dutyAngles[k]) + ", ");
      }
      PTL();
    }
};

Motion motion;

// balancing parameters
#define ROLL_LEVEL_TOLERANCE 3//the body is still considered as level, no angle adjustment
#define PITCH_LEVEL_TOLERANCE 2
float levelTolerance[2] = {ROLL_LEVEL_TOLERANCE, PITCH_LEVEL_TOLERANCE}; //the body is still considered as level, no angle adjustment

#define LARGE_ROLL 90
#define LARGE_PITCH 75

//the following coefficients will be divided by radPerDeg in the adjust() function. so (float) 0.1 can be saved as (int8_t) 1
//this trick allows using int8_t array insead of float array, saving 96 bytes and allows storage on EEPROM
#define panF 60
#define tiltF 60
#define sRF 50    //shoulder roll factor
#define sPF 12    //shoulder pitch factor
#define uRF 60    //upper leg roll factor
#define uPF 30    //upper leg pitch factor
#define lRF (-1.5*uRF)  //lower leg roll factor 
#define lPF (-1.5*uPF)  //lower leg pitch factor
#define LEFT_RIGHT_FACTOR 1.2
#define FRONT_BACK_FACTOR 1.2
#define POSTURE_WALKING_FACTOR 0.5
#ifdef POSTURE_WALKING_FACTOR
float postureOrWalkingFactor;
#endif

#ifdef X_LEG  // >< leg
int8_t adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, { -uRF, uPF}, { -uRF, uPF},
  {lRF, lPF}, {lRF, lPF}, { -lRF, lPF}, { -lRF, lPF}
};
#else         // >> leg
int8_t adaptiveParameterArray[16][NUM_ADAPT_PARAM] = {
  { -panF, 0}, { -panF / 2, -tiltF}, { -2 * panF, 0}, {0, 0},
  {sRF, -sPF}, { -sRF, -sPF}, { -sRF, sPF}, {sRF, sPF},
  {uRF, uPF}, {uRF, uPF}, {uRF, uPF}, {uRF, uPF},
  {lRF, -0.5 * lPF}, {lRF, -0.5 * lPF}, {lRF, 0.5 * lPF}, {lRF, 0.5 * lPF}
};
#endif

float RollPitchDeviation[2];
int8_t ramp = 1;
inline int8_t adaptiveCoefficient(byte idx, byte para) {
  return EEPROM.read(ADAPT_PARAM + idx * NUM_ADAPT_PARAM + para);
}

float adjust(byte i) {
  float rollAdj, pitchAdj, adj;
  if (i == 1 || i > 3)  {//check idx = 1
    bool leftQ = (i - 1 ) % 4 > 1 ? true : false;
    //bool frontQ = i % 4 < 2 ? true : false;
    //bool upperQ = i / 4 < 3 ? true : false;
    float leftRightFactor = 1;
    if ((leftQ && ramp * RollPitchDeviation[0] > 0 )// operator * is higher than &&
        || ( !leftQ && ramp * RollPitchDeviation[0] < 0))
      leftRightFactor = LEFT_RIGHT_FACTOR * abs(ramp);
    rollAdj = fabs(RollPitchDeviation[0]) * adaptiveCoefficient(i, 0) * leftRightFactor;

  }
  else
    rollAdj = RollPitchDeviation[0] * adaptiveCoefficient(i, 0) ;
  currentAdjust[i] = radPerDeg * (
#ifdef POSTURE_WALKING_FACTOR
                       (i > 3 ? postureOrWalkingFactor : 1) *
#endif
                       rollAdj - ramp * adaptiveCoefficient(i, 1) * ((i % 4 < 2) ? ( RollPitchDeviation[1]) : RollPitchDeviation[1]));
  return currentAdjust[i];
}

void saveCalib(int8_t *var) {
  for (byte i = 0; i < DOF; i++) {
#ifdef ESP8266    
    EEPROM.write(CALIB + i, var[i]);
#else    
    EEPROM.update(CALIB + i, var[i]);
#endif    
    calibratedDuty0[i] = SERVOMIN + PWM_RANGE / 2 + float(middleShifts[i] + var[i]) * pulsePerDegree[i] * rotationDirections[i];
  }
}

void saveMPUcalib(int * var) {
  for (byte i = 0; i < 6; i++)
#ifdef ESP8266
    EEPROM.write(MPUCALIB + i, var[i]);
#else    
    EEPROM.update(MPUCALIB + i, var[i]);
#endif    
}


void writeConsts() {
  PTLF("Reset Const values?(Y/n)");
  while (!Serial.available()); // empty buffer
  char resetConsts = Serial.read();
  if (resetConsts == 'Y' || resetConsts == 'y') {
    for (byte i = 0; i < DOF; i++) {
#ifdef ESP8266    
      EEPROM.write(CALIB + i, 0);
 #else    
      EEPROM.update(CALIB + i, 0);
#endif      
    }
  }
}

void calibratedPWM(byte i, float angle, float speedRatio = 0) {
  /*float angle = max(-SERVO_ANG_RANGE/2, min(SERVO_ANG_RANGE/2, angle));
    if (i > 3 && i < 8)
    angle = max(-5, angle);*/
  int duty0 = calibratedDuty0[i] + currentAng[i] * pulsePerDegree[i] * rotationDirections[i];
  currentAng[i] = angle;
  int duty = calibratedDuty0[i] + angle * pulsePerDegree[i] * rotationDirections[i];
  duty = max(SERVOMIN , min(SERVOMAX , duty));
  byte steps = byte(round(abs(duty - duty0) / 1.0/*degreeStep*/ / speedRatio)); //default speed is 1 degree per step
  for (byte s = 0; s <= steps; s++) {
    pwm.setPWM(pins[i], 0, duty + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * (duty0 - duty)));
  }
}

void allCalibratedPWM(char * dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}

void shutServos() {
  delay(100);
  for (int8_t i = DOF - 1; i >= 0; i--) {
    pwm.setPWM(i, 0, 4096);
  }
}
int8_t countDown = 0;
template <typename T> void transform( T * target, byte angleDataRatio = 1, float speedRatio = 1, byte offset = 0) {
  countDown = 5;
  if (speedRatio == 0)
    allCalibratedPWM(target, 8);
  else {
    int *diff = new int [DOF - offset], maxDiff = 0;
    for (byte i = offset; i < DOF; i++) {
      diff[i - offset] =   currentAng[i] - target[i - offset] * angleDataRatio;
      maxDiff = max(maxDiff, abs( diff[i - offset]));
    }

    byte steps = byte(round(maxDiff / 1.0/*degreeStep*/ / speedRatio));//default speed is 1 degree per step

    for (byte s = 0; s <= steps; s++) {
      for (byte i = offset; i < DOF; i++) {
        float dutyAng = (target[i - offset] * angleDataRatio + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
        calibratedPWM(i,  dutyAng);
        //delayMicroseconds(2);
      }
    }
    delete [] diff;
    //  printList(currentAng);
    //PTL();
  }
}


void skillByName(char* skillName, byte angleDataRatio = 1, float speedRatio = 1, bool shutServoAfterward = true) {
  motion.loadBySkillName(skillName);
  transform(motion.dutyAngles, angleDataRatio, speedRatio);
  if (shutServoAfterward) {
    shutServos();
    token = 'd';
  }
}


//short tools

template <typename T> void allCalibratedPWM(T * dutyAng, byte offset = 0) {
  for (int8_t i = DOF - 1; i >= offset; i--) {
    calibratedPWM(i, dutyAng[i]);
  }
}

template <typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

void printRange(int r0 = 0, int r1 = 0) {
  if (r1 == 0)
    for (byte i = 0; i < r0; i++) {
      PT(i);
      PT('\t');
    }
  else
    for (byte i = r0; i < r1; i++) {
      PT(i);
      PT('\t');
    }
  PTL();
}
template <typename T> void printList(T * arr, byte len = DOF) {
  String temp = "";
  for (byte i = 0; i < len; i++) {
    temp += String(int(arr[i]));
    temp += ",\t";
    //PT((T)(arr[i]));
    //PT('\t');
  }
  PTL(temp);
}
template <typename T> void printEEPROMList(int EEaddress, byte len = DOF) {
  for (byte i = 0; i < len; i++) {
    PT((T)(EEPROM.read(EEaddress + i)));
    PT('\t');
  }
  PTL();
}
char getUserInput() {//limited to one character
  while (!Serial.available());
  return Serial.read();
}
