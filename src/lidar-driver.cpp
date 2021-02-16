#include <Arduino.h>
#include <AccelStepper.h> // TODO: remove dependency as it's GPL licensed
#include <MultiStepper.h> // TODO: remove dependency as it's GPL licensed
#include <LIDARLite.h>

#define BAUD_RATE 115200

#define PIN_MC_TRIG 11
#define PIN_MC_MON 10

#define PIN_HALLY A6
#define PIN_HALLX A7

#define PIN_STEP1 9
#define PIN_DIR1 8
#define PIN_STEP2 3
#define PIN_DIR2 2

#define PIN_MS1 6
#define PIN_MS2 5
#define PIN_MS3 4

#define PIN_ENABLE 7

#define PIN_LED_GREEN A1
#define PIN_LED_ORANGE A2
#define PIN_LED_RED A3

#define MAX_STRING_LENGTH 10

#define INSTRUCTION_MOVE1 'x'
#define INSTRUCTION_MOVE2 'y'
#define INSTRUCTION_SPEED1 'm'
#define INSTRUCTION_SPEED2 'n'
#define INSTRUCTION_STOP 's'
#define INSTRUCTION_LIDAR 'l'
#define INSTRUCTION_DISABLE 'd'
#define INSTRUCTION_ENABLE 'e'
#define INSTRUCTION_PRINT 'p'
#define INSTRUCTION_ACC 'a'
#define INSTRUCTION_HOME 'h'
#define INSTRUCTION_DEBUG '?'
#define INSTRUCTION_RUN 'r'
#define INSTRUCTION_BENCHMARK 'b'

#define MODE_IDLE 0
#define MODE_SPEED 2
#define MODE_MOVE 3
#define MODE_HOMING 4   // Going back to home position
#define MODE_SCANNING 5 // scanning

#define XSTEPS 1600
#define YSTEPS 1200

int inByte = 0;

AccelStepper stepperx = AccelStepper(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper steppery = AccelStepper(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);

LIDARLite myLidarLite;

char stringText[MAX_STRING_LENGTH + 1];

bool hx, hy;

int mode = MODE_IDLE;
int homingPhase = 0;
int scanningPhase = 0;

int scanUntilRow = 0; // last row to scan

void setup()
{

  // LIDAR init
  myLidarLite.begin(0, true);
  // pinMode(PIN_MC_TRIG, OUTPUT);
  // digitalWrite(PIN_MC_TRIG, HIGH);
  // pinMode(PIN_MC_MON, INPUT);

  // LEDs init
  pinMode(PIN_LED_GREEN, OUTPUT);
  digitalWrite(PIN_LED_GREEN, HIGH);
  // pinMode(PIN_LED_ORANGE, OUTPUT);
  // digitalWrite(PIN_LED_ORANGE, HIGH);

  pinMode(PIN_HALLX, INPUT);
  pinMode(PIN_HALLY, INPUT);

  pinMode(PIN_MS1, OUTPUT);
  digitalWrite(PIN_MS1, HIGH);
  pinMode(PIN_MS2, OUTPUT);
  digitalWrite(PIN_MS2, LOW);
  pinMode(PIN_MS3, OUTPUT);
  digitalWrite(PIN_MS3, LOW);

  pinMode(PIN_STEP1, OUTPUT);

  // pinMode(PIN_DIR1, OUTPUT);
  // digitalWrite(PIN_DIR1, HIGH);
  steppery.setPinsInverted(true, false, false);

  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, HIGH); // disabled by default

  pinMode(LED_BUILTIN, OUTPUT);

  stepperx.setMaxSpeed(500);
  stepperx.setAcceleration(100);
  steppery.setMaxSpeed(50);
  steppery.setAcceleration(100);

  Serial.begin(BAUD_RATE);

  while (!Serial)
  {
    delay(10); // wait for serial port to connect. Needed for native USB port only
  }
}

void disableMotors()
{
  digitalWrite(PIN_ENABLE, HIGH);
}

void enableMotors()
{
  digitalWrite(PIN_ENABLE, LOW);
}

bool readHall(int hallPin)
{
  return analogRead(hallPin) < 100 ? true : false;
}

void readHall()
{
  hx = readHall(PIN_HALLX);
  hy = readHall(PIN_HALLY);
}

void fullstop()
{
  stepperx.stop();
  steppery.stop();
  // stepperx.setSpeed(0);
  // steppery.setSpeed(0);
  mode = MODE_IDLE;
  scanningPhase = 0;
  homingPhase = 0;
}

bool accelerate()
{
  stepperx.setSpeed(300);
  return true;

  // if (stepperx.currentPosition() == XSTEPS)
  // {
  //   Serial.println("dacceleration ready");
  //   return true;
  // }
  // // TODO: more gradular change (integrate into accelstepper?)
  // if (stepperx.currentPosition() < XSTEPS * 0.25)
  // {
  //   stepperx.setSpeed(50);
  // }
  // else if (stepperx.currentPosition() < XSTEPS * 0.5)
  // {
  //   stepperx.setSpeed(150);
  // }
  // else if (stepperx.currentPosition() < XSTEPS * 00.75)
  // {
  //   stepperx.setSpeed(300);
  // }
  // stepperx.runSpeed();
  // return false;
}

// TODO
bool decelerate()
{
  return true;
}

void serialFlush(void)
{
  while (Serial.available() > 0)
  {
    Serial.read();
  }
}

unsigned long measure()
{
  // unsigned long pulseWidth = pulseIn(PIN_MC_MON, HIGH);
  // pulseWidth = pulseWidth / 10; // 10us = 1cm

  return myLidarLite.distance();
  // return pulseWidth;
}

void printPoint()
{
  Serial.print("p");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(stepperx.currentPosition() % XSTEPS);
  Serial.print(",");
  Serial.print(steppery.currentPosition());
  Serial.print(",");
  Serial.println(measure());
}

void benchmark()
{
  int measurements = 1000;
  long startTime = millis();
  int zeros = 0;
  float avg = -1, dist;
  bool first = true;
  for (int i = 0; i < measurements; i++)
  {
    dist = (float)myLidarLite.distance();
    if (dist <= 0)
    {
      zeros++;
      continue;
    }
    if (first)
    {
      first = false;
      avg = dist;
    }
    else
    {
      avg = avg * (float)(i - 1) / float(i) + dist / (float)i;
    }
  }
  long time = millis() - startTime;
  float freq = (float)measurements / (float)time * 1000;
  Serial.print("benchmark complete. took ");
  Serial.print(measurements);
  Serial.print(" measurements in ");
  Serial.print(time);
  Serial.print("ms (");
  Serial.print(freq);
  Serial.println("Hz)");
  Serial.print("Average measurement: ");
  Serial.print(avg);
  Serial.print(", invalid count: ");
  Serial.println(zeros);
}

bool scan()
{
  bool stepped;
  steppery.run();
  stepped = stepperx.runSpeed();
  if (stepped)
  {
    printPoint();
  }

  if (stepped && stepperx.currentPosition() % XSTEPS == 0)
  {
    Serial.print("drow complete (");
    Serial.print(steppery.currentPosition());
    Serial.println(")");
    if (steppery.currentPosition() == scanUntilRow - 1)
    {
      Serial.println("dscan complete");
      return true;
    }
    steppery.move(1);
    // TODO: maybe block here until it does the step...
  }
  return false;
}

void scanning()
{
  // PH0: initiate
  // PH1: accelerating
  // PH2: actual scan
  // PH3: decelerate
  switch (scanningPhase)
  {
  case 0:
    Serial.println("p# version 1");
    Serial.print("p# x-resolution ");
    Serial.println(XSTEPS);
    Serial.print("p# y-resolution ");
    Serial.println(YSTEPS);
    Serial.print("p# num-rows ");
    Serial.println(scanUntilRow - steppery.currentPosition());

    homingPhase = 0;
    scanningPhase = 1;
    break;
  case 1:
    if (accelerate())
    {
      scanningPhase = 2;
    }
    break;
  case 2:
    if (scan())
    {
      scanningPhase = 3;
    }
    break;
  case 3:
    if (decelerate())
    {
      homingPhase = 0;
      scanningPhase = 0;
      mode = MODE_IDLE;
    }
    break;
  }
}

bool homing()
{
  // PH0: initiate move back a little
  // PH1: wait until moved, then set speed
  // PH2: wait until centered, then apply offset
  // PH3: wait until offset complete
  switch (homingPhase)
  {
  case 0:
    stepperx.move(-100);
    steppery.move(-50);
    homingPhase = 1;
    break;
  case 1:
    stepperx.run();
    steppery.run();
    if (stepperx.distanceToGo() == 0 && steppery.distanceToGo() == 0)
    {
      stepperx.setSpeed(100);
      steppery.setSpeed(50);
      homingPhase = 2;
    }
    break;
  case 2:
    readHall();
    if (hx)
    {
      stepperx.setSpeed(0);
    }
    if (hy)
    {
      steppery.setSpeed(0);
    }
    if (hx && hy)
    {
      steppery.move(10 - (YSTEPS / 4)); // YSTEPS/4 would be 90 deg, 10 is the offset for the hall sensor
      homingPhase = 3;
    }
    else
    {
      stepperx.runSpeed();
      steppery.runSpeed();
    }
    break;
  case 3:
    steppery.run();
    if (steppery.distanceToGo() == 0)
    {
      stepperx.setCurrentPosition(0);
      steppery.setCurrentPosition(0);
      homingPhase = 0;
      Serial.println("dhoming ready");
      return true; // return from homing mode
    }
    break;
  }

  return false;
}

void serialData(void)
{
  unsigned long dist;
  char instruction = Serial.read();

  delay(2);                                      //wait to make sure all data in the serial message has arived
  memset(&stringText[0], 0, sizeof(stringText)); //clear the array
  while (Serial.available())
  {                                 //set elemetns of stringText to the serial values sent
    char digit = Serial.read();     //read in a char
    strncat(stringText, &digit, 1); //add digit to the end of the array
  }
  serialFlush();                                //Clear any excess data in the serial buffer
  int serialCommandValueInt = atoi(stringText); //converts stringText to an int
  // float serialCommandValueFloat = atof(stringText); //converts stringText to a float

  switch (instruction)
  {
  case INSTRUCTION_DEBUG:
    readHall();
    Serial.print("dmode: ");
    Serial.print(mode);
    Serial.print(", scanningPhase: ");
    Serial.print(homingPhase);
    Serial.print(", homingPhase: ");
    Serial.print(homingPhase);
    Serial.print(", currentPosX: ");
    Serial.print(stepperx.currentPosition());
    Serial.print(", currentPosY: ");
    Serial.print(steppery.currentPosition());
    Serial.print(", distanceToGoX: ");
    Serial.print(stepperx.distanceToGo());
    Serial.print(", distanceToGoY: ");
    Serial.print(steppery.distanceToGo());
    Serial.print(", hallX: ");
    Serial.print(hx);
    Serial.print(", hallY: ");
    Serial.println(hy);

    break;
  case INSTRUCTION_HOME:
    mode = MODE_HOMING;
    homingPhase = 0;
    break;
  case INSTRUCTION_RUN:
    scanUntilRow = serialCommandValueInt;
    if (scanUntilRow <= steppery.currentPosition())
    {
      Serial.println("etarget row is below (or at) current position");
      break;
    }
    if (scanUntilRow > YSTEPS / 4)
    {
      Serial.println("etarget row is over vertical");
      break;
    }
    mode = MODE_SCANNING;
    scanningPhase = 0;
    break;
  case INSTRUCTION_MOVE1:
    mode = MODE_MOVE;
    stepperx.move(serialCommandValueInt);
    break;
  case INSTRUCTION_MOVE2:
    mode = MODE_MOVE;
    steppery.move(serialCommandValueInt);
    break;
  case INSTRUCTION_SPEED1:
    mode = MODE_SPEED;
    stepperx.setSpeed(serialCommandValueInt);
    break;
  case INSTRUCTION_SPEED2:
    mode = MODE_SPEED;
    steppery.setSpeed(serialCommandValueInt);
    break;
  case INSTRUCTION_ACC:
    stepperx.setAcceleration(serialCommandValueInt);
    steppery.setAcceleration(serialCommandValueInt);
    break;
  case INSTRUCTION_STOP:
    fullstop();
    break;
  case INSTRUCTION_LIDAR:
    dist = measure();
    Serial.print("d");
    Serial.println(dist);
    break;
  case INSTRUCTION_DISABLE:
    disableMotors();
    break;
  case INSTRUCTION_ENABLE:
    enableMotors();
    break;
  case INSTRUCTION_PRINT:
    printPoint();
    break;
  case INSTRUCTION_BENCHMARK:
    benchmark();
    break;
  default:
    Serial.println("eunknown command");
    break;
  }
}

void loop()
{
  while (1)
  {
    if (Serial.available())
    {
      serialData();
    }

    switch (mode)
    {
    case MODE_HOMING:
      if (homing())
      {
        mode = MODE_IDLE;
      }
      break;
    case MODE_SCANNING:
      scanning();
      break;
    case MODE_SPEED:
      stepperx.runSpeed();
      steppery.runSpeed();
      break;
    case MODE_MOVE:
      stepperx.run();
      steppery.run();
      break;
    }
  }
}
