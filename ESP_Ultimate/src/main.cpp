#include <Arduino.h>
#include <SpeedyStepper.h>
#include <FlexyStepper.h>

#include "static.h"
#include "enum.h"
#include "stepperfunk.h"
#include "defines.h"

void setup()
{

  Serial.begin(115200);

  pinMode(conterweihgt_pin, INPUT);
  pinMode(potentiometer_pin, INPUT);
  pinMode(zeroD, INPUT_PULLUP);
  pinMode(zeroCl, INPUT_PULLUP);

  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);
  pinMode(diametr, INPUT_PULLUP);
  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);

  pinMode(skiclamp, OUTPUT);
  pinMode(skistop, OUTPUT);
  pinMode(penddown, OUTPUT);
  pinMode(pendout, OUTPUT);
  pinMode(motdisk, OUTPUT);

  pinMode(alarmButton, INPUT_PULLUP);
  // pinMode(turnButton, INPUT_PULLUP);

  pinMode(dimondStep, OUTPUT);
  pinMode(dimondDir, OUTPUT);

  pinMode(clampStep, OUTPUT);
  pinMode(clampDir, OUTPUT);

  //-----------------
  stepperX.connectToPins(stepPinX, dirPinX);
  stepperX.setStepsPerMillimeter(scale_mm_to_steps_x);
  stepperX.setSpeedInStepsPerSecond(speedX);
  stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
  // stepperX.setupMoveInMillimeters(-200); // + Влево
  //-----------------

  //-----------------
  stepperY.connectToPins(stepPinY, dirPinY);
  stepperY.setStepsPerMillimeter(scale_mm_to_steps_y);
  stepperY.setSpeedInStepsPerSecond(speedY);
  stepperY.setAccelerationInStepsPerSecondPerSecond(accY);
  // stepperY.setupMoveInMillimeters(-160); // + на нас
  //-----------------

  //-----------------
  stepperD.connectToPins(dimondStep, dimondDir);
  stepperD.setStepsPerRevolution(20000);
  stepperD.setSpeedInStepsPerSecond(4000);
  stepperD.setAccelerationInStepsPerSecondPerSecond(16000);
  // stepperD.setupMoveInRevolutions(1);
  //-----------------

  //-----------------
  stepperC.connectToPins(overStep, overDir);
  stepperC.setStepsPerMillimeter(scale_mm_to_steps_z);
  stepperC.setSpeedInStepsPerSecond(3200);
  stepperC.setAccelerationInStepsPerSecondPerSecond(6400);
  // stepperC.setupMoveInMillimeters(-80); // - поднятие диска
  //-----------------

  //-----------------
  stepperCl.connectToPins(clampStep, clampDir);
  stepperCl.setStepsPerMillimeter(782.2277847);
  stepperCl.setSpeedInStepsPerSecond(8000);
  stepperCl.setAccelerationInStepsPerSecondPerSecond(16000);
  //  stepperCl.setupMoveInMillimeters(-50); // + увеличивает размер конька
  //-----------------

  time_now = millis();
  timeTransm = millis();
  timeClamp = millis();

  digitalWrite(motdisk, HIGH);
}


void loop()
{


  recieveData();

  if (!isAlarm && isTurnOn)
  {

    if (isProf)
    {
      isProf = !checkProfileFun(6, 600);
      isChecked |= !isProf;
    }
    else
      stateProfile = startCheck;

    if (isInit)
    {
      if (initialState != initialBoth)
        initializeZeros();
      else
        isInit = false;
    }
    else
      initialState = initialDown;

    if (isSharpering && isChecked)
      isSharpering = !startSharpening_t();
    else
      checker_t = Step_t_1;

    if (isGrinding)
      isGrinding = !initializeGrind();
    else
      diamChecker = StartDown;

    if (isChoosing)
    {
      isChoosing = !choseProf(profNum);
    }

    if (isX)
    {
      isX = !movingX();
    }

    if (isY)
    {
      isY = !movingY();
    }

    if (isCl)
    {
      isCl = !clamping();
    }

    if (isD)
    {
      isD = !choseD(dNum);
    }
    // else isTurnOn = false;
  }
  else if (isAlarm)
  {
    alarmProcess();
  }

  if (abs(millis() - time_now) > 5000)
  {
    if (digitalRead(pendout))
      digitalWrite(pendout, LOW);
    time_now = millis();
  }

  handler();

  if (abs(timeTransm - millis()) > 300)
  {
    timeTransm = millis();
    transmitData();
  }
}

