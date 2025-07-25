#include <Arduino.h>
#include <SpeedyStepper.h>
#include <FlexyStepper.h>

#include "enum.h"
#include "defines.h"
#include "static.h"
#include "stepperfunk.h"

DisksCheck checkerD = Step1;
Sharpering checker = Step_1;

Sharpering_t checker_t = Step_t_1;

InitialZero initialState = initialDown;
CheckDiamentr diamChecker = StartDown;
Alarm alarmProcessor = stop;
CheckProfile stateProfile = startCheck;

SpeedyStepper stepperX, stepperY, stepperD, stepperCl;
FlexyStepper stepperC;

int8_t checkAllDisks()
{
  return checkerD == DiskDone;
}

void alarmProcess()
{
  switch (alarmProcessor)
  {
  case stop:
    if (!stepperX.motionComplete() || !stepperY.motionComplete() || !stepperD.motionComplete() || !stepperC.motionComplete() || stepperCl.motionComplete())
    {
      if (!stepperX.motionComplete())
        stepperX.setupStop();
      if (!stepperY.motionComplete())
        stepperY.setupStop();
      if (!stepperD.motionComplete())
        stepperD.setupStop();
      if (!stepperC.motionComplete())
        stepperC.setTargetPositionToStop();
      if (!stepperCl.motionComplete())
        stepperCl.setupStop();
      alarmProcessor = processStop;
      isInit = false;
      isSharpering = false;
      isGrinding = false;
      isChoosing = false;
      isX = false;
      isY = false;
      isCl = false;
      isD = false;
    }
    else
      alarmProcessor = alarmStop;
    break;
  case processStop:
    if (stepperX.processMovement() && stepperY.processMovement() && stepperD.processMovement() && stepperC.processMovement() && stepperCl.processMovement())
      alarmProcessor = alarmStop;
    break;
  case alarmStop:
    isAlarm = false;
    isTurnOn = false;
    alarmProcessor = stop;
    delay(2000);
    break;
  default:
    break;
  }
}

uint8_t initializeGrind()
{
  return 0;
}

uint8_t moveX(float x)
{

  if (!stepperX.motionComplete())
  {
    stepperX.processMovement();
  }
  else if (stepperX.getCurrentPositionInMillimeters() != x)
    stepperX.setupMoveInMillimeters(x);
  if (stepperX.motionComplete())
    updateLazer();
  return stepperX.motionComplete();
}

uint8_t moveY(float y)
{
  if (!stepperY.motionComplete())
    stepperY.processMovement();
  else if (stepperY.getCurrentPositionInMillimeters() != y)
    stepperY.setupMoveInMillimeters(y);
  return stepperY.motionComplete();
}

uint8_t moveC(float z)
{
  if (!stepperC.motionComplete())
    stepperC.processMovement();
  else if (stepperC.getCurrentPositionInMillimeters() != z)
    stepperC.setTargetPositionInMillimeters(z);
  return stepperC.motionComplete();
}

uint8_t moveCl(float z)
{
  if (!stepperCl.motionComplete())
    stepperCl.processMovement();
  else if (stepperCl.getCurrentPositionInMillimeters() != z)
    stepperCl.setupMoveInMillimeters(z);
  return stepperCl.motionComplete();
}

uint8_t choseProf(uint32_t num)
{
  return moveCl(21 * (num - 1));
}

uint8_t choseD(uint32_t num)
{
  return moveD(disks[num - 1]);
}

uint8_t movingX()
{
  int8_t pos = -1;
  if (xNum == 2)
    pos = 1;
  return moveX(stepperX.getCurrentPositionInMillimeters() - 0.2 * pos);
}

uint8_t movingY()
{
  int8_t pos = -1;
  if (yNum == 2)
    pos = 1;
  return moveY(stepperY.getCurrentPositionInMillimeters() + 0.2 * pos);
}

uint8_t clamping()
{
  if (clNum == 2)
  {
    digitalWrite(skistop, LOW);
    digitalWrite(skiclamp, LOW);
    return true;
  }
  else
  {
    if (digitalRead(skiclamp) == LOW)
    {
      digitalWrite(skiclamp, HIGH);
      timeClamp = millis();
    }
    if (abs(timeClamp - millis()) > 1000)
    {
      digitalWrite(skistop, HIGH);
      return true;
    }
    return false;
  };
}

int8_t startSharpening()
{

  switch (checker)
  {
  case Step_1:
    digitalWrite(penddown, LOW); // поднимаем диск
    // digitalWrite(pendout, LOW); // зажим диска
    checker = Step_2;
    break;
  case Step_2:
    if (isDown || true)
    {
      checker = Step_3;
      stepperX.setSpeedInStepsPerSecond(speedX);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
    }
    break;
  case Step_3:
    moveX(-400);
    if (x_zero)
    {
      checker = Step_4;
      stepperX.setCurrentPositionInMillimeters(0);
      stepperX.setupMoveInMillimeters(0);
    }
    break;
  case Step_4:
    if (moveX(1))
      checker = Step_6;
    break;
  case Step_5:
    digitalWrite(penddown, HIGH);
    moveY(-140);
    if (y_zero)
    {
      checker = Step_6;
      stepperY.setCurrentPositionInMillimeters(0);
      stepperY.setupMoveInMillimeters(0);
    }
    break;
  case Step_6:
    // digitalWrite(penddown, HIGH); // опускаем диск
    if (moveY(140.1))
      checker = Step_7; // Y первого конька
    break;
  case Step_7:
    digitalWrite(penddown, LOW);
    // digitalWrite(pendout, HIGH);
    time_now = millis();
    checker = Step_8;
    break;
  case Step_8:
    digitalWrite(motdisk, HIGH);
    whichSharp = 1;
    time_now = millis();
    checker = Step_9;
    break;
  case Step_9:
    if (!isDown)
    {
      stepperX.setSpeedInStepsPerSecond(speedX);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      checker = Step_10;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_up)
        isDown = false;
    }
    break;
  case Step_10:
    if (moveX(325))
    {
      digitalWrite(penddown, HIGH);
      time_now = millis();
      checker = Step_11;
    }
    break;
  case Step_11:
    if (isDown)
    {
      stepperX.setSpeedInStepsPerSecond(speedX * 2);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX * 2);
      checker = Step_12;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_down)
      {
        isDown = true;
      }
    }
    break;
  case Step_12:
    if (moveX(1))
    {
      if (whichSharp >= count_of_Sharps)
      {
        time_now = millis();
        checker = Step_15;
      }
      else
      {
        whichSharp++;
        digitalWrite(penddown, LOW);
        time_now = millis();
        checker = Step_9;
      }
    }
    break;
  case Step_13:

    break;
  case Step_14:
    checker = Step_15;
    break;
  case Step_15:
    if (isDown)
    {
      checker = Step_16;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_down)
      {
        isDown = true;
      }
    }
    break;
  case Step_16:
    if (moveY(-8.61))
      checker = Step_17; // Y второго конька
    break;
  case Step_17:
    digitalWrite(penddown, LOW);
    time_now = millis();
    whichSharp = 1;
    checker = Step_19;
    break;
  case Step_18:

    break;
  case Step_19:
    if (!isDown)
    {
      stepperX.setSpeedInStepsPerSecond(speedX);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      checker = Step_20;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_up)
        isDown = false;
    }
    break;
  case Step_20:
    if (moveX(325))
    {
      digitalWrite(penddown, HIGH);
      time_now = millis();
      checker = Step_21;
    }
    break;
  case Step_21:
    if (isDown)
    {
      stepperX.setSpeedInStepsPerSecond(speedX * 2);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX * 2);
      checker = Step_22;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_down)
        isDown = true;
    }
    break;
  case Step_22:
    if (moveX(1))
    {
      if (whichSharp >= count_of_Sharps)
        checker = Step_23;
      else
      {
        whichSharp++;
        digitalWrite(penddown, LOW);
        time_now = millis();
        checker = Step_19;
      }
    }
    break;
  case Step_23:
    digitalWrite(motdisk, LOW);
    checker = Step_24;
    break;
  case Step_24:
    digitalWrite(penddown, HIGH);
    time_now = millis();
    checker = Step_25;
    break;
  case Step_25:
    if (isDown)
    {
      stepperX.setSpeedInStepsPerSecond(speedX * 2);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX * 2);
      checker = Step_26;
    }
    else
    {
      if (abs(millis() - time_now) > time_to_down)
        isDown = true;
    }
    break;
  case Step_26:
    if (moveX(1))
      checker = Step_27;
    break;
  case Step_27:
    if (moveY(140.1))
      checker = Step_28;
    break;
  case Step_28:
    digitalWrite(penddown, LOW);
    checker = Step_29;
    break;
  case Step_29:
    digitalWrite(skistop, LOW);
    digitalWrite(skiclamp, LOW);
    checker = Done;
    break;
  default:
    break;
  }

  return checker == Done;
}

void initializeZeros()
{

  switch (initialState)
  {
  case initialDown:
    digitalWrite(penddown, HIGH);
    // digitalWrite(pendout, LOW);
    initialState = initialC;
    break;
  case initialC:
    moveC(80);
    if (c_zero)
    {
      stepperC.setCurrentPositionInMillimeters(0);
      stepperC.setTargetPositionInMillimeters(0);
      initialState = afterC;
    }
    break;
  case afterC:
    if (moveC(-1))
      initialState = initialCl;
    break;
  case initialCl:
    moveCl(90);
    if (cl_zero || true)
    {
      Serial.println(stepperCl.getCurrentPositionInMillimeters());
      stepperCl.setCurrentPositionInMillimeters(87);
      stepperCl.setupMoveInMillimeters(87);
      Serial.println(stepperCl.getCurrentPositionInMillimeters());
      initialState = afterCl;
    }
    break;
  case afterCl:
    if (moveCl(84) || true)
    {
      stepperX.setSpeedInStepsPerSecond(speedX);
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      initialState = initialX;
    }
    break;
  case initialX:
    moveX(-600);
    if (x_zero)
    {
      Serial.println(stepperX.getCurrentPositionInMillimeters());
      initialState = afterX;
      stepperX.setCurrentPositionInMillimeters(0);
      stepperX.setupMoveInMillimeters(0);
      Serial.println(stepperX.getCurrentPositionInMillimeters());
    }
    break;
  case afterX:
    if (moveX(1))
      initialState = initialY;
    break;
  case initialY:
    moveY(-180);
    if (y_zero)
    {
      Serial.println(stepperY.getCurrentPositionInMillimeters());
      initialState = afterY;
      stepperY.setCurrentPositionInMillimeters(0);
      stepperY.setupMoveInMillimeters(0);
      Serial.println(stepperY.getCurrentPositionInMillimeters());
    }
    break;
  case afterY:
    if (moveY(140.1))
      initialState = initialD; // обнуление Y (-5, 16.8)
    break;
  case initialD:
    moveD(2);
    if (d_zero || true)
    {
      Serial.println(stepperD.getCurrentPositionInRevolutions());
      stepperD.setCurrentPositionInRevolutions(0);
      stepperD.setupMoveInRevolutions(0);
      Serial.println(stepperD.getCurrentPositionInRevolutions());
      initialState = afterD;
    }
    break;
  case afterD:
    if (moveD(disks[0]) || true)
      initialState = afterDown;
    break;
  case afterDown:
    digitalWrite(penddown, LOW);
    // digitalWrite(pendout, HIGH); // разжим диска
    time_now = millis();
    initialState = initialBoth;
    break;
  default:
    break;
  }
}

void handler()
{
  if (d_zero != digitalRead(zeroD))
  {
    d_zero = digitalRead(zeroD);
    Serial.println("d_zero" + String(digitalRead(zeroD)));
  }

  if (x_zero != digitalRead(x_pin))
  {
    x_zero = digitalRead(x_pin);
    Serial.println("x_zero");
  }

  if (y_zero != digitalRead(y_pin))
  {
    y_zero = digitalRead(y_pin);
    Serial.println("y_zero");
  }

  if (c_zero != digitalRead(conterweihgt_pin))
  {
    c_zero = digitalRead(conterweihgt_pin);
  }

  if (cl_zero != !digitalRead(zeroCl))
  {
    cl_zero = !digitalRead(zeroCl);
  }

  if (isTouched != (digitalRead(diametr)))
  {
    isTouched = (digitalRead(diametr));
  }

  if (!isAlarm && (!digitalRead(alarmButton) || isAlarmButton))
  {
    isAlarm = true;
    Serial.println("Alarm!");
  }

  if (!isTurnOn && isTurnOnButton)
  {
    isTurnOn = true;
    Serial.println("Start");
  }

  if (!isProf && isProfButton)
  {
    isProf = true;
    Serial.println("Start profile");
  }

  if (!isSharpering && (isSharpButton))
  {
    isSharpering = true;
    Serial.println("Sharp");
  }

  if (!isGrinding && (isGrindButton))
  {
    isGrinding = true;
    Serial.println("Disk Sharp");
  }

  if (!isChoosing && (isChoosingButton))
  {
    isChoosing = true;
    profNum = profNumTemp;
    Serial.println("Choose");
  }

  if (!isD && (isDButton))
  {
    isD = true;
    dNum = dNumTemp;
    Serial.println("Disk");
  }

  if (!isX && (isXButton))
  {
    isX = true;
    xNum = xNumTemp;
    // Serial.println("X");
  }

  if (!isY && (isYButton))
  {
    isY = true;
    yNum = yNumTemp;
    // Serial.println("Y");
  }

  if (!isCl && (isClButton))
  {
    isCl = true;
    clNum = clNumTemp;
    // Serial.println("Cl");
  }

  if (!isInit && (isZeroButton))
  {
    isInit = true;
    Serial.println("Init");
  }

  if (alarm_pos)
  {
    // Serial.println("alarm_pos");
  }

  if (conterweihgt_zero)
  {
    // Serial.println("conterweihgt_zero");
  }
}

void recieveData()
{
  if (Serial.available())
  {
    char whatNow[10];
    Serial.readBytesUntil('\n', whatNow, 10);
    switch (whatNow[0])
    {
    case 't':
      isTurnOnButton = whatNow[1] == '1';
      break;
    case 'a':
      isAlarmButton = whatNow[1] == '1';
      break;
    case 'z':
      isZeroButton = whatNow[1] == '1';
      break;
    case 's':
      isSharpButton = whatNow[1] == '1';
      break;
    case 'n':
      isGrindButton = whatNow[1] == '1';
      break;
    case 'p':
      isChoosingButton = whatNow[1] != '0';
      profNumTemp = whatNow[1] - 48;
      break;
    case 'x':
      isXButton = whatNow[1] != '0';
      xNumTemp = whatNow[1] - 48;
      break;
    case 'y':
      isYButton = whatNow[1] != '0';
      yNumTemp = whatNow[1] - 48;
      break;
    case 'c':
      isClButton = whatNow[1] != '0';
      clNumTemp = whatNow[1] - 48;
      break;
    case 'd':
      isDButton = whatNow[1] != '0';
      dNumTemp = whatNow[1] - 48;
      break;
    case 'l':
      voltage = Serial.readStringUntil('\n').toFloat();
      break;
    case 'q':
    {
      profileArrN = Serial.readStringUntil('\n').toInt();
      for (int i = 0; i < profileArrN; i++)
      {
        profileArrX[i] = Serial.readStringUntil('\n').toFloat();
        profileArrZ[i] = Serial.readStringUntil('\n').toFloat();
      }
    }
      break;
    case 'r':
      isProfButton = whatNow[1] == '1';
      break;
    }
  }
}

int8_t moveD(float rev)
{
  if (!stepperD.motionComplete())
    stepperD.processMovement();
  else if (stepperD.getCurrentPositionInRevolutions() != rev)
    stepperD.setupMoveInRevolutions(rev);
  return stepperD.motionComplete();
}

int8_t startSharpening_t()
{
  uint64_t xspeed = 24;
  uint64_t x_bias = 37;
  uint64_t z_bias = 30;
  float spZ = 0;
  float minZ = 100000;
  switch (checker_t)
  {
  case Step_t_1:
    digitalWrite(penddown, LOW); // поднимаем диск
    if (moveY(140.1))
      checker_t = Step_t_2;
    break;
  case Step_t_2:
    if (isDown || true)
    {
      for (int i = 0; i < profileArrN; i++)
      {
        if (profileArrZ[i] < minZ)
          minZ = profileArrZ[i];
      }
      checker_t = Step_t_3;
      stepperC.setSpeedInStepsPerSecond(speedZ);
      stepperC.setAccelerationInStepsPerSecondPerSecond(accZ);

      stepperX.setSpeedInMillimetersPerSecond(xspeed);
      stepperX.setAccelerationInMillimetersPerSecondPerSecond(xspeed);
    }
    break;
  case Step_t_3:
    if (moveC(-1))
    {
      checker_t = Step_t_4;
    }
    break;
  case Step_t_4:
    if (moveX(1))
    {
      checker_t = Step_t_5;
    }
    break;
  case Step_t_5:
    if (moveX(profileArrX[0] + x_bias))
    {
      checker_t = Step_t_6;
    }
    break;
  case Step_t_6:
    if (moveC(-(profileArrZ[0] + z_bias)))
    {
      spZ = abs(profileArrZ[profileInd] - profileArrZ[profileInd + 1]) / (abs(profileArrX[profileInd + 1] - profileArrX[profileInd]) / xspeed);
      stepperC.setSpeedInMillimetersPerSecond(spZ);
      stepperC.setAccelerationInMillimetersPerSecondPerSecond(spZ * 2);
      profileInd++;
      checker_t = Step_t_7;
    }
    break;
  case Step_t_7:
    moveX(profileArrX[3] + x_bias);
    if (moveC(-(minZ + z_bias)))
    {
      checker_t = Step_t_8;
    }else{
      spZ = abs(profileArrZ[profileInd] - profileArrZ[profileInd + 1]) / (abs(profileArrX[profileInd + 1] - profileArrX[profileInd]) / xspeed);
      stepperC.setSpeedInMillimetersPerSecond(spZ);
      stepperC.setAccelerationInMillimetersPerSecondPerSecond(spZ * 2);
      profileInd++;
    }
    break;
  case Step_t_8:
    moveX(profileArrX[3] + x_bias);
    if (moveC(-(profileArrZ[profileArrN - 1] + z_bias)))
    {
      stepperC.setSpeedInStepsPerSecond(speedZ);
      stepperC.setAccelerationInStepsPerSecondPerSecond(accZ);
      checker_t = Step_t_9;
    }else{
      spZ = abs(profileArrZ[profileInd] - profileArrZ[profileInd + 1]) / (abs(profileArrX[profileInd + 1] - profileArrX[profileInd]) / xspeed);
      stepperC.setSpeedInMillimetersPerSecond(spZ);
      stepperC.setAccelerationInMillimetersPerSecondPerSecond(spZ * 2);
      profileInd++;
    }
    break;
  case Step_t_9:
    if (moveC(-1) && moveX(profileArrX[3] + x_bias))
    {
      stepperX.setSpeedInMillimetersPerSecond(24);
      stepperX.setAccelerationInMillimetersPerSecondPerSecond(48);
      checker_t = Step_t_10;
    }
    break;
  case Step_t_10:
    if (moveX(1))
    {
      checker_t = Step_t_11;
    }
    break;
  default:
    break;
  }

  return checker_t == Step_t_11;
}

uint8_t checkProfileFun(float speed, float time)
{

  switch (stateProfile)
  {
  case startCheck:
    if (moveY(170.83))
    {
      stepperX.setSpeedInMillimetersPerSecond(speed);
      moveX(600);
      timeCheck = millis();
      stateProfile = findingStart;
    }
    break;
  case findingStart:
    if (abs(millis() - timeCheck) >= time)
    {
      updateLazer();
      if (leftVoltage <= 5)
      {
        profileArrX[0] = getXfromlazer(1);
        profileArrZ[0] = getYfromlazer(1);
        Serial.println("----------------------------");
        Serial.println(profileArrX[0]);
        Serial.println(profileArrZ[0]);
        stateProfile = findingLeftPlato;
      }
      timeCheck = millis();
    }
    moveX(600);
    break;
  case findingLeftPlato:
    if (abs(millis() - timeCheck) >= time)
    {
      updateLazer();
      if (abs(leftZ - rightZ) < 0.2)
      {
        profileArrX[1] = getXfromlazer(1);
        profileArrZ[1] = rightZ;
        Serial.println("----------------------------");

        Serial.println(profileArrX[1]);
        Serial.println(profileArrZ[1]);
        stateProfile = findingRightPlato;
      }
      timeCheck = millis();
    }
    moveX(600);
    break;
  case findingRightPlato:
    if (abs(millis() - timeCheck) >= time)
    {
      updateLazer();
      if (abs(leftZ - rightZ) > 0.25)
      {
        Serial.println("----------------------------");

        profileArrX[2] = getXfromlazer(1);
        profileArrZ[2] = leftZ;
        Serial.println(profileArrX[2]);
        Serial.println(profileArrZ[2]);
        stateProfile = findingFinish;
      }
      timeCheck = millis();
    }
    moveX(600);
    break;
  case findingFinish:
    moveX(600);
    if (abs(millis() - timeCheck) >= time)
    {
      updateLazer();
      if (rightVoltage >= 5)
      {
        profileArrX[3] = getXfromlazer(-1);
        profileArrZ[3] = getYfromlazer(1);
        Serial.println("----------------------------");
        Serial.println(profileArrX[3]);
        Serial.println(profileArrZ[3]);
        stateProfile = profileStop;
        stepperX.setupStop();
      }
      timeCheck = millis();
    }
    break;
  case profileStop:
    if (stepperX.processMovement())
    {
      stateProfile = movingZero;
      stepperX.setSpeedInMillimetersPerSecond(36);
    }
    break;
  case movingZero:
    if (moveX(1))
      stateProfile = endProfile;
  default:
    break;
  }

  return stateProfile == endProfile;
}

float getXfromlazer(int8_t sign)
{

  float r = 50;
  float hy = rightZ - leftZ;
  float hx = rightX - leftX;

  float cy = rightZ - leftZ;
  float cx = 0;

  float l = hy * cy / (hy * hy + hx * hx);

  float vx = (leftX + hx * l) - leftX;
  float vy = (leftZ + hy * l) - rightZ;

  float x = -sign * vx * r / sqrt(vx * vx + vy * vy) + rightX;
  float y = -sign * vy * r / sqrt(vx * vx + vy * vy) + rightZ;
  return x;
}

float getYfromlazer(int8_t sign)
{

  float r = 50;
  float hy = rightZ - leftZ;
  float hx = rightX - leftX;

  float cy = rightZ - leftZ;
  float cx = 0;

  float l = hy * cy / (hy * hy + hx * hx);

  float vx = (leftX + hx * l) - leftX;
  float vy = (leftZ + hy * l) - rightZ;

  float x = -sign * vx * r / sqrt(vx * vx + vy * vy) + rightX;
  float y = -sign * vy * r / sqrt(vx * vx + vy * vy) + rightZ;

  return y;
}

void updateLazer()
{
  leftX = rightX;
  leftZ = rightZ;
  leftVoltage = rightVoltage;
  rightVoltage = voltage;
  rightZ = (5.3 - voltage) * 6.1648049166;
  rightX = stepperX.getCurrentPositionInMillimeters();
}

void transmitData()
{
  Serial.print("x:");
  Serial.print(stepperX.getCurrentPositionInMillimeters());
  Serial.print(",z:");
  Serial.println(voltage * 6.1648049166);
}
