#include <Arduino.h>
#include <Wire.h>
#include <SpeedyStepper.h>
#include <FlexyStepper.h>

#define stepPinX 19
#define dirPinX 18
#define stepPinY 5
#define dirPinY 17

#define alarmButton 26
// #define turnButton 25
#define diametr 35
#define x_pin 36
#define y_pin 39

#define skiclamp 22
#define skistop 21
#define penddown 23
#define pendout 12
#define motdisk 25

#define conterweihgt_pin 27
#define potentiometer_pin 34
#define zeroD 32
#define zeroCl 0

#define overStep 16
#define overDir 4

#define dimondStep 2
#define dimondDir 15

#define clampStep 13
#define clampDir 14

float scale_mm_to_steps_x = 64.116f * 5.97f; // 1.26 за 1000 шагов 7.524
float scale_mm_to_steps_y = 52.9661f;        // 1.26 за 1000 шагов 7.524
float scale_mm_to_steps_z = 505.3316f;
// put function declarations here:
uint8_t moveX(float);
uint8_t moveY(float);
uint8_t moveC(float);
uint8_t moveCl(float);
int8_t moveD(float rev);

float getXfromlazer(int8_t);
float getYfromlazer(int8_t);
void updateLazer();

uint8_t choseProf(uint32_t);
uint8_t choseD(uint32_t);
uint8_t movingX();
uint8_t movingY();
uint8_t clamping();

uint8_t initializeGrind();

int8_t turnUp();
int8_t turnDown();

int8_t startSharpening();
int8_t startSharpening_t();

int8_t checkAllDisks();

void transmitData();
void recieveData();
void handler();
void initializeZeros();
void alarmProcess();

typedef enum
{
  Step_1 = 1,
  Step_2 = 2,
  Step_3 = 3,
  Step_4 = 4,
  Step_5 = 5,
  Step_6 = 6,
  Step_7 = 7,
  Step_8 = 8,
  Step_9 = 9,
  Step_10 = 10,
  Step_11 = 11,
  Step_12 = 12,
  Step_13 = 13,
  Step_14 = 14,
  Step_15 = 15,
  Step_16 = 16,
  Step_17 = 17,
  Step_18 = 18,
  Step_19 = 19,
  Step_20 = 20,
  Step_21 = 21,
  Step_22 = 22,
  Step_23 = 23,
  Step_24 = 24,
  Step_25 = 25,
  Step_26 = 26,
  Step_27 = 27,
  Step_28 = 28,
  Step_29 = 29,
  Step_30 = 30,
  Done

} Sharpering;

typedef enum
{
  initialDown,
  initialX,
  afterX,
  initialY,
  afterY,
  initialD,
  afterD,
  initialC,
  afterC,
  initialCl,
  afterCl,
  afterDown,
  initialBoth
} InitialZero;

typedef enum
{
  StartDown,
  Down,
  StartPos,
  StartCheck,
  StopCheck,
  Right,
  SlowCheck,
  SlowStop,
  ZeroX,
  ZeroY,
  FastGo,
  SlowGo,
  Grind,
  FinalZeroX,
  FinalZeroY,
  Grinded
} CheckDiamentr;

typedef enum
{
  stop,
  processStop,
  alarmStop
} Alarm;

typedef enum
{
  right,
  initialZero
} Comands;

typedef enum
{
  Step1 = 1,
  Step2 = 2,
  Step3 = 3,
  Step4 = 4,
  Step5 = 5,
  Step6 = 6,
  DiskDone

} DisksCheck;

typedef enum
{
  Step_t_1 = 1,
  Step_t_2 = 2,
  Step_t_3 = 3,
  Step_t_4 = 4,
  Step_t_5 = 5,
  Step_t_6 = 6,
  Step_t_7 = 7,
  Step_t_8 = 8,
  Step_t_9 = 9,
  Step_t_10 = 10,
  Step_t_11,
  Step_t_12

} Sharpering_t;

DisksCheck checkerD = Step1;
Sharpering checker = Step_1;

Sharpering_t checker_t = Step_t_1;

InitialZero initialState = initialDown;
CheckDiamentr diamChecker = StartDown;
Alarm alarmProcessor = stop;

uint8_t isTouched = false;
float diam = 0;

long long time_now = 0;

long long time_to_up = 2000;
long long time_to_down = 2000;

int64_t timeTransm = 0;
int64_t timeClamp = 0;
int64_t grind_time = 0;

uint8_t isSharpering = false;
uint8_t isGrinding = false;
uint8_t isInit = false;

uint8_t isAlarmButton = false;
uint8_t isTurnOnButton = false;
uint8_t isProfButton = false;
uint8_t isZeroButton = false;
uint8_t isSharpButton = false;
uint8_t isGrindButton = false;
uint8_t isChoosingButton = false;
uint8_t isXButton = false;
uint8_t isYButton = false;
uint8_t isClButton = false;
uint8_t isDButton = false;

uint32_t profNumTemp = 0;
uint32_t xNumTemp = 0;
uint32_t yNumTemp = 0;
uint32_t clNumTemp = 0;
uint32_t dNumTemp = 0;

uint8_t d_zero = false;
uint8_t x_zero = false;
uint8_t y_zero = false;
uint8_t c_zero = false;
uint8_t cl_zero = false;
uint8_t alarm_pos = false;

uint8_t isDown = true;

uint8_t isChoosing = false;
uint32_t profNum = false;

uint8_t isX = false;
uint32_t xNum = false;

uint8_t isY = false;
uint32_t yNum = false;

uint8_t isCl = false;
uint32_t clNum = false;

uint8_t isD = false;
uint32_t dNum = false;

uint8_t isAlarm = false;
uint8_t isTurnOn = false;
uint8_t isProf = false;

uint8_t conterweihgt_zero = false;

uint8_t zeroInit = false;

uint64_t speedX = 4000; // 12500 максимимальная скорость
uint64_t accX = speedX * 2;

uint64_t speedY = 1200; // 12500 максимимальная скорость
uint64_t accY = speedY * 2;

uint64_t speedZ = 12000; // 12500 максимимальная скорость
uint64_t accZ = speedZ * 2;

uint8_t isChecked = false;

uint32_t whichSharp = 0;
uint32_t count_of_Sharps = 3;

float rightZ = 0, leftZ = 0, rightX = 0, leftX = 0, leftVoltage = 10, rightVoltage = 10;
float voltage = 0; // 30 - voltage/2.72 * 30

float profileArrX[4];
float profileArrZ[4];
uint32_t profileInd = 0;
float step = 3;
long long timeCheck = 0;

typedef enum
{
  startCheck = 1,
  findingStart = 2,
  findingLeftPlato = 3,
  findingRightPlato = 4,
  findingFinish = 5,
  profileStop,
  movingZero,
  endProfile

} CheckProfile;

CheckProfile stateProfile = startCheck;

uint8_t checkProfileFun(float, float);

long t_changed = millis();

// uint8_t checkX(uint32_t);

SpeedyStepper stepperX, stepperY, stepperD, stepperC, stepperCl;

float disks[6] = {-0.25, -0.41666666, -0.5833333333, -0.75, -0.91666666666, -0.08333333};
// float disks_Y[6] = {6.26, 6.86, 6.86, 6.86, 6.36, 6.26}; // По порядку для каждого диска, выбор профиля будет автоматически опрделять Y. Абсолютная координата была 6.66, в массив записывать тоже абсолютные коориднаты
// float disks_Y[6] = {7.05, 7.45, 7.65, 7.75, 7.45, 7.05};
float disks_Y[6] = {11.2, 11.2, 11.2, 11.2, 11.2, 11.2};

void setup()
{

  Serial.begin(115200);

  pinMode(conterweihgt_pin, INPUT);
  pinMode(potentiometer_pin, INPUT);
  pinMode(zeroD, INPUT_PULLUP);
  pinMode(zeroCl, INPUT_PULLUP);

  pinMode(x_pin, INPUT);          // Устанавливаем пин как вход
  pinMode(y_pin, INPUT);          // Устанавливаем пин как вход
  pinMode(diametr, INPUT_PULLUP); // Устанавливаем пин как вход
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
      isProf = !checkProfileFun(6, 500);
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

int8_t checkAllDisks()
{

  /* switch ( checkerD)
   {
   case Step1:
     // delay(2000); // Поднять маятник: закрыть реле 1(нижний клапан) открыть реле 2(верхний клапан)
     if (moveD(disks[0])) {
        checkerD = Step2;
       delay(2000);
     }
     break;
   case Step2:
     if (moveD(disks[1])) {
        checkerD = Step3;
       delay(2000);
     }
     break;
   case Step3:
     if (moveD(disks[2])) {
        checkerD = Step4;
       delay(2000);
     }
     break;
   case Step4:
     if (moveD(disks[3])){
      checkerD = Step5;
     delay(2000);
     }
     break;
   case Step5:
     if (moveD(disks[4])) {
        checkerD = Step6;
     delay(2000);
     }
     break;
   case Step6:
     if (moveD(disks[5])) {
        checkerD = DiskDone;
       delay(2000);
     }
     break;

   default:
     break;
   }
 */
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
        stepperC.setupStop();
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

uint8_t initializeGrind() { return 0; }

/*
uint8_t initializeGrind()
{

  switch (diamChecker)
  {
  case StartDown:
    digitalWrite(penddown, HIGH);
    digitalWrite(pendout, LOW);
    diamChecker = Down;
    break;
  case Down:
    if (isDown) diamChecker = StartPos;
    break;
  case StartPos:
    if (moveY(-190)) {
      diamChecker = StartCheck;
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      stepperX.setSpeedInStepsPerSecond(speedX);
    }
    break;
  case StartCheck:
    moveX(-100);
     if (isTouched) {
      stepperX.setupStop();
      diam = stepperX.getCurrentPositionInMillimeters();
      Serial.println(diam);
      diamChecker = StopCheck;
    }
    break;
  case StopCheck:
    stepperX.processMovement();
    if (stepperX.motionComplete()) diamChecker = Right;
    break;
  case Right:
    if (moveX(-diam+5)) {
      diamChecker = SlowCheck;
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      stepperX.setSpeedInStepsPerSecond(speedX);
    }
    break;
  case SlowCheck:
    moveX(-diam - 10);
     if (isTouched) {
      stepperX.setupStop();
      diam = stepperX.getCurrentPositionInMillimeters();
      Serial.println(diam);
      diamChecker = SlowStop;
    }
    break;
  case SlowStop:
    stepperX.processMovement();
    if (stepperX.motionComplete()) {
      diamChecker = ZeroX;
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      stepperX.setSpeedInStepsPerSecond(speedX);
    }
    break;
  case ZeroX:
    if (moveX(2)) diamChecker = ZeroY;
    break;
  case ZeroY:
    if (moveY(disks_Y[dNum-1])) { // положение y заточка (-1, 8.15, 6.95)
      digitalWrite(motdisk, LOW);
      diamChecker = FastGo;
    }
    break;
  case FastGo: // (x - 21.46) = y --- 81 + y //59.54
    if (moveX(-62.54 - diam)) { // добавил (3 мм )
      diamChecker = SlowGo;
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      stepperX.setSpeedInStepsPerSecond(speedX);
    }
    break;
  case SlowGo:
    if (moveX(62.54 + diam + 0.8)) { // добавил (3 мм)
      diamChecker = Grind;
      stepperX.setAccelerationInStepsPerSecondPerSecond(accX);
      stepperX.setSpeedInStepsPerSecond(speedX);
      grind_time = millis();
    }
    break;
  case Grind:
    if (abs(grind_time - millis()) > 1500){ // время точения (2000, 500 )
      digitalWrite(motdisk, HIGH);
      diamChecker = FinalZeroX;
    }
    break;
  case FinalZeroX:
    if (moveX(-5)) diamChecker = FinalZeroY;
    break;
  case FinalZeroY:
    if (moveY(-16.9)) {    // возврат по Y (-5)
      diamChecker = Grinded;
      digitalWrite(penddown, LOW);
      digitalWrite(pendout, LOW);
    }
    break;

  default:
    break;
  }

  return diamChecker == Grinded;
}
*/
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
    stepperC.setupMoveInMillimeters(z);
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
    // delay(2000); // Поднять маятник: закрыть реле 1(нижний клапан) открыть реле 2(верхний клапан)
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
      // Serial.println(stepperX.getCurrentPositionInMillimeters());
      checker = Step_4;
      stepperX.setCurrentPositionInMillimeters(0);
      stepperX.setupMoveInMillimeters(0);
      // Serial.println(stepperX.getCurrentPositionInMillimeters());
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
      // Serial.println(stepperY.getCurrentPositionInMillimeters());
      checker = Step_6;
      stepperY.setCurrentPositionInMillimeters(0);
      stepperY.setupMoveInMillimeters(0);
      // Serial.println(stepperY.getCurrentPositionInMillimeters());
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
    // if (moveC(5)|| true) checker = Step_14;
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
    // time_now = millis();
    // if (moveC(25)|| true) checker = Step_19; // было 75, 50, 40 , 30
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
      stepperC.setupMoveInMillimeters(0);
      initialState = afterC;
      // delay(5000);
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
    // Serial.println("c_zero" + String(digitalRead(conterweihgt_pin)));
  }

  if (cl_zero != !digitalRead(zeroCl))
  {
    cl_zero = !digitalRead(zeroCl);
    // Serial.println("cl_zero" + String(digitalRead(zeroCl)));
  }

  if (isTouched != (digitalRead(diametr)))
  {
    isTouched = (digitalRead(diametr));
    // Serial.println(digitalRead(diametr));
    // Serial.println("touched");
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

  // if (isDown && (analogRead(potentiometer_pin) < 3600)) {
  //       isDown = false;
  //       // Serial.println("p" + String(isDown));
  //   }

  //   if (!isDown && (analogRead(potentiometer_pin) > 4020)) {
  //       isDown = true;
  //       // Serial.println("p" + String(isDown));
  //   }

  // if (y_zero != digitalRead(y_pin)){
  //   y_zero = digitalRead(y_pin);
  //   Serial.println("y_zero");
  // }

  // // if (isTouched != (digitalRead(diametr))){
  // //   isTouched = (digitalRead(diametr));
  // //   Serial.println(digitalRead(diametr));
  // //   Serial.println("touched");
  // // }
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
      profileArrX[0] = Serial.readStringUntil('\n').toFloat();
      profileArrX[1] = Serial.readStringUntil('\n').toFloat();
      profileArrX[2] = Serial.readStringUntil('\n').toFloat();
      profileArrX[3] = Serial.readStringUntil('\n').toFloat();
      
      profileArrZ[0] = Serial.readStringUntil('\n').toFloat();
      profileArrZ[1] = Serial.readStringUntil('\n').toFloat();
      profileArrZ[2] = Serial.readStringUntil('\n').toFloat();
      profileArrZ[3] = Serial.readStringUntil('\n').toFloat();

      Serial.println("Profile X: " + String(profileArrX[0]) + " " + String(profileArrX[1]) + " " + String(profileArrX[2]) + " " + String(profileArrX[3]));
      Serial.println("Profile Z: " + String(profileArrZ[0]) + " " + String(profileArrZ[1]) + " " + String(profileArrZ[2]) + " " + String(profileArrZ[3]));
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

  switch (checker_t)
  {
  case Step_t_1:
    digitalWrite(penddown, LOW); // поднимаем диск
    // digitalWrite(pendout, LOW); // зажим диска
    // delay(2000); // Поднять маятник: закрыть реле 1(нижний клапан) открыть реле 2(верхний клапан)
    if (moveY(140.1))
      checker_t = Step_t_2;
    break;
  case Step_t_2:
    if (isDown || true)
    {
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

      // Serial.println(stepperX.getCurrentPositionInMillimeters());
      checker_t = Step_t_4;
      // Serial.println(stepperX.getCurrentPositionInMillimeters());
    }
    break;
  case Step_t_4:
    if (moveX(1))
    {
      // Serial.println(stepperX.getCurrentPositionInMillimeters());
      checker_t = Step_t_5;

      // Serial.println(stepperX.getCurrentPositionInMillimeters());
    }
    break;
  case Step_t_5:
    if (moveX(profileArrX[0] + x_bias))
    {
      checker_t = Step_t_6;
    }
    break;
  case Step_t_6:
    if (moveC(-(profileArrZ[0] - 12)))
    {
      stepperC.setSpeedInMillimetersPerSecond(abs(profileArrZ[0] - profileArrZ[1])/(abs(profileArrX[1] - profileArrX[0]) / xspeed));
      stepperC.setAccelerationInMillimetersPerSecondPerSecond(abs(profileArrZ[0] - profileArrZ[1])/(abs(profileArrX[1] - profileArrX[0]) / xspeed) * 2);
      checker_t = Step_t_7;
    }
    break;
  case Step_t_7:
    moveX(profileArrX[3] + x_bias);
    if (moveC(-(profileArrZ[1] - 12)))
    {
      checker_t = Step_t_8;
    }
    break;
  case Step_t_8:
    moveX(profileArrX[3] + x_bias);
    if (abs(stepperX.getCurrentPositionInMillimeters() - profileArrX[2]) < 0.1)
    {
      stepperC.setSpeedInMillimetersPerSecond(abs(profileArrZ[3] - profileArrZ[2])/(abs(profileArrX[3] - profileArrX[2]) / xspeed));
      stepperC.setAccelerationInMillimetersPerSecondPerSecond(abs(profileArrZ[3] - profileArrZ[2])/(abs(profileArrX[3] - profileArrX[2]) / xspeed) * 2);
      checker_t = Step_t_9;
    }
    break;
  case Step_t_9:
    moveX(profileArrX[3] + x_bias);
    if (moveC(-(profileArrZ[3] - 12)))
    {
      stepperC.setSpeedInStepsPerSecond(speedZ);
      stepperC.setAccelerationInStepsPerSecondPerSecond(accZ);
      checker_t = Step_t_10;
    }
    break;
  case Step_t_10:
    if (moveC(-1) && moveX(profileArrX[3] + x_bias))
    {
            stepperX.setSpeedInMillimetersPerSecond(24);
      stepperX.setAccelerationInMillimetersPerSecondPerSecond(48);
      checker_t = Step_t_11;
    }
    break;
  case Step_t_11:
    if (moveX(1))
    {
      checker_t = Step_t_12;
    }
    break;
  default:
    break;
  }

  return checker_t == Step_t_12;
}

uint8_t checkProfileFun(float speed, float time)
{

  switch (stateProfile)
  {
  case startCheck:
    if (moveY(170.9)){
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
        // Serial.println(leftVoltage == rightVoltage);
        // Serial.println(leftX);
        // Serial.println(leftZ);
        // Serial.println(rightZ);
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
      if (abs(leftZ - rightZ) < 0.15)
      {
        profileArrX[1] = getXfromlazer(1);
        profileArrZ[1] = rightZ;
        Serial.println("----------------------------");
        // Serial.println(leftVoltage);
        // Serial.println(leftX);
        // Serial.println(leftZ);
        //           Serial.println(rightZ);

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
      if (abs(leftZ - rightZ) > 0.3)
      {
        Serial.println("----------------------------");
        // Serial.println(leftVoltage);
        // Serial.println(leftX);
        // Serial.println(leftZ);
        //           Serial.println(rightZ);

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
        // Serial.println(leftVoltage);
        // Serial.println(leftX);
        // Serial.println(leftZ);
        // Serial.println(rightZ);
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

// uint8_t checkX(uint32_t k){
//   float x = getXfromlazer();
//   if (!isnan(x)) Serial.println(x);//profileArr[k] = x;
//   return 0;
// }

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

  // Serial.println(vx*1000);
  // Serial.println(vy);
  // Serial.println(hx);
  // Serial.println(hy);

  // Serial.println(rightZ);
  // Serial.println(leftZ);

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
  float y = -sign * vy * r / sqrt(vx * vx + vy * vy) + rightZ;//sign == 1 ? -vy * r / sqrt(vx * vx + vy * vy) + rightZ : -vy * r / sqrt(vx * vx + vy * vy) + leftZ;

  // Serial.println(vx*1000);
  // Serial.println(vy);
  // Serial.println(hx);
  // Serial.println(hy);

  // Serial.println(rightZ);
  // Serial.println(leftZ);

  return y;
}

void updateLazer()
{
  leftX = rightX;
  leftZ = rightZ;
  leftVoltage = rightVoltage;
  rightVoltage = voltage;
  rightZ = voltage * 6.1648049166;
  rightX = stepperX.getCurrentPositionInMillimeters();
}

void transmitData()
{

  Serial.print("x:");
  Serial.print(stepperX.getCurrentPositionInMillimeters());
  Serial.print(",y:");
  Serial.println(stepperY.getCurrentPositionInMillimeters());//stepperY.getCurrentPositionInMillimeters()); // Send data in the format x:{int},y:{int}

  // Serial.println( analogRead(potentiometer_pin)); // 1180 - 5.26 точка конька x: 28.57,y:154.72 28 - метров 0 ADC 0 метров - максимум ADC
  // Serial.println(voltage * 6.164804916); // 1180 - 5.26 точка конька x: 28.57,y:154.72 28 - метров 0 ADC 0 метров - максимум ADC
  // Serial.println(getXfromlazer());
  //   Serial.println("----------------------------");
  // Serial.println(leftVoltage);
  // Serial.println(leftX);
  // Serial.println(leftZ);
}
