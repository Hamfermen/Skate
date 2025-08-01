#include "static.h"
#include <Arduino.h>

float scale_mm_to_steps_x = 64.116f * 5.97f; // 1.26 за 1000 шагов 7.524
float scale_mm_to_steps_y = 52.9661f;        // 1.26 за 1000 шагов 7.524
float scale_mm_to_steps_z = 505.3316f;

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

float profileArrX[50];
float profileArrZ[50];

float profileArrSpZ[50];

float minZ = 1000;

uint32_t profileArrN = 0;
uint32_t profileInd = 0;
float step = 3;
long long timeCheck = 0;

float disks[6] = {-0.25, -0.41666666, -0.5833333333, -0.75, -0.91666666666, -0.08333333};
float disks_Y[6] = {11.2, 11.2, 11.2, 11.2, 11.2, 11.2};