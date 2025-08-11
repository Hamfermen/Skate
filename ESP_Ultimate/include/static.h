#pragma once
#include <Arduino.h>

// Объявления переменных
extern float scale_mm_to_steps_x;
extern float scale_mm_to_steps_y;
extern float scale_mm_to_steps_z;

extern uint8_t isTouched;
extern float diam;

extern long long time_now;
extern long long time_to_up;
extern long long time_to_down;

extern int64_t timeTransm;
extern int64_t timeClamp;
extern int64_t grind_time;

extern uint8_t isSharpering;
extern uint8_t isGrinding;
extern uint8_t isInit;

extern uint8_t isChange;

extern uint8_t isAlarmButton;
extern uint8_t isTurnOnButton;
extern uint8_t isProfButton;
extern uint8_t isZeroButton;
extern uint8_t isSharpButton;
extern uint8_t isGrindButton;
extern uint8_t isChoosingButton;
extern uint8_t isXButton;
extern uint8_t isYButton;
extern uint8_t isClButton;
extern uint8_t isDButton;

extern uint32_t profNumTemp;
extern uint32_t xNumTemp;
extern uint32_t yNumTemp;
extern uint32_t clNumTemp;
extern uint32_t dNumTemp;

extern uint8_t d_zero;
extern uint8_t x_zero;
extern uint8_t y_zero;
extern uint8_t c_zero;
extern uint8_t cl_zero;
extern uint8_t alarm_pos;

extern uint8_t isDown;

extern uint8_t isChoosing;
extern uint32_t profNum;

extern uint8_t isX;
extern uint32_t xNum;

extern uint8_t isY;
extern uint32_t yNum;

extern uint8_t isCl;
extern uint32_t clNum;

extern uint8_t isD;
extern uint32_t dNum;

extern uint8_t isAlarm;
extern uint8_t isTurnOn;
extern uint8_t isProf;

extern uint8_t conterweihgt_zero;
extern uint8_t zeroInit;

extern uint64_t speedX;
extern uint64_t accX;

extern uint64_t speedY;
extern uint64_t accY;

extern uint64_t speedZ;
extern uint64_t accZ;

extern uint8_t isChecked;

extern uint32_t whichSharp;
extern uint32_t count_of_Sharps;

extern float rightZ, leftZ, rightX, leftX, leftVoltage, rightVoltage;
extern float voltage;

extern float profileArrX[50];
extern float profileArrZ[50];

extern float profileArrSpZ[50];

extern float minZ;

extern uint32_t profileArrN;
extern uint32_t profileInd;
extern float step;
extern long long timeCheck;

extern float disks[6];
extern float disks_Y[6];
