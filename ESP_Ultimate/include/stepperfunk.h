#pragma once

#include <Arduino.h>
#include "SpeedyStepper.h"
#include "FlexyStepper.h"
#include "enum.h"

extern DisksCheck checkerD;
extern Sharpering checker;
extern Sharpering_t checker_t;
extern InitialZero initialState;
extern CheckDiamentr diamChecker;
extern Alarm alarmProcessor;
extern CheckProfile stateProfile;
extern DiskHandling diskHandler;

extern SpeedyStepper stepperX, stepperY, stepperD, stepperCl;
extern FlexyStepper stepperC;

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
uint8_t handleDisk(uint8_t isDiskOnChuck);

int8_t checkAllDisks();

void transmitData();
void recieveData();
void handler();
void initializeZeros();
void alarmProcess();
uint8_t checkProfileFun(float, float);