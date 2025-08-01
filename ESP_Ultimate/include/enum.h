#pragma once

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

enum DiskHandling {
  DiskStart,
  // Disk on Chuck steps
  Chuck_MoveX,
  Chuck_RaiseZ,
  Chuck_MoveYOverBed,
  Chuck_LowerZ,
  Chuck_OpenClamp,
  Chuck_RetractY,
  Chuck_CloseClamp,
  // Disk in Bed steps
  Bed_MoveXZ,
  Bed_OpenClamp,
  Bed_MoveYToDisk,
  Bed_CloseClamp,
  Bed_RaiseZ,
  Bed_RetractY,
  Bed_LowerZ,
  // Return to zero
  ReturnToZero,
  DiskHandlingDone
};