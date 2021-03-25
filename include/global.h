#include "vex.h"

// PID variables

// auton return functions
int avgEnc();
int Inertail_rotation();

// auton non-return functions
void resetEnc();
void DriveBreak();
void Inertial_reset();
void ForwardIntakePD(double goal, float KP, float KI, float KD);
void ForwardPD(double goal, float KP,float KI,float KD);
void BackwardPD(double goal, float KP, float KD);
void TurnLeft(double degree, float kP);
void TurnRight(double degree, float kP);
void TurnRightPD(double degree, float kP,float kD);
void TurnLeftPD(double degree, float kP,float kD);
void shoot(int time);
void insuck(int time);
void stopball();
void stopshoot();
void forwardintakestop();

// autons
void skills1();
void skills2();
void skills3();

// robot control periods
void pre_auton();
void userControl();

// displays
int controllerDisplay();
