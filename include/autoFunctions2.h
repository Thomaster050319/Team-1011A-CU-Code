#pragma once
#include "vex.h"

void ForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange);
void ForwardOutakePD(double goal, float KP,float KI,float KD, double slewMaxChange);
void ForwardPD(double goal, float KP,float KI,float KD, double slewMaxChange);
void BackwardPD(double goal, float KP, float KD, double slewMaxChange);
void BackwardAlignPD(double goal, float KP, float KD, double slewMaxChange);
void BackwardOPD(double goal, float KP, float KD, double slewMaxChange);
void TurnLeft(double degree, float kP);
void TurnRight(double degree, float kP);
void TurnRightPD(double degree, float kP,float kD);
void TurnLeftPD(double degree, float kP,float kD);
void shoot(int time);
void insuck(int time);
void insuck2(int time);
void stopshoot();
void flipout(int time);
void descore(int time);
void cycleC(int time, int timev);
void cycleM(int time);
void downSpin();
void SmoothLeftIntakePD(double goal, float KP, float KI, float KD, double slewMaxChange, double turnKP, double turnKD, double turnDegrees, double turnStartPoint);
void SlipForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange, double degree);
void StrafePD(double goal, float KP,float KI,float KD, double slewMaxChange);
void BackwardNPD(double goal, float KP, float KD, double slewMaxChange);
void visionAlign(vex::vision::signature objSig, double lDivide, double rDivide);
int Inertail_rotation();
void Inertial_reset();
void DriveBreak();
void distanceForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange);
void SlipForwardIntakePDR(double goal, float KP,float KI,float KD, double slewMaxChange, double degree);
void worldsWS();
void cycleMS(int time);
void forwardiS();


void worlds();
void worldsW();
void visionTest();

int autonTimer();
extern int timeBoi;