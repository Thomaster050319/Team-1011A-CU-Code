/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PIDS.cpp                                                  */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

using namespace vex;


void ForwardIntakePD(double goal, float KP, float KI, float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;

    lateralmotorpower = (error * KP + derivative * KD);
    
    IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
    task::sleep(10);
  }
  DriveBreak();
}

void ForwardPD(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;

    lateralmotorpower = (error * KP + derivative * KD);
    
    
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
      task::sleep(10);
  }
  DriveBreak();
}

void BackwardPD(double goal, float KP, float KD){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error > 3) { 
    error = goal - avgEnc();
    derivative = error - prevError;
    lateralmotorpower = (error * KP + derivative * KD);
    
    LB.spin(reverse,lateralmotorpower,pct);
    LF.spin(reverse,lateralmotorpower,pct);
    RB.spin(reverse,lateralmotorpower,pct);
    RF.spin(reverse,lateralmotorpower,pct);

    prevError = error;
      task::sleep(10);

  }
  DriveBreak();
}

void TurnLeft(double degree, float kP) {
    Inertial_reset();
  
    double error = (inertial1.rotation()- degree);

    int range = 4;
    while (fabs(error) > range) {
      error = (fabs(inertial1.rotation()- (degree)));
      //spin leftMotor reverse by error * kP;
      LF.spin(reverse, error * kP, pct);
      RF.spin(forward, error * kP, pct);
      LB.spin(reverse, error * kP, pct);
      RB.spin(forward, error * kP, pct);
      
      wait(10, msec);
    }
    DriveBreak();
   
}
void TurnRight(double degree, float kP) {
    Inertial_reset();
  
    double error = (Inertail_rotation()- degree);

    int range = 4;
    while (fabs(error) > range) {
      error = (fabs(Inertail_rotation()- (degree)));
      //spin leftMotor reverse by error * kP;
      LF.spin(forward, error * kP, pct);
      RF.spin(reverse, error * kP, pct);
      LB.spin(forward, error * kP, pct);
      RB.spin(reverse, error * kP, pct);
      
      wait(10, msec);
    }
    DriveBreak();
   
}

void TurnRightPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation()- degree);
    double derivative;
    double prevError = 0;
    double TurnPower;

    int range = 4;
    while (fabs(error) > range) {
      error = (fabs(Inertail_rotation()- (degree)));
      derivative = error - prevError;
      TurnPower = (error * kP + derivative * kD);

      //spin leftMotor reverse by error * kP;
      LF.spin(forward,TurnPower, pct);
      RF.spin(reverse,TurnPower, pct);
      LB.spin(forward,TurnPower, pct);
      RB.spin(reverse,TurnPower, pct);
      
      wait(10, msec);
    }
    DriveBreak();
   
}
void TurnLeftPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation()- degree);
    double derivative;
    double prevError = 0;
    double TurnPower;

    int range = 4;
    while (fabs(error) > range) {
      error = (fabs(Inertail_rotation()- (degree)));
      derivative = error - prevError;
      TurnPower = (error * kP + derivative * kD);

      //spin leftMotor reverse by error * kP;
      LF.spin(reverse,TurnPower, pct);
      RF.spin(forward,TurnPower, pct);
      LB.spin(reverse,TurnPower, pct);
      RB.spin(forward,TurnPower, pct);
      
      wait(10, msec);
    }
    DriveBreak();
   
}

