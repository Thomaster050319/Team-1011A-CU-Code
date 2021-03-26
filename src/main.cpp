/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\tlee0                                            */
/*    Created:      Sun Mar 07 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- PORT NUMBERS FOR VEXCODE CONFIGURED DEVICES ----
/*

*/
// ---- END VEXCODE CONFIGURED DEVICES ----

//Importing Public Libraries//a
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Importing Pre-Existing Files//
#include "vex.h"
#include "v5.h"
#include "v5_vcs.h"
#include "robot-config.h"
#include "AutoFunctions.h"
//#include "AutonomousFunc.h"
// ---------- End ------------//

bool Thomas = false;

vex::competition Competition;
using namespace vex;
int avgEnc(){
  return((fabs(LF.position(deg) + LB.position(deg) + RF.position(deg) + RB.position(deg))) / 4);
}

void resetEnc(){
  LB.resetPosition();
  LF.resetPosition();
  RB.resetPosition();
  RF.resetPosition();
}

void DriveBreak(){
  LB.stop(brake);
  LF.stop(brake);
  RB.stop(brake);
  RF.stop(brake);
}

void Inertial_reset(){
  inertial1.setRotation(0, deg);
  inertial2.setRotation(0, deg);
}

int Inertail_rotation(){
  return(fabs(inertial1.rotation() + inertial2.rotation()));
  //return (fabs(inertial2.rotation()))
}

int Inertial1_acceleration() {
  return inertial1.acceleration(yaxis);
}

int Inertial2_acceleration() {
  return inertial2.acceleration(yaxis);
}

void AutoFunctions::resetPID() {
  error = 0;
  prevError = 0;
  derivative = 0;
  totalError = 0;
}

void AutoFunctions::autoTurnTo(double degrees) {
  int t = 0;               // Time variable
  while (t < turnMargin) { // break when time exceeds the turnMargin
    // PID
    error = Inertail_rotation() - degrees;
    derivative = error - prevError;
    totalError += error;
    prevError = error;

    // Run motors according to PID values
    LB.spin(forward,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 vex::pct);
    RB.spin(forward,
                 error * turnkP + totalError * turnkI + derivative * turnkD,
                 vex::pct);
    LF.spin(forward,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 vex::pct);
    RF.spin(forward,
                 error * turnkP + totalError * turnkI + derivative * turnkD,
                 vex::pct);
           
    wait(loopTime, msec); // Wait to prevent wasted resources
    // Exit the turn function once the robot is pointing in the correct
    // direction
    if (fabs(error) < turnRange) { // increase time value when the robot is
                                   // pointing within turnRange
      t += loopTime;
    } else {
      t = 0;
    }
  }
  resetPID();
  // stop the drive
  DriveBreak();
}


void ForwardIntakePD(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;
    totalerror += error;

    lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);
    
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

void ForwardWeirdIntakePD(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;
    totalerror += error;

    lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);

    if (error < 800) {
      IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else {
      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    }
    
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
      task::sleep(10);
  }
  DriveBreak();


}

void ForwardWeirdIntakePD2(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;
    totalerror += error;

    lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);

    if (error < 2100) {
      IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else {
      IntakeR.stop();
      IntakeL.stop();
    }
    
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
      task::sleep(10);
  }
  DriveBreak();


}



void ForwardOutakePD(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    error = goal - avgEnc();
    derivative = error - prevError;
    totalerror += error;

    lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);
    
    IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  
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
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3){
    if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0) {
      vexDelay(50);
      resetEnc();
      error = 0;
      prevError = 0;
      derivative = 0;
      totalerror = 0;
      lateralmotorpower = 0;
    } else {
          error = goal - avgEnc();
    derivative = error - prevError;
    totalerror += error;

    lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);
    
    
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
      task::sleep(10);
    }
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
    if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0) {
      vexDelay(150);
      error = 0;
      prevError = 0;
      derivative = 0;
      lateralmotorpower = 0;
    } else {
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
  }
  DriveBreak();
}

void BackwardOPD(double goal, float KP, float KD){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error > 3) { 
    error = goal - avgEnc();
    derivative = error - prevError;
    lateralmotorpower = (error * KP + derivative * KD);
    IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::fwd, 60, vex::velocityUnits::pct);
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

    int range = 5;
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

    int range = 5;
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
void shoot(int time) {
  TopIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop(brake);
  
  

}
void insuck(int time){
  TopIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::fwd, 20, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop();
}
void stopball(){
  Brain.Timer.reset(); 
  while(1){
    if(limit1.pressing()) {
        TopIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
        BottomIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
        break;

  
      
    }
  }


}
void stopshoot(){
  TopIndexer.stop(brake);
  BottomIndexer.stop(brake);
}
void forwardintakestop(){
  IntakeL.stop();
  IntakeR.stop();
  LB.stop(brake);
  LF.stop(brake);
  RB.stop(brake);
  RF.stop(brake);
}
void flipout(int time){
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  task::sleep(time);
  BottomIndexer.stop(brake);
}
void descore(int time){
    IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    task::sleep(time);
    IntakeR.stop();
    IntakeL.stop();
    BottomIndexer.stop();
}
void skills1(){ // Left = - Right = +
//////////1st row ///////////
  ForwardIntakePD(1000,0.25,0,0.1);
  vexDelay(250);
  TurnLeftPD(138,0.8,0.1);
  forwardintakestop();
  vexDelay(300);
  ForwardPD(1100,0.25,0,0.1);
  shoot(300); // 1st goal 
  //stopball();
  vexDelay(300);
  BackwardPD(590,0.2,0.1);
  //stopshoot();
  vexDelay(400);
  TurnRightPD(138,0.9,0.1);
  vexDelay(400);
  ForwardIntakePD(1650,0.25,0,0.1);
  forwardintakestop();
  TurnLeftPD(90,0.7,0.1);
  ForwardPD(335,0.2,0,0.1);
  shoot(400); // 2nd goal
  //stopball();
  vexDelay(300);
  BackwardPD(275,0.2,0.1);
  //stopshoot();
  TurnRightPD(90, 0.7, 0.1);
  ForwardPD(1700, 0.3, 0, 0.1);
  TurnLeftPD(45,0.7,0.1);
  vexDelay(200);
  ForwardPD(380,0.25,0,0.1);
  shoot(400); // 3rd goal
  vexDelay(150);
  ///////// 2nd row /////////
  BackwardPD(380,0.3,0.1);
  TurnRightPD(135,0.7,0.1);
  ForwardIntakePD(2075,0.30,0,0.1);
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(335,0.3,0,0.1);
  shoot(400); // 4th goal
  vexDelay(100);
  BackwardPD(290,0.3,0.1);

  TurnRightPD(94,0.8,0.1);
  vexDelay(100);
  ForwardIntakePD(1700,0.3,0,0.1);
  forwardintakestop();
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(580,0.3,0,0.1);
  shoot(400);
  BackwardPD(580, 0.3, 0.1);
  /*///////// 3rd row /////////
  TurnRightPD(161, 0.8, 0.1);
  ForwardIntakePD(1700, 0.3, 0, 0.1); 
  forwardintakestop();
  TurnLeftPD(19,0.8,0.1);
  ForwardIntakePD(1480, 0.3, 0, 0.1);
  shoot(400);
  vexDelay(150);
  BackwardPD(630, 0.3, 0.1);
  TurnRightPD(90,0.8,0.1);
  ForwardPD(1700,0.3,0,0.1);
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(500,0.3,0,0.1);
  shoot(400);
  vexDelay(150);
  BackwardPD(500, 0.3, 0.1);
  TurnRightPD(135,0.8,0.1);
  ///////// 4th row /////////
  ForwardIntakePD(1700,0.3,0,0.1);
  TurnLeftPD(90,0.8,0.1);
  shoot(400);
  vexDelay(150);
  TurnRightPD(180, 0.8, 0.1);
  ///////// Middle Goal /////////
  ForwardIntakePD(1500,0.3,0,0.1);
  ForwardIntakePD(100,0.3,0,0.1);
  BackwardPD(100,0.3,0.1);
  ForwardIntakePD(150, 0.3,0, 0.1);
  shoot(400);
  BackwardPD(200, 0.3, 0.1);*/

  






}
void skills2(){
  ForwardIntakePD(1000,0.27,0,0.1);
  vexDelay(100);
  TurnLeftPD(138,0.6,0.1);
  forwardintakestop();
  vexDelay(200);
  ForwardPD(1150,0.4,0,0.1);
  shoot(350);
  vexDelay(150);
  BackwardPD(550,0.25,0.1);
  vexDelay(200);
  TurnRightPD(138,0.9,0.1);
  vexDelay(200);
  ForwardIntakePD(1660,0.25,0,0.1);
  forwardintakestop();
  TurnLeftPD(92,0.7,0.1);
  ForwardPD(360,0.2,0,0.1); //2nd goal forward
  shoot(300);
  vexDelay(200);
  BackwardPD(750,0.25,0.1);
  TurnRightPD(63,0.7,0.1);
  vexDelay(200);
  ForwardPD(2200,0.27,0,0.1);
  shoot(400); // 3rd goal
  vexDelay(150);
  ///////// 2nd row /////////
  BackwardPD(400,0.3,0.1);
  TurnRightPD(110,0.7,0.1); //3rd to 4th angle 
  ForwardIntakePD(1900,0.30,0,0.1);//3rd to 4th transition
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(335,0.3,0,0.1); //4th goal forward
  shoot(400);
  vexDelay(100);
  BackwardPD(290,0.3,0.1);
  TurnRightPD(92,0.8,0.1);
  vexDelay(100);
  ForwardIntakePD(1700,0.3,0,0.1);
  forwardintakestop();
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(550,0.3,0,0.1);
  shoot(400);
  /*vexDelay(150);
  BackwardPD(550,0.3,0.1);
  ///////// 3rd row /////////
  TurnRightPD(161, 0.8, 0.1);
  ForwardIntakePD(1900, 0.3, 0, 0.1); 
  forwardintakestop();
  TurnLeftPD(19,0.8,0.1);
  ForwardIntakePD(1480, 0.3, 0, 0.1);
  shoot(400);
  vexDelay(150);
  BackwardPD(630, 0.3, 0.1);
  TurnRightPD(90,0.8,0.1);
  ForwardPD(1700,0.3,0,0.1);
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(500,0.3,0,0.1);
  shoot(400);
  vexDelay(150);
  BackwardPD(500, 0.3, 0.1);
  TurnRightPD(135,0.8,0.1);
  ///////// 4th row /////////
  ForwardIntakePD(1700,0.3,0,0.1);
  TurnLeftPD(90,0.8,0.1);
  shoot(400);
  vexDelay(150);
  TurnRightPD(180, 0.8, 0.1);
  ///////// Middle Goal /////////
  ForwardIntakePD(1500,0.3,0,0.1);
  ForwardIntakePD(100,0.3,0,0.1);
  BackwardPD(100,0.3,0.1);
  ForwardIntakePD(150, 0.3,0, 0.1);
  shoot(400);
  BackwardPD(200, 0.3, 0.1);*/
  
}
void skills3(){
  ForwardIntakePD(935,0.27,0,0.5);
  vexDelay(150);
  TurnLeftPD(133,0.75,0.3);
  forwardintakestop();
  vexDelay(100);
  ForwardPD(1150,0.4,0,0.3);
  insuck(400);
  shoot(450);
  vexDelay(100);
  BackwardPD(720,0.25,0.1);
  vexDelay(100);
  TurnRightPD(144,0.9,0.1);
  vexDelay(100);
  ForwardIntakePD(1690,0.25,0,0.3); //was 1650
  forwardintakestop();
  TurnLeftPD(93,0.9,0.1);
  ForwardPD(295,0.3,0,0.1); //2nd goal forward (was 320)
  insuck(300);
  shoot(400);
  vexDelay(100);
  BackwardPD(400,0.25,0.1);//changed from 600 (1st wall drift)
  forwardintakestop();
  TurnRightPD(65,0.7,0.1);
  vexDelay(200);
  ForwardIntakePD(2210,0.27,0,0.1);
  forwardintakestop();
  vexDelay(100);
  insuck(200);
  shoot(600); // 3rd goal
  vexDelay(150);
  ///////// 2nd row /////////
  BackwardOPD(380,0.3,0.1);
  vexDelay(100);
  forwardintakestop();
  vexDelay(150);
  TurnRightPD(108,0.7,0.1); //3rd to 4th angle 
  ForwardIntakePD(2090,0.30,0,0.1);//3rd to 4th transition
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(400,0.3,0,0.1); //4th goal forward
  insuck(200);
  shoot(400);// 4th goal shoot
  vexDelay(100);
  BackwardPD(340,0.3,0.1);
  vexDelay(150);
  TurnRightPD(93,0.8,0.1);
  vexDelay(150);
  ForwardIntakePD(1920,0.3,0,0.1);//4th to 5th
  forwardintakestop();
  TurnLeftPD(46,0.8,0.1);
  ForwardPD(450,0.3,0,0.1); //was 590
  insuck(200);
  shoot(600);
  BackwardPD(680,0.3,0.1);//changed from 650
  vexDelay(200);
  TurnRightPD(136, 0.8, 0.3);// turn from 5th to 6th
  ForwardIntakePD(1620, 0.27, 0, 0.1);//1580
  forwardintakestop();
  TurnLeftPD(92,0.9,0.1);
  ForwardPD(300,0.3,0,0.1); // was 390
  insuck(200);
  shoot(600);
  BackwardPD(355,0.3,0.1);
  TurnRightPD(64,0.8,0.1);
  ForwardWeirdIntakePD2(2225, 0.3, 0, 0.1);
  forwardintakestop();
  insuck(400);
  shoot(540);
  BackwardOPD(350,0.3,0.1);
  TurnRightPD(120,0.9,0.1);
  BackwardPD(340,0.6,0.3);
  ForwardIntakePD(2280,0.35,0,0.1);
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(300,0.3,0,0.1);
  insuck(400);
  shoot(720);
  forwardintakestop();
  descore(1000);
  BackwardOPD(335, 0.2, 0.1);
  IntakeL.stop();
  IntakeR.stop();
  TopIndexer.stop();
  BottomIndexer.stop();
  TurnLeftPD(175,0.8,0.1); //changed from 182
  ForwardWeirdIntakePD(1530, 0.35, 0, 0.01);// changed from 0.25
  
  
  //TurnLeftPD(10, 0.8, 0.1);
 // ForwardIntakePD(500,0.25,0,0.1);
 // forwardintakestop();
  insuck(400);
  shoot(590);
  BackwardPD(200, 0.3, 0.1);
}
void test(){
  shoot(300);
  stopball();
  BackwardPD(500,0.3,0.1);
  stopshoot();
}

void autonomous(){ // Forward KP = 0.2 KD = 0.1

 flipout(100);
 vexDelay(400);
 
 skills3();
 //test();
}

void usercontrol(){

  bool Donald = false;
  int deadband = 0;
  while (1) {
    Brain.Screen.newLine();
    double drive = 1;
    /*int Ch1 = Controller1.Axis1.value();
    int Ch3 = Controller1.Axis3.value();
    int Ch4 = Controller1.Axis4.value();
    Brain.Screen.print(Ch4);
    LeftMotorF.spin(directionType::fwd, Ch3 - Ch1 - Ch4,velocityUnits::pct);
    LeftMotorB.spin(directionType::fwd, Ch3 + Ch1 - Ch4,velocityUnits::pct);
    RightMotorF.spin(directionType::fwd, Ch3 + Ch1 + Ch4,velocityUnits::pct);
    RightMotorB.spin(directionType::fwd, Ch3 - Ch1 + Ch4,velocityUnits::pct);*/
    //Get the velocity percentage of the left motor. (Axis3 + Axis1)
    int leftMotorSpeed =
        Controller1.Axis3.position() + Controller1.Axis1.position() * drive;
    // Get the velocity percentage of the right motor. (Axis3 - Axis1)
    int rightMotorSpeed =
        Controller1.Axis3.position() - Controller1.Axis1.position()* drive;
    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    if (Controller1.ButtonUp.pressing()){
      drive = 1.0;
    
    }
    if (Controller1.ButtonDown.pressing()){
      drive = 0.5;
    }
    if (Controller1.ButtonL1.pressing()) {
      Donald = true;
    }
    if (Controller1.ButtonUp.pressing()) {
      Donald = false;
    }
      if (abs(leftMotorSpeed) < deadband) {
      // Set the speed to zero.
      LF.setVelocity(0, percent);
      LB.setVelocity(0, percent);
    } else {
      // Set the speed to leftMotorSpeed
      LF.setVelocity(leftMotorSpeed, percent);
      LB.setVelocity(leftMotorSpeed, percent);
    }
    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(rightMotorSpeed) < deadband) {
      // Set the speed to zero
      RF.setVelocity(0, percent);
      RB.setVelocity(0, percent);
    } else {
      // Set the speed to rightMotorSpeed
      RF.setVelocity(rightMotorSpeed, percent);
      RB.setVelocity(rightMotorSpeed, percent);
    }
    // Spin both motors in the forward direction.
    LF.spin(forward);
    LB.spin(forward);
    RF.spin(forward);
    RB.spin(forward);
    wait(25, msec);
    // Tank Drive Code
    if (Donald == true) {
      IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else if (Donald == false) {
      IntakeR.stop();
      IntakeL.stop();
      BottomIndexer.stop();
    }
    if (Controller1.ButtonR1.pressing()) {
      TopIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeR.spin(directionType::fwd, 127, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 127, vex::velocityUnits::pct);
    } else if (Controller1.ButtonR2.pressing()) {
      TopIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else if (Controller1.ButtonL2.pressing()) {
      IntakeL.spin(directionType::rev, 127, vex::velocityUnits::pct);
      IntakeR.spin(directionType::rev, 127, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else if (Controller1.ButtonL1.pressing()) {
      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    } else {
      TopIndexer.stop();
      BottomIndexer.stop();
      IntakeL.stop();
      IntakeR.stop();
    }
  }
}

void usercontrol2(){

  bool Donald = false;
  int deadband = 0;
  while (1) {
    Brain.Screen.newLine();
    double drive = 1;
    /*int Ch1 = Controller1.Axis1.value();
    int Ch3 = Controller1.Axis3.value();
    int Ch4 = Controller1.Axis4.value();
    Brain.Screen.print(Ch4);
    LeftMotorF.spin(directionType::fwd, Ch3 - Ch1 - Ch4,velocityUnits::pct);
    LeftMotorB.spin(directionType::fwd, Ch3 + Ch1 - Ch4,velocityUnits::pct);
    RightMotorF.spin(directionType::fwd, Ch3 + Ch1 + Ch4,velocityUnits::pct);
    RightMotorB.spin(directionType::fwd, Ch3 - Ch1 + Ch4,velocityUnits::pct);*/
    //Get the velocity percentage of the left motor. (Axis3 + Axis1)
    int leftMotorSpeed =
        Controller1.Axis3.position() + Controller1.Axis1.position() * drive;
    // Get the velocity percentage of the right motor. (Axis3 - Axis1)
    int rightMotorSpeed =
        Controller1.Axis3.position() - Controller1.Axis1.position()* drive;
    // Set the speed of the left motor. If the value is less than the deadband,
    // set it to zero.
    if (Controller1.ButtonUp.pressing()){
      drive = 1.0;
    
    }
    if (Controller1.ButtonDown.pressing()){
      drive = 0.5;
    }
    if (Controller1.ButtonL1.pressing()) {
      Donald = true;
    }
    if (Controller1.ButtonUp.pressing()) {
      Donald = false;
    }
      if (abs(leftMotorSpeed) < deadband) {
      // Set the speed to zero.
      LF.setVelocity(0, percent);
      LB.setVelocity(0, percent);
    } else {
      // Set the speed to leftMotorSpeed
      LF.setVelocity(leftMotorSpeed, percent);
      LB.setVelocity(leftMotorSpeed, percent);
    }
    // Set the speed of the right motor. If the value is less than the deadband,
    // set it to zero.
    if (abs(rightMotorSpeed) < deadband) {
      // Set the speed to zero
      RF.setVelocity(0, percent);
      RB.setVelocity(0, percent);
    } else {
      // Set the speed to rightMotorSpeed
      RF.setVelocity(rightMotorSpeed, percent);
      RB.setVelocity(rightMotorSpeed, percent);
    }
    // Spin both motors in the forward direction.
    LF.spin(forward);
    LB.spin(forward);
    RF.spin(forward);
    RB.spin(forward);
    wait(25, msec);
    // Tank Drive Code
    if (Donald == true) {
      IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else if (Donald == false) {
      IntakeR.stop();
      IntakeL.stop();
      BottomIndexer.stop();
    }
    if (Controller1.ButtonR1.pressing()) {
/*      TopIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeR.spin(directionType::fwd, 127, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 127, vex::velocityUnits::pct);*/
      IntakeL.spin(reverse, 140, percent);
      IntakeR.spin(reverse, 140, percent);
    } else if (Controller1.ButtonR2.pressing()) {
/*       TopIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);*/
      IntakeL.spin(forward, 140, percent);
      IntakeR.spin(forward, 140, percent);
    } else if (Controller1.ButtonL2.pressing()) {
/*      IntakeL.spin(directionType::rev, 127, vex::velocityUnits::pct);
      IntakeR.spin(directionType::rev, 127, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct); */
            TopIndexer.spin(reverse, 140, percent);
      BottomIndexer.spin(reverse, 140, percent);
    } else if (Controller1.ButtonL1.pressing()) {
/*      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);*/
            TopIndexer.spin(forward, 140, percent);
      BottomIndexer.spin(forward, 140, percent);
    } else {
      TopIndexer.stop();
      BottomIndexer.stop();
      IntakeL.stop();
      IntakeR.stop();
    }
  }
}


double drivetrainTemp() {
  return (LB.temperature() + LF.temperature() + RB.temperature() + RF.temperature()) / 4;
}

int brainScreen() {
  // draw boxes here so we dont waste cpu resources
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.drawRectangle(0, 110, 200, 110);
  Brain.Screen.drawRectangle(210, 110, 210, 110);
  Brain.Screen.drawImageFromFile("image.png", 200, 180);
  Brain.Screen.setFont(propXXL);
  Brain.Screen.setPenColor(green);
  Brain.Screen.print("1011A");
  Brain.Screen.setPenColor(white);

  for(;;) {
    Brain.Screen.setFont(monoL);
    
    // display text in the boxes
    Brain.Screen.printAt(20, 150, "Battery:");
    Brain.Screen.printAt(20, 200, "%d percent", Brain.Battery.capacity());
    Brain.Screen.printAt(250, 175, "DriveTemp: %d ", (LB.temperature() + LF.temperature() + RB.temperature() + RF.temperature()) / 4);
    // sleep 1000 msecs
    this_thread::sleep_for(1000);
    // clear the line
    Brain.Screen.clearLine(20, 200);
    Brain.Screen.clearLine(250, 175);

  }
  return 0;
}

void pre_auton(){
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  inertial1.calibrate();
  inertial2.calibrate();
  
  waitUntil(!inertial1.isCalibrating() && !inertial2.isCalibrating());
  task::sleep(100);

  task BrainScreen = task(brainScreen);
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  if (Thomas) {
    Competition.drivercontrol(usercontrol);
  } else {
    Competition.drivercontrol(usercontrol2);
  }

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
  inertial1.calibrate();
  // waits for the Inertial Sensor to calibrate
  while (inertial1.isCalibrating()) {
    wait(100, msec);
  }
  waitUntil((inertial1.rotation(degrees) >= 90.0));
  RB.stop();
  RF.stop();
  LB.stop();
  LF.stop();
  wait(1, seconds); 
  return 0;
}
