/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\tlee0                                            */
/*    Created:      Sun Mar 07 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//Importing Pre-Existing Files//
#include "vex.h"
#include "robot-config.h"
#include "AutoFunctions.h"
//#include "AutonomousFunc.h"
// ---------- End ------------//



vex::competition Competition;
using namespace vex;


////////////variables////////////

bool Thomas = true; // true for Thomas's driver code, false for Liam's


// Function that returns average encoder position of the drivetrain motors
int avgEnc(){
  return((fabs(LF.position(deg) + LB.position(deg) + RF.position(deg) + RB.position(deg))) / 4);
}

bool hitDetected() {
  if(inertial1.acceleration(yaxis) < 0 || inertial2.acceleration(yaxis) < 0 || accel1.acceleration() < 0 || bumperLeft.pressing() || bumperRight.pressing()) {
    return true;
  } else {
    return false;
  }
}

// resets encoders for PID loops
void resetEnc(){
  LB.resetPosition();
  LF.resetPosition();
  RB.resetPosition();
  RF.resetPosition();
}

// stops all drivetrain motors
void DriveBreak(){
  LB.stop(brake);
  LF.stop(brake);
  RB.stop(brake);
  RF.stop(brake);
}

// sets heading of inertial sensors to 0 for ease of use
void Inertial_reset(){
  inertial1.setRotation(0, deg);
  inertial2.setRotation(0, deg);
}

// returns average heading of inertial sensor 1 and 2
int Inertail_rotation(){
  return(fabs(inertial1.rotation() + inertial2.rotation()));
  //return (fabs(inertial2.rotation()))
}

// PD loop to go forward while intaking
void ForwardIntakePD(double goal, float KP,float KI,float KD){
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  //lateral motor power//
  double lateralmotorpower;

  while (error > 3) {
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

// PD loop for drifting (stopping does not work ATM)
void DriftPD(double goal, float KP,float KI,float KD){
  
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

  int hitCount = 0;

  while (error > 3){
    if (hitDetected()) {
      hitCount++;
      if(hitCount == 2) {
        error = 0;
        prevError = 0;
        derivative = 0;
        totalerror = 0;
        lateralmotorpower = 0;
        hitCount = 0;
      }
    } else {
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
  }
  DriveBreak();


}

// PD loop that outakes while going forward
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
    if (hitDetected()) {
      vexDelay(50);
      resetEnc();
      error = 0;
      goal = 0;
      prevError = 0;
      derivative = 0;
      totalerror = 0;

    } else {
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
  }
  DriveBreak();
}

// PD loop that goes forward
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
    if (hitDetected()) {
      vexDelay(50);
      resetEnc();
      error = 0;
      lateralmotorpower = 0;
      prevError = 0;
      derivative = 0;
      totalerror = 0;
      lateralmotorpower = 0;

      task::sleep(10);

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

// PD loop for going backwards
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

// function to align with field perimeter
void BackwardAlignPD(double goal, float KP, float KD){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error > 3) { 
    if (inertial1.acceleration(yaxis) > 0 || inertial2.acceleration(yaxis) > 0 || accel1.acceleration() > 0) {
      vexDelay(150);
      resetEnc();
      error = 0;
      lateralmotorpower = 0;
      goal = 0;
      prevError = 0;
      derivative = 0;
      
      task::sleep(10);
    }
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

// KD loop to go backward while outaking
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

// P loop to turn left
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

// P loop to turn right
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

// PD loop to turn right
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

// PD loop for turning left
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

// function to shoot ball into goal
void shoot(int time) {
  TopIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop(brake);
  
  

}

// function to idk
void insuck(int time){
  TopIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::fwd, 20, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop();
}

// function to stopball when limit switch is pressed
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

// function to stop indexers
void stopshoot(){
  TopIndexer.stop(brake);
  BottomIndexer.stop(brake);
}

// function to stop drivetrain and intakes
void forwardintakestop(){
  IntakeL.stop();
  IntakeR.stop();
  LB.stop(brake);
  LF.stop(brake);
  RB.stop(brake);
  RF.stop(brake);
}

// function to run to activate flipout for set amount of time
void flipout(int time){
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  task::sleep(time);
  BottomIndexer.stop(brake);
}

// decore for set time
void descore(int time){
    IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    task::sleep(time);
    IntakeR.stop();
    IntakeL.stop();
    BottomIndexer.stop();
}







// skils function 1.0
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

// skills function 2.0
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
  TurnRightPD(139.78654,0.9,0.1); // 138
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








// skills function 3.0
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
  BackwardPD(730,0.25,0.1);
  vexDelay(100);
  TurnRightPD(141.3,0.9,0.1);//was 141
  vexDelay(100);
  ForwardIntakePD(1690,0.25,0,0.3); //was 1650
  forwardintakestop();
  TurnLeftPD(93,0.9,0.1);
  ForwardPD(2000,0.3,0,0.1); //2nd goal forward (2000 to gauruntee touching)
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
  vexDelay(250);
  shoot(300);
  vexDelay(100);
  TurnRightPD(106,0.7,0.1); //3rd to 4th angle 
  ForwardIntakePD(2100,0.30,0,0.1);//3rd to 4th transition
  forwardintakestop();
  TurnLeftPD(85,0.9,0.1);
  ForwardPD(1000,0.3,0,0.1); //4th goal forward
  insuck(200);
  shoot(500);// 4th goal shoot
  vexDelay(100);
  BackwardPD(370,0.3,0.1); //340
  vexDelay(400);
  TurnRightPD(93,0.8,0.1); // change from 93 to 92 to 94.5
  vexDelay(150);
  ForwardIntakePD(1850,0.3,0,0.1);//4th to 5th
  forwardintakestop();
  TurnLeftPD(46,0.8,0.1);
  ForwardPD(800,0.3,0,0.1); //was 590
  insuck(200);
  shoot(600);
  BackwardPD(690,0.3,0.1);//changed from 650
  vexDelay(200);
  TurnRightPD(137, 0.8, 0.3);// turn from 5th to 6th
  ForwardIntakePD(1610, 0.27, 0, 0.1);//1580
  forwardintakestop();
  TurnLeftPD(92,0.9,0.1);
  ForwardPD(2000,0.3,0,0.1); // was 390
  insuck(200);
  shoot(600);
  BackwardPD(355,0.3,0.1);
  TurnRightPD(64,0.8,0.1);
  ForwardIntakePD(2225, 0.3, 0, 0.1);
  forwardintakestop();
  insuck(400);
  shoot(540);
  BackwardOPD(350,0.3,0.1);
  TurnRightPD(120,0.9,0.1);
  BackwardAlignPD(350,0.6,0.3);
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
  TurnLeftPD(180,0.8,0.1); //changed from 182
  ForwardIntakePD(1000, 0.35, 0, 0.01);
  
  
  TurnLeftPD(10, 0.8, 0.1);
  ForwardOutakePD(500,0.25,0,0.1);
  LF.spin(forward, 10000000, rpm);
  LB.spin(forward, 10000000, rpm);
  RB.stop(brake);
  RF.stop(brake);
  vexDelay(300);
  LF.stop(coast);
  LB.stop(coast);
  RF.stop(coast);
  RB.stop(coast);
  shoot(590);
  BackwardPD(200, 0.3, 0.1);
}





// skills function 4.0
void skills4(){
  // --- First Goal --- //
  ForwardIntakePD(935,0.27,0,0.5);
  vexDelay(150);
  TurnLeftPD(133,0.75,0.3);
  forwardintakestop();
  vexDelay(100);
  ForwardPD(1150,0.4,0,0.3);
  insuck(400);
  shoot(450);
  vexDelay(100);
  BackwardPD(730,0.25,0.1);
  vexDelay(100);
  TurnRightPD(141.3,0.9,0.1); //was 141
  visionAlign(RED_BALL, 0.9, 0.1); // aligns with the red ball
  ForwardIntakePD(1690,0.25,0,0.3); //was 1650
  forwardintakestop();
  TurnLeftPD(93,0.9,0.1);
  ForwardPD(2000,0.3,0,0.1); //2nd goal forward (2000 to gauruntee touching)
  insuck(300);
  shoot(400);
  vexDelay(100);
  BackwardPD(400,0.25,0.1);//changed from 600 (1st wall drift)
  forwardintakestop();
  TurnRightPD(65,0.7,0.1);
  visionAlign(RED_BALL, 0.9, 0.1); // align with red ball
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
  vexDelay(250);
  shoot(300);
  vexDelay(100);
  TurnRightPD(106,0.7,0.1); //3rd to 4th angle 
  visionAlign(RED_BALL, 0.9, 0.1); // align with ball
  ForwardIntakePD(2100,0.30,0,0.1);//3rd to 4th transition
  forwardintakestop();
  TurnLeftPD(85,0.9,0.1);
  ForwardPD(1000,0.3,0,0.1); //4th goal forward
  insuck(200);
  shoot(500);// 4th goal shoot
  vexDelay(100);
  BackwardPD(370,0.3,0.1); //340
  vexDelay(400);
  TurnRightPD(93,0.8,0.1); // change from 93 to 92 to 94.5
  visionAlign(RED_BALL, 0.9, 0.1); // aligns with red ball
  ForwardIntakePD(1850,0.3,0,0.1);//4th to 5th
  forwardintakestop();
  TurnLeftPD(46,0.8,0.1);
  ForwardPD(800,0.3,0,0.1); //was 590
  insuck(200);
  shoot(600);
  BackwardPD(690,0.3,0.1);//changed from 650
  vexDelay(200);
  TurnRightPD(137, 0.8, 0.3);// turn from 5th to 6th
  visionAlign(RED_BALL, 0.9, 0.1);
  ForwardIntakePD(1610, 0.27, 0, 0.1);//1580
  forwardintakestop();
  TurnLeftPD(92,0.9,0.1);
  ForwardPD(2000,0.3,0,0.1); // was 390
  insuck(200);
  shoot(600);
  BackwardPD(355,0.3,0.1);
  TurnRightPD(64,0.8,0.1);
  visionAlign(RED_BALL, 0.9, 0.1);
  ForwardIntakePD(2225, 0.3, 0, 0.1);
  forwardintakestop();
  insuck(400);
  shoot(540);
  BackwardOPD(350,0.3,0.1);
  TurnRightPD(120,0.9,0.1);
  BackwardAlignPD(350,0.6,0.3);
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
  TurnLeftPD(180,0.8,0.1); //changed from 182
  visionAlign(RED_BALL, 0.9, 0.1); // align with red ball
  ForwardIntakePD(1000, 0.35, 0, 0.01);
  
  
  TurnLeftPD(10, 0.8, 0.1);
  ForwardOutakePD(500,0.25,0,0.1);
  LF.spin(forward, 10000000, rpm);
  LB.spin(forward, 10000000, rpm);
  RB.stop(brake);
  RF.stop(brake);
  vexDelay(300);
  LF.stop(coast);
  LB.stop(coast);
  RF.stop(coast);
  RB.stop(coast);
  shoot(590);
  BackwardPD(200, 0.3, 0.1);
}





// test stopball functions
void test(){
  shoot(300);
  stopball();
  BackwardPD(500,0.3,0.1);
  stopshoot();
}

// functions to run during autonomous period
void autonomous(){ // Forward KP = 0.2 KD = 0.1

 flipout(100);
 vexDelay(400);
 
 skills3();
 //test();
}


// Thomas's driver control code
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


// Liam's drivercontrol code
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
        Controller1.Axis3.position() - Controller1.Axis1.position() * drive;
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

// Function that returns average temperature of drivetrain motors
double drivetrainTemp() {
  return (LB.temperature() + LF.temperature() + RB.temperature() + RF.temperature()) / 4;
}

// Main thread for brain screen display
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

  while(true) {
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

// pre autonomous code
void pre_auton(){

  inertial1.calibrate();
  inertial2.calibrate();
  
  waitUntil(!inertial1.isCalibrating() && !inertial2.isCalibrating());
  task::sleep(100);

  task BrainScreen = task(brainScreen);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

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

  return 0;
}
