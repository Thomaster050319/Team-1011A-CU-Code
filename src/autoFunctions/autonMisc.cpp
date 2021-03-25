/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PIDvariables.cpp                                          */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

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

void shoot(int time) {
  TopIndexer.setVelocity(100, percent);
  BottomIndexer.setVelocity(100, percent);
  TopIndexer.spin(forward);
  BottomIndexer.spin(reverse);
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
