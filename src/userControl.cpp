/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       userControl.cpp                                           */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

using namespace vex;

void userControl(){

  bool Donald = false;
  bool Liam = false;
  int deadband = 15;

  if (Liam) {
    while (true) {
      
    }
  }



  while (true) {
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
    if (Donald) {
      IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
    } else {
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
