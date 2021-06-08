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

//Importing Public Libraries//
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
#include "autoFunctions2.h"
//#include "AutonomousFunc.h"
// ---------- End ------------//

AutoFunctions autoFunctions;

bool Thomas = true;

vex::competition Competition;
using namespace vex;




void autonomous(){ // Forward KP = 0.2 KD = 0.1

 flipout(1000);
 vexDelay(500);
 //cycleC(1000,500); // for the corner
 //cycleC(500, 500); // middle but not actual middle goal 
 //skills6();
 //cycleM(1000);
// vexDelay(5000);
 //visionAlign(RED_BALL);
 //inPerson();
 //TurnRightPD(30, 0.8, 0.2);
 worldsW();
 //visionTest();
 
}





void test2() {
  vexDelay(2000);
  //visionAlign(RED_BALL, 0.05, 0.05);
  //ForwardIntakePD(2000, 0.27, 0, 0.1);
  
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
    Brain.Screen.printAt(20, 200, "%d percent", Inertail_rotation());
    Brain.Screen.printAt(250, 175, "DriveTemp: %d ", (LB.temperature() + LF.temperature() + RB.temperature() + RF.temperature()) / 4);
    // sleep 1000 msecs
    this_thread::sleep_for(10);
    // clear the line
    Brain.Screen.clearLine(20, 200);
    Brain.Screen.clearLine(250, 175);

  }
  return 0;
}

void pre_auton(){
  // Initializing Robot Configuration. DO NOT REMOVE!


  task BrainScreen = task(brainScreen);
}

void intakeOut() {
  IntakeL.spin(reverse, 100, rpm);
  IntakeR.spin(reverse, 100, rpm);
}












void buttonL1() {
  IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
  IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);

  waitUntil(!Controller1.ButtonL1.pressing());

  BottomIndexer.stop(coast);
  IntakeL.stop(coast);
  IntakeR.stop(coast);


}


void buttonL2() {
    IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);

    waitUntil(!Controller1.ButtonL2.pressing());

    BottomIndexer.stop(coast);
    IntakeL.stop(coast);
    IntakeR.stop(coast);

}


void buttonR1() {
  TopIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);

  waitUntil(!Controller1.ButtonR2.pressing());

  TopIndexer.stop(coast);
  BottomIndexer.stop(coast);
  IntakeR.stop(coast);
  IntakeL.stop(coast);
  
}


void buttonR2() {
  TopIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  IntakeL.spin(reverse, 140, pct);
  IntakeR.spin(reverse, 140, pct);

  waitUntil(!Controller1.ButtonR1.pressing());

  TopIndexer.stop(coast);
  BottomIndexer.stop(coast);
  IntakeL.stop(coast);
  IntakeR.stop(coast);

}


void usercontrol(){
  // button callbacks
  Controller1.ButtonL1.pressed(buttonL1);
  Controller1.ButtonL2.pressed(buttonL2);
  Controller1.ButtonR1.pressed(buttonR2);
  Controller1.ButtonR2.pressed(buttonR1);
  while (true) {

      // configure deadzones
      double strafeDeadzone = 2;
      double turnDeadzone = 2;
      double deadzone = 2;


      // get lateral and turning variables from controller joystick positions
      double power = Controller1.Axis3.position(pct);
      double turn = Controller1.Axis1.position(pct);
      double strafe = Controller1.Axis4.position(pct);
      double left = power + turn;
      double right = power - turn;

      // raw strafing power for each motor
      double sLF = 0;
      double sLB = 0;
      double sRF = 0;
      double sRB = 0;

      // final power for each motor
      double powerLF = 0;
      double powerLB = 0;
      double powerRF = 0;
      double powerRB = 0;

      // strafe deadzone
      if (fabs(strafe) > strafeDeadzone) { // if it is not within the strafe deadzone the strafe
        sLF = strafe;
        sLB = -strafe;
        sRF = -strafe;
        sRB = strafe;

      } else { // else dont deliver strafing power
        sLF = 0;
        sLB = 0;
        sRF = 0;
        sRB = 0;
      }

      // if the turn deadzone is not reached do not deliver turn power
      if (fabs(turn) <  turnDeadzone) {
        turn = 0;
      }

      // if the forward/backward deadzone is not reached do not deliver turn power
      if (fabs(power) < deadzone) {
        power = 0;
      }

      // set total motor powers
      powerLF = left + sLF;
      powerLB = left + sLB;
      powerRF = right + sRF;
      powerRB = right + sRB;


      // move motors
      // LF motor
      if (fabs(powerLF) > 600) {  // if statement so the motor does not always go max speed
        LF.spin(fwd, (power/powerLF + turn/powerLF + sLF/powerLF)*600, rpm); // balance out values
      } else {
        LF.spin(fwd, powerLF * 6, rpm);  // else move normally
      }

      // LB motor
      if (fabs(powerLB) > 600) {
        LB.spin(fwd, (power/powerLB + turn/powerLB + sLB/powerLB)*600, rpm);
      } else {
        LB.spin(fwd, powerLB * 6, rpm);
      }

      // RF motor
      if (fabs(powerRF) > 600) {
        RF.spin(fwd, (power/powerRF - turn/powerRF + sRF/powerRF)*600, rpm);
      } else {
        RF.spin(fwd, powerRF * 6, rpm);
      }

      // RB motor
      if (fabs(powerRB) > 600) {
        RB.spin(fwd, (power/powerRB - turn/powerRB + sRB/powerRB)*600, rpm);
      } else {
        RB.spin(fwd, powerRB * 6, rpm);
      }


    wait(10, msec); // don't waste resources
  }
}


////////////////////////////////////////////////////
// autonomous timer
////////////////////////
int timeBoi = 0;

int autonTimer() {

  while (Competition.isAutonomous()) {
    timeBoi++;
    wait(1000, msec);
  }

  return 0;
}



int main() {
  // Set up callbacks for autonomous and driver control periods.
  vexcodeInit();
  inertial1.calibrate();
  inertial2.calibrate();
  vex::wait(500, msec);
  
  waitUntil(!inertial1.isCalibrating() && !inertial2.isCalibrating());
  task::sleep(20);



  Competition.autonomous(autonomous);
  if (Thomas) {
    Competition.drivercontrol(usercontrol);
  } else {
    Competition.drivercontrol(usercontrol);
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



  

  return 0;
}
