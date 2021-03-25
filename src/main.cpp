/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Thomas Lee                                                */
/*    Created:      Sun Mar 07 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- PORT NUMBERS FOR VEXCODE CONFIGURED DEVICES ----
/*

*/
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "global.h"

using namespace vex;
competition Competition;

// pre autonomous
void pre_auton(){
  
}

void autonomous(){ // Forward KP = 0.2 KD = 0.1
  pre_auton();
  skills3();
  //test();
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // calibrate inertial sensors here so they don't waste time during autonomous (calibrates when disabled by comm switch)
  inertial1.calibrate();
  inertial2.calibrate();
  
  waitUntil(!inertial1.isCalibrating() && !inertial2.isCalibrating());
  task::sleep(100);

  // run tasks the need to be ran
  task ControllerDisplay = task(controllerDisplay);


  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);


  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
