/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       displays.cpp                                              */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

using namespace vex;

int controllerDisplay() {
  while(true) {
    Controller1.Screen.clearLine();
    Controller1.Screen.print(Inertail_rotation());
    
    vexDelay(300);
  }

  return 0;
}