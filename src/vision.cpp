#include "vex.h"
#include "AutoFunctions.h"

// 158 x is center of screen

void visionAlign(vex::vision::signature objSig, double vKP, double vKD) {
  double vError = 69420;
  double vDerivative = 0;
  double vPrevError = 0;
  double vMotorPower = 0;

  while (vError != 0) {
    Vision1.takeSnapshot(objSig);
    vError = 158 - Vision1.largestObject.centerX;
    vDerivative = vError - vPrevError;
    vMotorPower = vError * vKP + vDerivative * vKD;

    if (vError > 0) { // if the target is too much to the left, turn left
      LF.spin(forward, vMotorPower, velocityUnits::pct);
      LB.spin(forward, vMotorPower, velocityUnits::pct);
      RF.spin(reverse, vMotorPower, velocityUnits::pct);
      RB.spin(reverse, vMotorPower, velocityUnits::pct);

    } else { // if the target is too much too much to the right, turn right
      LF.spin(reverse, vMotorPower, velocityUnits::pct);
      LB.spin(reverse, vMotorPower, velocityUnits::pct);
      RF.spin(forward, vMotorPower, velocityUnits::pct);
      RB.spin(forward, vMotorPower, velocityUnits::pct);
    }

    vPrevError = vError;

    task::sleep(50);
  }
}