#include "vex.h"
#include "robot-config.h"
#include "AutoFunctions.h"
#include "autoFunctions2.h"
#include "visionL.h"


void VisionTurnLeftPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation()- degree);
    double derivative;
    double prevError = 0;
    double TurnPower;
    bool first = true;

    int range = 1;
    while (fabs(error) > range || first) {
      error = (fabs(Inertail_rotation()- (degree)));
      derivative = error - prevError;
      TurnPower = (error * kP + derivative * kD);

      //spin leftMotor reverse by error * kP;
      LF.spin(reverse,TurnPower, pct);
      RF.spin(forward,TurnPower, pct);
      LB.spin(reverse,TurnPower, pct);
      RB.spin(forward,TurnPower, pct);
      
      wait(5, msec);
      first = false;
    }
    DriveBreak();
   
}


void VisionTurnRightPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);
    double derivative;
    double prevError = 0;
    double TurnPower;
    double first = true;

    int range = 1;
    while (fabs(error) > range || first) {
      error = (fabs(Inertail_rotation()- (degree)));
      derivative = error - prevError;
      TurnPower = (error * kP + derivative * kD);

      //spin leftMotor reverse by error * kP;
      LF.spin(forward,TurnPower, pct);
      RF.spin(reverse,TurnPower, pct);
      LB.spin(forward,TurnPower, pct);
      RB.spin(reverse,TurnPower, pct);
      
      wait(5, msec);
      first = false;
    }
    DriveBreak();
   
}


void visionAlign(vex::vision::signature objSig, double lDivide, double rDivide) {
  // set variables
  Vision1.takeSnapshot(objSig);
  Controller1.rumble("-");
  double vError = 158 - Vision1.largestObject.centerX; // too far to right = negative, too far to left = positive


  if (fabs(vError) > 1){  // exit condition, range of error (vision sensor readings fluctuate) and speed (won't have enough power when close)
    // where it want to be - where it is
    vError = 158 - Vision1.largestObject.centerX; // too far to right = negative, too far to left = positive

    if(vError < 0) {
      VisionTurnRightPD(fabs(vError)/rDivide, 0.9, 0.01); // 
    } else {
      VisionTurnLeftPD(fabs(vError)/lDivide, 2, 0.3); // 5.2
    }

  }
  DriveBreak(); // stop the drivetrain when its done
}




void worlds(){
  vexDelay(3000);
  SlipForwardIntakePDR(2300, 0.11, 0, 0.05, 100, 10);
  /*cycleM(1120);
  vexDelay(100);
  BackwardPD(1150, 0.1310, 0.5, 10);
  vexDelay(180);
  TurnRightPD(128, 0.45, 0.5);
  vexDelay(200);
  ForwardIntakePD(1700, 0.15, 0, 0.01, 5);
  vexDelay(200);
  Controller1.rumble("---");
  TopIndexer.stop(coast);
  BottomIndexer.stop(coast);
  RF.spin(reverse, 40, pct);
  RB.spin(fwd, 40, pct);
  LF.spin(fwd, 40, pct);
  LB.spin(reverse, 40, pct);
  wait(1900, msec);
  RF.stop(coast);
  RB.stop(coast);
  LF.stop(coast);
  LB.stop(coast);

 // vexDelay(200);
  BackwardNPD(1400, 0.13, 0.3, 6);
  vexDelay(100);
  TurnRightPD(80, 0.8, 0.3);
  vexDelay(100);
  ForwardIntakePD(3000, 0.12, 0, 0.3, 4);
  cycleM(800);*/


}


void worldsW(){
  vexDelay(3000);
  SlipForwardIntakePDR(2300, 0.11, 0, 0.05, 100, 10);
  cycleM(1080);
  vexDelay(100);
  BackwardPD(1100, 0.1310, 0.5, 10);    
  
  /*vexDelay(300);
  TurnRightPD(129, 0.45, 0.5);
  vexDelay(200);
  LF.spin(reverse, 60, percent);
  LB.spin(reverse, 60, percent);
  RF.spin(reverse, 60, percent);
  RB.spin(reverse, 60, percent);
  wait(1000, msec);
  LF.stop();
  LB.stop();
  RF.stop();
  RB.stop();
  distanceForwardIntakePD(2530, 0.1 , 0, 0.01, 0.2);
  vexDelay(200);
  TopIndexer.stop(coast);
  BottomIndexer.stop(coast);
  RF.spin(reverse, 35, pct);
  RB.spin(fwd, 40, pct);
  LF.spin(fwd, 40, pct);
  LB.spin(reverse, 40, pct);
  wait(1750, msec);
  RF.stop(coast);
  RB.stop(coast);
  LF.stop(coast);
  LB.stop(coast);

  vexDelay(200);
  BackwardNPD(1700, 0.13, 0.3, 6);
  vexDelay(100);
  TurnRightPD(90,0.7,0.5);
  ForwardIntakePD(3000,0.12,0,0.3,10);
  cycleM(1125);
 TurnRightPD(86, 0.8, 0.3);
  vexDelay(100);
  ForwardIntakePD(3000, 0.12, 0, 0.3, 4);
  cycleM(800);*/
}

void visionTest() {
  Controller1.rumble("---");
  visionAlign(RED_BALL, 2.0, 2.0);
  Controller1.rumble("---");
}






void worldsWS(){
  vex::thread autoTimer = thread(autonTimer);
  SlipForwardIntakePD(2300, 0.11, 0, 0.05, 100, 10);
  //SlipForwardIntakePDR(2000, 0.11, 0, 0.05, 100, 20);


























































































  cycleM(950);
  vexDelay(100);
  BackwardPD(800,0.13,0.5,10);
 /* TurnLeftPD(135, 0.45, 0.5);
  LF.spin(reverse, 60, percent);
  LB.spin(reverse, 60, percent);
  RF.spin(reverse, 60, percent);
  RB.spin(reverse, 60, percent);
  wait(1250, msec);
  LF.stop();
  LB.stop();
  RF.stop();
  RB.stop();
  ForwardPD(4650,0.11,0,0.05, 0.5);
  TurnRightPD(43,0.8,0.2);
  IntakeL.spin(forward, 140, pct);
  IntakeR.spin(forward, 140, pct);
  vexDelay(300);
  ForwardPD(1100, 0.11, 0, 0.05, 2);
  IntakeL.stop();
  IntakeR.stop();
  BottomIndexer.spin(forward, 140, percent);
  TopIndexer.spin(reverse, 140, percent);
  //forwardiS();
  //BottomIndexer.spin(forward, 140, pct);
  //TopIndexer.spin(forward, 140, pct);*/
  
  
}