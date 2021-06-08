#include "vex.h"
#include "robot-config.h"
#include "autoFunctions.h"

using namespace vex;

/////////////////////////////////////////////////////////////
// Return the average encoder value of the tracking wheels
////////////////////////////
int avgEnc(){
  return(fabs((-lTrackingWheel.rotation(deg) + -rTrackingWheel.rotation(degrees))) / 2.0);
}

////////////////////////////////////////////////////////////
// Reset the tracking wheel encoders
//////////////////////////////////
void resetEnc(){
 lTrackingWheel.resetRotation();
 rTrackingWheel.resetRotation();
 mTrackingWheel.resetRotation();
}

///////////////////////////////////////////////////////////
// Stop the drivetrain
/////////////////////////
void DriveBreak(){
  LB.stop(brake);
  LF.stop(brake);
  RB.stop(brake);
  RF.stop(brake);
}

/////////////////////////////////////////////////////////
// Reset the inertial sensors
//////////////////////////////////
void Inertial_reset(){
  inertial1.setRotation(0, deg);
  inertial2.setRotation(0, deg);
  inertial3.setRotation(0, deg);
}

////////////////////////////////////////////////////////
// Return the average value of the inertial sensors
////////////////////////////////////
int Inertail_rotation(){
  return(fabs((inertial1.rotation() + inertial2.rotation() + inertial3.rotation())/3));
  //return (fabs(inertial2.rotation()))
}

///////////////////////////////////////////////////////
// return the value of the acceleration of imu1
//////////////////////////////////
int Inertial1_acceleration() {
  return inertial1.acceleration(yaxis);
}

////////////////////////////////////////////////////
// return the value of the acceleration of imu2
//////////////////////////////////
int Inertial2_acceleration() {
  return inertial2.acceleration(yaxis);
}

//////////////////////////////////////////////////////////
// return the value of the acceleration of imu3
//////////////////////////////////
int Inertial3_acceleration() {
  return inertial3.acceleration(yaxis);
}



/////////////////////////////////////////////////////////
// reset PID of prev advanced code
//////////////////////////////////////
void AutoFunctions::resetPID() {
  error = 0;
  prevError = 0;
  derivative = 0;
  totalError = 0;
}

//////////////////////////////////////////////////////////
// prev adavced turn to right code
///////////////////////////////////////////////////
void AutoFunctions::autoTurnToR(double degrees) {
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

//////////////////////////////////////////////////////////////
// previous advanced code turn left function
/////////////////////////////////////
void AutoFunctions::autoTurnToL(double degrees) {
  int t = 0;               // Time variable
  while (t < turnMargin) { // break when time exceeds the turnMargin
    // PID
    error = Inertail_rotation() - degrees;
    derivative = error - prevError;
    totalError += error;
    prevError = error;

    // Run motors according to PID values
    LB.spin(reverse,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 vex::pct);
    RB.spin(reverse,
                 error * turnkP + totalError * turnkI + derivative * turnkD,
                 vex::pct);
    LF.spin(reverse,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 vex::pct);
    RF.spin(reverse,
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



///////////////////////////////////////////////
// Forward Intake PD
////////////////////////////
void ForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange){ // revert for skills
  
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
  double prevMotorPower = 0;



  while (error > 3) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
    
      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
      LB.spin(forward,lateralmotorpower,pct);
      LF.spin(forward,lateralmotorpower,pct);
      RB.spin(forward,lateralmotorpower,pct);
      RF.spin(forward,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);
      }
      
  IntakeL.stop();
  IntakeR.stop();
  DriveBreak();
}




//////////////////////////////////////////////////////
// Forward outake PD
//////////////////////////
void ForwardOutakePD(double goal, float KP,float KI,float KD, double slewMaxChange){
  
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
  double prevMotorPower = 0;

  while (error > 3){
    if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0) {
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

    if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
    
    IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  
    TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
    LB.spin(forward,lateralmotorpower,pct);
    LF.spin(forward,lateralmotorpower,pct);
    RB.spin(forward,lateralmotorpower,pct);
    RF.spin(forward,lateralmotorpower,pct);

    prevError = error;
    prevMotorPower = lateralmotorpower;

      task::sleep(10);
    }
  }
  DriveBreak();
}

///////////////////////////////////////////////////////
// Forward PD
////////////////////////////
void ForwardPD(double goal, float KP,float KI,float KD, double slewMaxChange){
  
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
  double prevMotorPower = 0;

  while (error > 3){
    if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0) {
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

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }

      BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
      LB.spin(forward,lateralmotorpower,pct);
      LF.spin(forward,lateralmotorpower,pct);
      RB.spin(forward,lateralmotorpower,pct);
      RF.spin(forward,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;

      task::sleep(10);
    }
  }
  DriveBreak();
}



/////////////////////////////////////////////////////////////////////
// Backward PD
//////////////////////////////
void BackwardPD(double goal, float KP, float KD, double slewMaxChange){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;
  double prevMotorPower = 0;

  while(error > 3) { 
      error = goal - avgEnc();
      derivative = error - prevError;
      lateralmotorpower = (error * KP + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
  
      TopIndexer.spin(directionType::fwd, 1000000000, voltageUnits::volt);
      IntakeR.spin(directionType::rev, 80, vex::velocityUnits::pct);
      IntakeL.spin(directionType::rev, 80, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      LB.spin(reverse,lateralmotorpower,pct);
      LF.spin(reverse,lateralmotorpower,pct);
      RB.spin(reverse,lateralmotorpower,pct);
      RF.spin(reverse,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);


  }
  DriveBreak();
}

void BackwardNPD(double goal, float KP, float KD, double slewMaxChange){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;
  double prevMotorPower = 0;

  while(error > 3) { 
      error = goal - avgEnc();
      derivative = error - prevError;
      lateralmotorpower = (error * KP + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
  
      LB.spin(reverse,lateralmotorpower,pct);
      LF.spin(reverse,lateralmotorpower,pct);
      RB.spin(reverse,lateralmotorpower,pct);
      RF.spin(reverse,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);


  }
  DriveBreak();
}
//////////////////////////////////////////
// Align with the wall
////////////////////////
void BackwardAlignPD(double goal, float KP, float KD, double slewMaxChange){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;
  double prevMotorPower = 0;

  while(error > 3) { 
    if (Inertial1_acceleration() > 0 || Inertial2_acceleration() > 0) {
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

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
    
      LB.spin(reverse,lateralmotorpower,pct);
      LF.spin(reverse,lateralmotorpower,pct);
      RB.spin(reverse,lateralmotorpower,pct);
      RF.spin(reverse,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);


  }
  DriveBreak();
}


////////////////////////////////////////////
// Backward outake PD
//////////////////////////////
void BackwardOPD(double goal, float KP, float KD, double slewMaxChange){
  resetEnc();
  double error = goal - avgEnc();
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;
  double prevMotorPower = 0;

  while(error > 3) { 
    error = goal - avgEnc();
    derivative = error - prevError;
    lateralmotorpower = (error * KP + derivative * KD);

    if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }

    TopIndexer.spin(directionType::rev, 1000000000, voltageUnits::volt);
    IntakeR.spin(directionType::rev, 80, vex::velocityUnits::pct);
    IntakeL.spin(directionType::rev, 80, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::fwd, 60, vex::velocityUnits::pct);
    LB.spin(reverse,lateralmotorpower,pct);
    LF.spin(reverse,lateralmotorpower,pct);
    RB.spin(reverse,lateralmotorpower,pct);
    RF.spin(reverse,lateralmotorpower,pct);
    

    prevError = error;
    prevMotorPower = lateralmotorpower;
      task::sleep(10);

  }
  DriveBreak();
}


/////////////////////////////////////////////////
// Turn left P
//////////////////////////
void TurnLeft(double degree, float kP) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);

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


//////////////////////////////////////////////////
// Turn right P
/////////////////////
void TurnRight(double degree, float kP) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);

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


////////////////////////////////////////////////////////
// Turn right PD
////////////////////////
void TurnRightPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);
    double derivative;
    double prevError = 0;
    double TurnPower;

    int range = 1;
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


//////////////////////////////////////////
// Turn left PD
/////////////////////
void TurnLeftPD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);
    double derivative;
    double prevError = 0;
    double TurnPower;

    int range = 1;
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

/////////////////////////////////////////////////////
// Shoot balls into goals
//////////////////////////////
void shoot(int time) {
  TopIndexer.spin(directionType::fwd, 1000000000, voltageUnits::volt);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop(brake);
}

///////////////////////////////////////////////////
// Move the balls up the chamber
///////////////////////////////////
void insuck(int time){
  TopIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::fwd, 20, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop();
}

////////////////////////////////////////////////////////
// Move the balls up to the chamber iteration 2
/////////////////////////////////////////////////
void insuck2(int time){
  TopIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 20, vex::velocityUnits::pct);
  task::sleep(time);
  TopIndexer.stop();
}

///////////////////////////////////////////////////////
// Stop shooting forcefully
//////////////////////////////////
void stopshoot(){
  TopIndexer.stop(brake);
  BottomIndexer.stop(brake);
}

//////////////////////////////////////////////////////
// Flipout hood
////////////////////////////////////
void flipout(int time){
  TopIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  IntakeR.spin(reverse, 40, pct);
  task::sleep(time);
  TopIndexer.stop(brake);
  IntakeR.stop();
}

/////////////////////////////////////////////////////
// descore balls from a goal
////////////////////////////////////////
void descore(int time){
    IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
    IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
    BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
    task::sleep(time);
    IntakeR.stop();
    IntakeL.stop();
    BottomIndexer.stop();
}


////////////////////////////////////////////////////
// Cycle iteration 1
//////////////////////////////
void cycleC(int time, int timev) {
  insuck(150);
  TopIndexer.spin(directionType::fwd, 1000000000, voltageUnits::volt);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  vexDelay(timev);
  TopIndexer.stop();
  TopIndexer.spin(directionType::rev, 1000000000, voltageUnits::volt);
  IntakeR.spin(directionType::rev, 140, vex::velocityUnits::pct);
  IntakeL.spin(directionType::rev, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);

  task::sleep(time);

  TopIndexer.stop();
  BottomIndexer.stop();
  IntakeL.stop();
  IntakeR.stop();
}


///////////////////////////////////////////////////////
// Cycle iteration 2
////////////////////////////////////
void cycleM(int time) {
  insuck(150);
  IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  TopIndexer.spin(directionType::rev, 1000000000, voltageUnits::volt);


  task::sleep(time);

  TopIndexer.stop();
  BottomIndexer.stop();
  IntakeL.stop();
  IntakeR.stop();
}
void cycleMS(int time) {
  insuck(150);
  IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
  BottomIndexer.spin(directionType::rev, 140, vex::velocityUnits::pct);
  TopIndexer.spin(directionType::rev, 1000000000, voltageUnits::volt);


  task::sleep(time);

  TopIndexer.stop();
  BottomIndexer.stop();
  IntakeL.stop();
  IntakeR.stop();
}

//////////////////////////////////////////////////////
// move the balls down the chamber
////////////////////////////////////
void downSpin() {
  BottomIndexer.spin(forward, 140, percent);
  TopIndexer.spin(reverse, 140, percent);
  wait(135, msec);
  BottomIndexer.stop();
  TopIndexer.stop();
}






/////////////////////////////////////////////////////////
// Smooth, combined turning and forward movement PD
///////

///////////////////////////////////////////////
// Forward Intake PD
////////////////////////////
void SmoothLeftIntakePD(double goal, float KP, float KI, float KD, double slewMaxChange, double turnKP, double turnKD, double turnDegrees, double turnStartPoint) { // revert for skills
  
  resetEnc(); // resets the Enc 
  Inertial_reset(); // reset inertial sensors

  //Error// 
  double error = goal - avgEnc();
  double turnError = turnDegrees - Inertail_rotation();
  //Previous Error//
  double prevError = 0; 
  double prevTurnError = 0;
  //Derivative//
  double derivative;
  double turnDerivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;
  double prevMotorPower = 0;
  double leftSubtraction = 0;



  while (error > 3) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/

      error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      turnError = turnDegrees - Inertail_rotation();
      turnDerivative = turnError - prevTurnError;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }

      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
      LB.spin(forward, lateralmotorpower - leftSubtraction, pct);
      LF.spin(forward, lateralmotorpower - leftSubtraction, pct);
      RB.spin(forward, lateralmotorpower, pct);
      RF.spin(forward, lateralmotorpower, pct);

      prevError = error;
      prevTurnError = turnError;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);
      }
      

  DriveBreak();
}
void forwardiS(){
  LB.stop();
  RB.stop();
  LF.stop();
  RF.stop();
  IntakeL.stop();
  IntakeR.stop();
  TopIndexer.stop();
  BottomIndexer.stop();
}




////////////////////////////////////////////////////////////////
// Modified forward intake PD for intentional motor slippage
////////////////////////////////////

void SlipForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange, double degree){ // revert for skills
  
  resetEnc(); // resets the Enc
  Inertial_reset();
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;
  double prevMotorPower = 0;
  bool stop = false;
  int counter = 0;



  while (error > 3 && counter < 150) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }

  
        IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
        IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
        BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
        TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
        if (Inertail_rotation() < degree && !stop) {
          LB.spin(fwd, 70, pct);
          LF.spin(fwd, 70, pct);
        } else {
          LB.spin(fwd, lateralmotorpower, pct);
          LF.spin(fwd, lateralmotorpower, pct);
          stop = true;
        }
        
        RB.spin(forward,lateralmotorpower,pct);
        RF.spin(forward,lateralmotorpower,pct);
      
    
      
      prevError = error;
      prevMotorPower = lateralmotorpower;

      counter++;
      task::sleep(10);
      }
      

  DriveBreak();
}


void SlipForwardIntakePDR(double goal, float KP,float KI,float KD, double slewMaxChange, double degree){ // revert for skills
  
  resetEnc(); // resets the Enc
  Inertial_reset();
  //Error// 
  double error = goal - avgEnc();
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;
  double prevMotorPower = 0;
  bool stop = false;
  int counter = 0;



  while (error > 3 && counter < 150) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }

  
        IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
        IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
        BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
        TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
        if (Inertail_rotation() < degree && !stop) {
          

          RB.spin(forward, 60 ,pct);
          RF.spin(forward, 60 ,pct);
        } else {
          LB.spin(fwd, lateralmotorpower, pct);
          LF.spin(fwd, lateralmotorpower, pct);
          stop = true;
        }

        LB.spin(fwd, lateralmotorpower, pct);
        LF.spin(fwd, lateralmotorpower, pct);
        
        
      
    
      
      prevError = error;
      prevMotorPower = lateralmotorpower;

      counter++;
      task::sleep(10);
      }
      

  DriveBreak();
}


void StrafePD(double goal, float KP,float KI,float KD, double slewMaxChange){ // revert for skills
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - mTrackingWheel.position(deg);
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;
  double prevMotorPower = 0;



  while (error > 3) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - mTrackingWheel.position(deg);
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
    
      //IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      //IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      //BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      //TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
      LB.spin(reverse,lateralmotorpower,pct);
      LF.spin(forward,lateralmotorpower,pct);
      RB.spin(forward,lateralmotorpower,pct);
      RF.spin(reverse,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);
      }
      

  DriveBreak();
}




void distanceForwardIntakePD(double goal, float KP,float KI,float KD, double slewMaxChange){ // revert for skills
  
  resetEnc(); // resets the Enc 
  //Error// 
  double error = goal - ((distanceL.objectDistance(mm) + distanceR.objectDistance(mm))/2);
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;
  double prevMotorPower = 0;



  while (error > 10) {
      /*if (Inertial1_acceleration() < 0 || Inertial2_acceleration() < 0 ) {
      IntakeL.stop();
      IntakeR.stop();
      } else {*/
        error = goal - avgEnc();
      derivative = error - prevError;
      totalerror += error;

      lateralmotorpower = (error * KP + totalerror * KI + derivative * KD);

      if (lateralmotorpower - prevMotorPower > slewMaxChange) {
        lateralmotorpower = prevMotorPower + slewMaxChange;
      }
    
      IntakeR.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      IntakeL.spin(directionType::fwd, 140, vex::velocityUnits::pct);
      BottomIndexer.spin(directionType::rev, 60, vex::velocityUnits::pct);
      TopIndexer.spin(directionType::rev, 40, vex::velocityUnits::pct);
      LB.spin(forward,lateralmotorpower,pct);
      LF.spin(forward,lateralmotorpower,pct);
      RB.spin(forward,lateralmotorpower,pct);
      RF.spin(forward,lateralmotorpower,pct);

      prevError = error;
      prevMotorPower = lateralmotorpower;
      task::sleep(10);
      }
      
  IntakeL.stop();
  IntakeR.stop();
  DriveBreak();
}


//////////////////////////////////////////
// Turn left PD
/////////////////////
void distancePD(double degree, float kP,float kD) {
    Inertial_reset();
  
    double error = (Inertail_rotation() - degree);
    double derivative;
    double prevError = 0;
    double TurnPower;

    int range = 1;
    while (fabs(error) > range) {
      error = (fabs(Inertail_rotation() - (degree)));
      derivative = error - prevError;
      TurnPower = (error * kP + derivative * kD);

      //spin leftMotor reverse by error * kP;
      LF.spin(reverse,TurnPower, pct);
      RF.spin(forward,TurnPower, pct);
      LB.spin(reverse,TurnPower, pct);
      RB.spin(forward,TurnPower, pct);
      
      wait(10, msec);
    }
}