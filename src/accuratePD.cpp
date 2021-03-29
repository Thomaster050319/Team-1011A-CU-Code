#include "vex.h"
#include "AutoFunctions.h"

double encoder1Pos() {
  return encoder1.position(deg);
}

void encoder1Reset() {
  encoder1.setPosition(0, degrees);
}

void accurateForwardIntakePD(double goal, float KP,float KI,float KD){
  
  encoder1Reset(); // resets the Enc 
  //Error// 
  double error = goal - encoder1Pos() + 1;
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  //lateral motor power//
  double lateralmotorpower;

  while (error != 0) {
      error = goal - encoder1Pos();
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
void accurateDriftPD(double goal, float KP,float KI,float KD){
  
  encoder1Reset(); // resets the Enc 
  //Error// 
  double error = goal - encoder1Pos() + 1;
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  int hitCount = 0;

  while (error != 0){
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
      error = goal - encoder1Pos();
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
void accurateForwardOutakePD(double goal, float KP,float KI,float KD){
  
  encoder1Reset(); // resets the Enc 
  //Error// 
  double error = goal - encoder1Pos() + 1;
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error != 0){
    if (hitDetected()) {
      vexDelay(50);
      resetEnc();
      error = 0;
      goal = 0;
      prevError = 0;
      derivative = 0;
      totalerror = 0;

    } else {
    error = goal - encoder1Pos();
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
void accurateForwardPD(double goal, float KP,float KI,float KD){
  
  encoder1Reset(); // resets the Enc 
  //Error// 
  double error = goal - encoder1Pos() + 1;
  //Previous Error//
  double prevError = 0; 
  //Derivative//
  double derivative;
  double totalerror;
  //lateral motor power//
  double lateralmotorpower;

  while (error != 0){
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
      error = goal - encoder1Pos();
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
void accurateBackwardPD(double goal, float KP, float KD){
  encoder1Reset();
  double error = goal - encoder1Pos() + 1;
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error != 0) { 
      error = goal - encoder1Pos();
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
void accurateBackwardAlignPD(double goal, float KP, float KD){
  encoder1Reset();
  double error = goal - encoder1Pos() + 1;
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error != 0) { 
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
      error = goal - encoder1Pos();
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
void accurateBackwardOPD(double goal, float KP, float KD){
  encoder1Reset();
  double error = goal - encoder1Pos() + 1;
  double prevError = 0; 
  double derivative;
  double lateralmotorpower;

  while(error != 0) { 
    error = goal - encoder1Pos();
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
