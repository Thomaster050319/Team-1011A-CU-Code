/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       skills2.cpp                                               */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

using namespace vex;

void skills2(){
  ForwardIntakePD(1050,0.20,0,0.1);
  vexDelay(100);
  TurnLeftPD(140,0.6,0.1);
  forwardintakestop();
  vexDelay(200);
  ForwardPD(1180,0.4,0,0.1);
  shoot(350);
  vexDelay(150);
  BackwardPD(650,0.25,0.1);
  vexDelay(200);
  TurnRightPD(145,0.9,0.1);
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