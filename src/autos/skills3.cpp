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

void skills3(){

  ForwardIntakePD(970,0.20,0,0.3);
  vexDelay(200);
  TurnLeftPD(140,0.75,0.3);
  forwardintakestop();
  vexDelay(150);
  ForwardPD(1240,0.4,0,0.3);
  shoot(350);


  vexDelay(300);
  BackwardPD(580,0.25,0.1);
  vexDelay(500);
  TurnRightPD(141,0.9,0.1);
  vexDelay(300);
  ForwardIntakePD(1780,0.25,0,0.3);
  forwardintakestop();
  TurnLeftPD(89,0.9,0.1);
  ForwardPD(300,0.3,0,0.1); //2nd goal forward
  insuck(200);
  shoot(300);


  vexDelay(200);
  BackwardPD(800,0.25,0.1);
  TurnRightPD(64,0.7,0.1);
  vexDelay(200);
  ForwardPD(2320,0.27,0,0.1);
  insuck(200);
  shoot(600); // 3rd goal


  vexDelay(150);
  ///////// 2nd row /////////
  BackwardPD(380,0.3,0.1);
  vexDelay(150);
  TurnRightPD(115,0.7,0.1); //3rd to 4th angle 
  ForwardIntakePD(2000,0.30,0,0.1);//3rd to 4th transition
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(390,0.3,0,0.1); //4th goal forward
  shoot(400);


  vexDelay(100);
  BackwardPD(340,0.3,0.1);
  vexDelay(150);
  TurnRightPD(90,0.8,0.1);
  vexDelay(150);
  ForwardIntakePD(1830,0.3,0,0.1);
  forwardintakestop();
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(1100,0.3,0,0.1);
  shoot(400);


  vexDelay(150);
  BackwardPD(560,0.3,0.1);
  vexDelay(200);
  TurnRightPD(129, 0.8, 0.3);
  ForwardIntakePD(1790, 0.27, 0, 0.1);
  TurnLeftPD(92,0.9,0.1);
  ForwardPD(350,0.3,0,0.1);
  shoot(400);


  BackwardPD(335,0.3,0.1);
  TurnRightPD(60,0.8,0.1);
  ForwardIntakePD(2100, 0.3, 0, 0.1);
  shoot(400);


 /*///////// 3rd row /////////
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