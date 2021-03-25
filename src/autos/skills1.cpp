/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       skills1.cpp                                               */
/*    Author:       Thomas Lee                                                */
/*    Created:      Wed Mar 24 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "global.h"

using namespace vex;

void skills1(){ // Left = - Right = +
//////////1st row ///////////
  ForwardIntakePD(1000,0.25,0,0.1);
  vexDelay(250);
  TurnLeftPD(138,0.8,0.1);
  forwardintakestop();
  vexDelay(300);
  ForwardPD(1130,0.25,0,0.1);
  shoot(300); // 1st goal 


  //stopball();
  vexDelay(300);
  BackwardPD(590,0.2,0.1);
  //stopshoot();
  vexDelay(400);
  TurnRightPD(138,0.9,0.1);
  vexDelay(400);
  ForwardIntakePD(1650,0.25,0,0.1);
  forwardintakestop();
  TurnLeftPD(90,0.7,0.1);
  ForwardPD(335,0.2,0,0.1);
  shoot(400); // 2nd goal


  //stopball();
  vexDelay(300);
  BackwardPD(295,0.2,0.1);
  //stopshoot();
  TurnRightPD(90, 0.7, 0.1);
  ForwardPD(1700, 0.3, 0, 0.1);
  TurnLeftPD(45,0.7,0.1);
  vexDelay(200);
  ForwardPD(380,0.25,0,0.1);
  shoot(400); // 3rd goal


  vexDelay(150);
  ///////// 2nd row /////////
  BackwardPD(380,0.3,0.1);
  TurnRightPD(135,0.7,0.1);
  ForwardIntakePD(2075,0.30,0,0.1);
  forwardintakestop();
  TurnLeftPD(90,0.9,0.1);
  ForwardPD(335,0.3,0,0.1);
  shoot(400); // 4th goal


  vexDelay(100);
  BackwardPD(290,0.3,0.1);
  TurnRightPD(94,0.8,0.1);
  vexDelay(100);
  ForwardIntakePD(1700,0.3,0,0.1);
  forwardintakestop();
  TurnLeftPD(45,0.8,0.1);
  ForwardPD(580,0.3,0,0.1);
  shoot(400);
  BackwardPD(580, 0.3, 0.1);


  /*///////// 3rd row /////////
  TurnRightPD(161, 0.8, 0.1);
  ForwardIntakePD(1700, 0.3, 0, 0.1); 
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