/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RED_BALL = vex::vision::signature (1, 7957, 11989, 9973, -1567, -227, -897, 3.1, 0);
vex::vision::signature BLUE_BALL = vex::vision::signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature GOAL = vex::vision::signature (3, -4681, -3383, -4032, -5691, -3787, -4739, 2.6, 0);
vex::vision::signature PURE_RED = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7= vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT17, 50, RED_BALL, BLUE_BALL, GOAL, PURE_RED, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/