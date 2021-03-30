/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RED_BALL = vex::vision::signature (1, 5335, 9417, 7376, -1489, 231, -629, 2, 0);
vex::vision::signature BLUE_BALL = vex::vision::signature (2, -3163, -1873, -2518, 7657, 10559, 9108, 3, 0);
vex::vision::signature GOAL_CORNER = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision Vision1 = vex::vision (vex::PORT16, 50, RED_BALL, BLUE_BALL, GOAL_CORNER, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/