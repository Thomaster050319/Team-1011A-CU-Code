#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

//🔥  𝐂𝐡Ａᔕ𝒾𝕤 𝐜𝑜ᑎｆ𝐈Ǥ  😝★//
motor LB = motor(PORT2,ratio18_1, false); // 2 (Port #)
motor RB = motor(PORT21,ratio18_1, true); // 21
motor LF = motor(PORT19,ratio18_1, false); // 15
motor RF = motor(PORT15,ratio18_1, true); // 19

//𝕾𝖚𝖇𝖘𝖞𝖘𝖙𝖊𝖒 𝕮𝖔𝖓𝖋𝖎𝖌//
motor IntakeR = motor(PORT14,ratio6_1, true); // 14
motor IntakeL = motor(PORT20,ratio6_1, false); // 20

motor TopIndexer = motor(PORT13,ratio6_1, true); // 13
motor BottomIndexer = motor(PORT1, ratio6_1, false); // 1

//𝕾𝖊𝖓𝖘𝖔𝖗𝖘 𝕮𝖔𝖓𝖋𝖎𝖌//
inertial inertial1 = inertial(PORT11); // 11
inertial inertial2 = inertial(PORT3); // 3

bumper bumperLeft = bumper(Brain.ThreeWirePort.A); // A
bumper bumperRight = bumper(Brain.ThreeWirePort.B); // B
accelerometer accel1 = accelerometer(Brain.ThreeWirePort.C, false); // C
limit limit1 = limit(Brain.ThreeWirePort.D); // D
encoder encoder1 = encoder(Brain.ThreeWirePort.E); // E and F


//𝕮𝖔𝖓𝖙𝖗𝖔𝖑𝖑𝖊𝖗 𝕮𝖔𝖓𝖋𝖎𝖌//
controller Controller1 = controller(primary);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}