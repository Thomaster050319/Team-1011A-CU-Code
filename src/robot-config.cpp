#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

//🔥  𝐂𝐡Ａᔕ𝒾𝕤 𝐜𝑜ᑎｆ𝐈Ǥ  😝★//
motor LB = motor(PORT16,ratio6_1, true); // 16 (Port #)
motor RB = motor(PORT9,ratio6_1, false); // 9
motor LF = motor(PORT11,ratio6_1, true); // 11
motor RF = motor(PORT19,ratio6_1, false); // 19

//𝕾𝖚𝖇𝖘𝖞𝖘𝖙𝖊𝖒 𝕮𝖔𝖓𝖋𝖎𝖌//
motor IntakeR = motor(PORT18,ratio6_1, true); // 18
motor IntakeL = motor(PORT20,ratio6_1, false); // 20

motor TopIndexer = motor(PORT2,ratio6_1, true); // 2
motor BottomIndexer = motor(PORT1, ratio6_1, false); // 1

//𝕾𝖊𝖓𝖘𝖔𝖗𝖘 𝕮𝖔𝖓𝖋𝖎𝖌//
inertial inertial1 = inertial(PORT7); // 7
inertial inertial2 = inertial(PORT10); // 10
inertial inertial3 = inertial(PORT17); // 17
distance distanceL = distance(PORT3); // 3
distance distanceR = distance(PORT4); // 4
optical intakeOptical = optical(PORT15); // 15
//accelerometer accelY = accelerometer(Brain.ThreeWirePort.E); //E

//limit limit1 = limit(Brain.ThreeWirePort.D); // D

encoder rTrackingWheel = encoder(Brain.ThreeWirePort.A); // Ports A and B
encoder lTrackingWheel = encoder(Brain.ThreeWirePort.C); // Ports C and D
encoder mTrackingWheel = encoder(Brain.ThreeWirePort.E); // Ports E and F

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
/*
pros::Motor LF (11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // Left Front Port 11
pros::Motor LB (16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // Left Back Port 16
pros::Motor RF (19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // Right Front Port 19
pros::Motor RB (9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // Right Back Port 17

// intakes
pros::Motor lIntake (20, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES); // Left intake Port 20
pros::Motor rIntake (18, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES); // right intake port 18

// indexers
pros::Motor bIndexer (1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // bottom indexer Port 1
pros::Motor tIndexer (2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // top indexer Port 2

// tracking wheels
pros::ADIEncoder lTrackingWheel('C', 'D', true); // Left tracking wheel
pros::ADIEncoder rTrackingWheel('A', 'B', true);
pros::ADIEncoder mTrackingWheel('E', 'F', false);

// sensors
pros::Distance lDist (6); // left intake distance sensor Port 6
pros::Distance rDist (5); // Right intake distance sensor Port 5
pros::Imu imu1 (7); // imu 1 Port 7
pros::Imu imu2 (10); // imu 2 Port 10
pros::Imu imu3 (17); // imu 3 port 17
*/