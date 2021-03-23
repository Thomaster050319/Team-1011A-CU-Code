using namespace vex;


///Brain Config///
extern brain Brain;


///Chasis Config///
extern motor RB;
extern motor LB; 
extern motor RF;
extern motor LF;

///Subsystem Config///
extern motor IntakeR;
extern motor IntakeL;

extern motor TopIndexer;
extern motor BottomIndexer;


///Sensors Config///
extern inertial inertial1;
extern inertial inertial2;

//Controller Config//
extern controller Controller1;
extern limit limit1;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
