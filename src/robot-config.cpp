#include "vex.h"



using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain Brain;


// VEXcode device constructors
controller Controller1 = controller(primary);
motor fl = motor(PORT2, ratio6_1, true);
motor ml = motor(PORT7, ratio6_1, true);
motor bl = motor(PORT3, ratio6_1, true);
motor fr = motor(PORT20, ratio6_1, false);
motor mr = motor(PORT19, ratio6_1, false);
motor br = motor(PORT4, ratio6_1, false);

motor intake2 = motor(PORT15, ratio18_1, true);
motor intake1 = motor(PORT14, ratio18_1, false);
motor wallstakes = motor(PORT8, ratio18_1, false);

rotation Rotational = rotation(PORT21, false);
inertial Inertial = inertial(PORT13);
optical colorsort = optical(PORT18);

pneumatics mogo = pneumatics(Brain.ThreeWirePort.E);
pneumatics doinker = pneumatics(Brain.ThreeWirePort.F);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;


/**
* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
*
* This should be called at the start of your int main function.
*/
void vexcodeInit( void ) {


}


