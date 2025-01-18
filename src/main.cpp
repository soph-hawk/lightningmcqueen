/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sophie                                                    */
/*    Created:      7/3/2024, 3:23:48 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "auton.h"
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
bool inauton = false;


//auton selector
void describe(int n) {
  Controller1.Screen.setCursor(4, 1);

  if (n == 1) { //tells which auton you have selected on screen
    Controller1.Screen.print("blue right 3");
  } else if (n == 2) {
    Controller1.Screen.print("red left 3");
  } else if (n == 3) {
    Controller1.Screen.print("blue goal rush 1+1");
  } else if (n == 4) {
    Controller1.Screen.print("red goal rush 1+1");
  } else if (n == 5) {
    Controller1.Screen.print("red 4 ring");
  } else if (n == 6) {
    Controller1.Screen.print("blue 4 ring");
  } else if (n == 7) {
    Controller1.Screen.print("goal rush blue");
  } else if (n == 8) {
    Controller1.Screen.print("alliance stake left");
  } else if (n == 9) {
    Controller1.Screen.print("alliance stake right");
  } else if (n == 10) {
    Controller1.Screen.print("goal rush red");
  } else if (n == 11) {
    Controller1.Screen.print("drivePID");
  } else if (n == 12) {
    Controller1.Screen.print("turnPID");
  }
}

int autons = 12;
int displayauton = 0;
void selectorout() {
  while (true) {
    if (Controller1.ButtonRight.pressing()) {
      displayauton++;
      wait(100, msec);
    }
    if (Controller1.ButtonLeft.pressing()) {
      displayauton--;
      wait(100, msec);
    }
    if (Controller1.ButtonA.pressing()) {
      wait(1000, msec);
      if (Controller1.ButtonA.pressing()) {
        Controller1.rumble(rumbleLong);
        break;
      }
    }
    if (displayauton > autons) {
      displayauton = 0;
    }
    if (displayauton < 0) {
      displayauton = autons;
    }
    if (displayauton == 0) {
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Choose Program");
    }
    if (displayauton != 0 && inauton == false) {
      // Autonomous name with number
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("Auton");
      Controller1.Screen.setCursor(1, 6);
      Controller1.Screen.print(displayauton);


      Controller1.Screen.setCursor(4, 1);


      // Autonomous Short Description
      describe(displayauton);
    }
  }
}


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Inertial.calibrate();
  Controller1.Screen.print("DON'T MOVE");


  while (Inertial.isCalibrating()) {
    wait(1000, msec);
  }


  Controller1.Screen.clearScreen();


  selectorout();
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  inauton = true;

  if (displayauton == 1) {
    auton1();
  }
  if (displayauton == 2) {
    auton2();
  }
  if (displayauton == 3) {
    auton3();
  }
  if (displayauton == 4) {
    auton4();
  }
  if (displayauton == 5) {
    auton5();
  }
  if (displayauton == 6) {
    auton6();
  }
  if (displayauton == 7) {
    auton7();
  }
  if (displayauton == 8) {
    auton8();
  }
  if (displayauton == 9) {
    auton9();
  }
  if (displayauton == 10) {
    auton10();
  }
  if (displayauton == 11) {
    auton11();
  }
  if (displayauton == 12) {
    auton12();
  }


  inauton = false;
}



/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  // User control code here, inside the loop
  bool prevMogoDown = false;
  bool mogoDown = false;
  bool prevDoinkerDown = false;
  bool doinkerDown = false;
  bool wallstakesscore = false;
  int intakerun = 0;
  double lspeed = 0;
  double rspeed = 0;
  double kspeed = 6;
  double slowspeed = 3;
  bool drivespeed = false;
  double y = 0;
  double r = 0;


  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    double intakespeed = 0;

    //tank drive control
    lspeed = Controller1.Axis3.position()*kspeed;
    rspeed = Controller1.Axis2.position()*kspeed;

      fl.spin(fwd, lspeed, rpm);
      ml.spin(fwd, lspeed, rpm);
      bl.spin(fwd, lspeed, rpm);
      fr.spin(fwd, rspeed, rpm);
      mr.spin(fwd, rspeed, rpm);
      br.spin(fwd, rspeed, rpm); 

    if(Controller1.ButtonA.pressing()){
      if(drivespeed == false){
        kspeed = 6;
        drivespeed = true;
      } else if (drivespeed == true){
        kspeed = 3;
        drivespeed = false;
      }
    }

//intake
// check the ButtonL1/ButtonL2 status to control intake
   //int variable = 0;
    //bool last = false;

//intake control
  if (Controller1.ButtonR1.pressing()){
    intake1.spin(forward, 90, pct);
    intake2.spin(forward, 90, pct);
  }
  else if (Controller1.ButtonR2.pressing()){
    intake2.spin(reverse, 80, pct);
    intake1.spin(reverse, 80, pct);
  }
  else if (Controller1.ButtonL2.pressing()){
    intake1.spin(fwd, 80, pct);
    intake2.stop();
  }
  else if (Controller1.ButtonL1.pressing()) {
    intake1.stop();
    intake2.stop();
}
/*
  if(Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
    intakespeed = 200;
  }
  if(!Controller1.ButtonR1.pressing() && Controller1.ButtonR2.pressing()){
    intakespeed = -200;
  }
  if(!Controller1.ButtonR2.pressing() && !Controller1.ButtonR1.pressing()){
    intakespeed = 0;
  }
  intake1.spin(fwd, intakespeed, rpm);
  intake2.spin(fwd, intakespeed, rpm); */
     /*
     if (Controller1.ButtonR1.pressing()) {
        intake1.spin(fwd, 200, rpm);
        intake2.spin(fwd, 200, rpm);
       intakerun = false;
     } else if (Controller1.ButtonR2.pressing()) {
       //intaking
       intake1.spin(reverse, 200, rpm);
       intake2.spin(reverse, 200, rpm);
       intakerun = false;
     } else if (!intakerun) {
       intake1.stop();
       intake2.stop();
       // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
       intakerun = true;
     }
     */
    //mogo
    if (Controller1.ButtonY.pressing()) {
      if (prevMogoDown == false) {
        mogoDown = !mogoDown;
        prevMogoDown = true;
      }
    } else {
      if (prevMogoDown == true) {
        prevMogoDown = false;
      }
    }
    mogo.set(mogoDown);

    //doinker
    if (Controller1.ButtonX.pressing()) {
      if (prevDoinkerDown == false) {
        doinkerDown = !doinkerDown;
        prevDoinkerDown = true;
      }
    } else {
      if (prevDoinkerDown == true) {
        prevDoinkerDown = false;
      }
    }
    doinker.set(doinkerDown);

    //wallstakes load
  if(Controller1.ButtonUp.pressing()) {
    if(Rotational.angle(degrees) < 236 && Rotational.angle(degrees) > 200) {
      while(Rotational.angle(degrees) < 236 && Rotational.angle(degrees) > 200) {
        wallstakes.spin(fwd, 60, pct);
      }
    } else if (Rotational.angle(degrees) > 236 or Rotational.angle(degrees) < 200) {
        while(Rotational.angle(degrees) > 236 or Rotational.angle(degrees) < 200) {
          wallstakes.spin(reverse, 60, pct);
      }
        while(Rotational.angle(degrees) < 237) {
          wallstakes.spin(fwd, 60, pct);
        }
    }
    wallstakes.stop(hold);
  } 
  //Controller1.Screen.print(Rotational.angle(degrees));
  //wall stakes score
  if(Controller1.ButtonB.pressing()){
    wallstakes.spin(fwd, 80, pct);
    intake2.spin(reverse, 90, pct);
    wait(100, msec);
    intake1.stop();
    intake2.stop();
    
  } else if(Controller1.ButtonDown.pressing()){
    wallstakes.spin(reverse, 80, pct);
  } else {
    wallstakes.stop(hold);
  } 
  /* use this after angles are determined
  if(Controller1.ButtonDown.pressing()) {
    if(wallstakesscore == true) {
      while(angle > 170){  //figure out what angle
        wallstakes.spin(fwd, 80, pct);
      }
      wallstakesscore == false;
    } else if (wallstakesscore == false){
      while(angle != 0){ //figure out what angle
        wallstakes.spin(reverse, 90, pct);
        wallstakesscore == true;
      }
    }
  } */

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

