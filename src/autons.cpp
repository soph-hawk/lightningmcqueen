
#include "vex.h"
#include "auton.h"
#include <string>

using namespace vex;
using namespace std;


// Drivetrain PID
double drivekp = 0.25;
double driveki = 0;
double drivekd = 0;

double turnkp = 2.7;
double turnki = 0;
double turnkd = 0.7;

// Drivetrain PID Function
void drivePID(double targetdegrees) {
    Inertial.setRotation(0, degrees);
    double error = targetdegrees;
    double integral = 0;
    double lasterror = error;
    double lspeed;
    double rspeed;
    double prevdegrees = fl.position(degrees);
    double startrotation = Inertial.rotation(degrees);
    double rotdif = 0;
    double krotdif = 0;


    double count = 0;


    fl.setPosition(0, degrees);
    bl.setPosition(0, degrees);
    fr.setPosition(0, degrees);
    br.setPosition(0, degrees);
    ml.setPosition(0, degrees);
    mr.setPosition(0, degrees);

    while(true) {
        double measureddegrees = (fl.position(degrees) + fr.position(degrees)) / 2;
        error = targetdegrees - measureddegrees;

        if(fabs(measureddegrees - prevdegrees) < 3){
            count++; //add to count
        } else { //if not being stalled
            count = 0;
        }

        if (count > 10) { //exit when stuck for 200 ms
            fl.stop(brake);
            fr.stop(brake);
            ml.stop(brake);
            mr.stop(brake);
            br.stop(brake);
            bl.stop(brake);
            return;
        }

        prevdegrees = measureddegrees;

        //Integral windup
        if (fabs((error) < targetdegrees / 10*3 && fabs(integral) < 300)) {
            integral += error;
        }

        if(fabs(error) < 30) {
            fl.stop(brake);
            fr.stop(brake);
            ml.stop(brake);
            mr.stop(brake);
            br.stop(brake);
            bl.stop(brake);
            return;
        }

        lspeed = error * drivekp + integral * driveki +(error - lasterror) * drivekd; //motor speed
        rspeed = error * drivekp + integral * driveki +(error - lasterror) * drivekd; //motor speed

        rotdif = Inertial.rotation(degrees) - startrotation;
        krotdif = rotdif * 4;

        if(fabs(rotdif) <= 1) { //go straight
            fl.spin(fwd, lspeed, rpm);
            ml.spin(fwd, lspeed, rpm);
            bl.spin(fwd, lspeed, rpm);
            fr.spin(fwd, rspeed, rpm);
            mr.spin(fwd, rspeed, rpm);
            br.spin(fwd, rspeed, rpm);
        } else {
            fl.spin(fwd, lspeed - krotdif, rpm);
            ml.spin(fwd, lspeed - krotdif, rpm);
            bl.spin(fwd, lspeed - krotdif, rpm);
            fr.spin(fwd, rspeed + krotdif, rpm);
            mr.spin(fwd, rspeed + krotdif, rpm);
            br.spin(fwd, rspeed + krotdif, rpm);
        }

        lasterror = error;
        wait(20, msec);
    }
}

// turnPID
void turnPID(double targetheading) {
    Inertial.setRotation(0, degrees);
    double currentheading = Inertial.heading(degrees);
    double turndegrees = 0;

    if(targetheading - currentheading > 180) { //if turning left past 0
        turndegrees = (targetheading - currentheading) - 360;
    } else if (targetheading - currentheading < -180) { //if turning right past 360
        turndegrees = (targetheading - currentheading) +360;
    } else { //normal turn
        turndegrees = targetheading - currentheading;
    }

    double error = turndegrees;
    double integral = 0;
    double lasterror = error;
    double speed = 0;

    double prevdeg = Inertial.rotation(degrees);
    double currentdeg = prevdeg;

    double count = 0;

    while (fabs(error) > 2) {
        error = turndegrees - Inertial.rotation(degrees);

        integral += error;

        if (fabs(currentdeg - prevdeg) < 0.3) { // add to count if stalled
            count++;
        } else {
            count = 0;
        }

        if (fabs(error) > 2 && turndegrees <= 90) {
            if(count==50) {
                fl.stop(brake);
                fr.stop(brake);
                ml.stop(brake);
                mr.stop(brake);
                br.stop(brake);
                bl.stop(brake);
                return;
            }
        }
        if (fabs(error) < 2 && turndegrees > 90) {
            if(count == 30) {
                fl.stop(brake);
                fr.stop(brake);
                ml.stop(brake);
                mr.stop(brake);
                br.stop(brake);
                bl.stop(brake);
                return;
            }
        }

        speed = error * turnkp + integral * turnki + (error - lasterror) * turnkd;

        fl.spin(fwd, speed, rpm);
        ml.spin(fwd, speed, rpm);
        bl.spin(fwd, speed, rpm);
        fr.spin(fwd, -speed, rpm);
        mr.spin(fwd, -speed, rpm);
        br.spin(fwd, -speed, rpm);

        lasterror = error;
        wait(20, msec);
    }
    fl.stop(brake);
    fr.stop(brake);
    ml.stop(brake);
    mr.stop(brake);
    br.stop(brake);
    bl.stop(brake);
    return;

}

//red stack side 3+1
void auton1() {
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    //mogo
    drivekp = 0.4;
    drivePID(-800);
    turnPID(30);
    drivekp = 0.3;
    drivePID(-500);
    mogo.set(false);
    //stack ring
    drivekp = 0.6;
    turnPID(139);
    intake2.spin(fwd, 80, pct);
    intake1.spin(fwd, 80, pct);
    drivePID(970);
    wait(400, msec);
    //3rd ring
    drivePID(-300);
    turnPID(118);
    drivekp = 0.6;
    drivePID(500);
    wait(500, msec);
    //protected ring
    drivekp = 0.6;
    drivePID(-1100);
    turnPID(79);
    drivePID(700);
    wait(500, msec);
    //preload
    turnPID(-5);
    drivePID(1300);
    wait(700, msec);
    intake2.stop();
    mogo.set(true);
    turnPID(-90);
    drivePID(1600);
    //mogo.set(true);
    turnPID(-182);
    drivePID(-500);
    intake1.stop();
    intake2.spin(fwd, 90, pct);
    doinker.set(true);
    wait(800, msec);
    drivePID(1200);


}
//blue stack side 3+1
void auton2() {
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    //mogo
    drivekp = 0.4;
    drivePID(-800);
    turnPID(-30);
    drivekp = 0.3;
    drivePID(-500);
    mogo.set(false);
    //stack ring
    drivekp = 0.6;
    turnPID(-144);
    intake2.spin(fwd, 80, pct);
    intake1.spin(fwd, 80, pct);
    drivePID(970);
    wait(400, msec);
    //3rd ring
    drivePID(-300);
    turnPID(-120);
    drivekp = 0.6;
    drivePID(500);
    wait(500, msec);
    //protected ring
    drivekp = 0.6;
    drivePID(-1100);
    turnPID(-74);
    drivePID(600);
    wait(500, msec);
    //preload
    turnPID(5);
    drivePID(1300);
    intake2.stop();
    mogo.set(true);
    turnPID(90);
    drivePID(1600);
    mogo.set(false);
    turnPID(180);
    drivePID(-500);
    intake1.stop();
    intake2.spin(fwd, 90, pct);
    doinker.set(true);
    wait(800, msec);
    drivePID(1200);

}
//blue goal rush 1+1
void auton3() {
    //set up mogo support with closest ridge to red ring
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    drivekp = 0.4;
    //nuetral mogo
    drivePID(-1600);
    turnPID(33);
    drivekp = 0.3;
    drivePID(-700);
    mogo.set(false);
    turnPID(-10);
    //second ring
    drivekp = 0.5;
    drivePID(400);
    intake1.spin(fwd, 90, pct);
    intake2.spin(fwd, 90, pct);
    wait(500, msec);
    intake2.stop();
    wait(500, msec);
    //next mogo
    drivePID(750);
    intake1.stop();
    turnPID(-180);
    mogo.set(true);
    turnPID(-10);
    drivePID(-200);
    turnPID(90);
    drivekp = 0.3;
    drivePID(-1500);
    mogo.set(false);
    wait(300, msec);
    intake1.spin(fwd, 80, pct);
    intake2.spin(fwd, 80, pct);
    turnPID(-90);
    drivekp = 0.5;
    drivePID(500);
}
//red goal rush 1+1
void auton4() {
    //set up mogo support with closest ridge to red ring
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    drivekp = 0.4;
    //nuetral mogo
    drivePID(-1600);
    turnPID(-33);
    drivekp = 0.3;
    drivePID(-660);
    mogo.set(false);
    //second ring
    drivekp = 0.5;
    intake1.spin(fwd, 90, pct);
    intake2.spin(fwd, 90, pct);
    wait(700, msec);
    intake2.spin(reverse, 70, pct);
    turnPID(10);
    intake2.stop();
    drivePID(400);
    wait(500, msec);
    //next mogo
    drivePID(750);
    intake1.stop();
    turnPID(180);
    mogo.set(true);
    turnPID(10);
    drivePID(-200);
    turnPID(-90);
    drivekp = 0.3;
    drivePID(-1400);
    mogo.set(false);
    intake1.spin(fwd, 90, pct);
    intake2.spin(fwd, 90, pct);
    wait(300, msec);
    turnPID(90);
    drivekp = 0.5;
    drivePID(500);

}
//4 ring red
void auton5() {
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    //mogo
    drivekp = 0.4;
    drivePID(-800);
    turnPID(30);
    drivekp = 0.3;
    drivePID(-500);
    mogo.set(false);
    //stack ring
    drivekp = 0.6;
    turnPID(139);
    intake1.spin(fwd, 80, pct);
    drivePID(970);
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    //3rd ring
    drivePID(-300);
    turnPID(118);
    drivekp = 0.6;
    drivePID(500);
    wait(500, msec);
    //protected ring
    drivekp = 0.6;
    drivePID(-1200);
    turnPID(79);
    drivePID(700);
    wait(1200, msec);
    turnPID(-90);
    drivePID(1300);
}
//4 ring blue
void auton6() {
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    //mogo
    drivekp = 0.4;
    drivePID(-800);
    turnPID(-30);
    drivekp = 0.3;
    drivePID(-500);
    mogo.set(false);
    //stack ring
    drivekp = 0.6;
    turnPID(-139);
    intake1.spin(fwd, 80, pct);
    drivePID(1000);
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    //3rd ring
    drivePID(-300);
    turnPID(-122);
    drivekp = 0.6;
    drivePID(500);
    wait(500, msec);
    //protected ring
    drivekp = 0.6;
    drivePID(-1200);
    turnPID(-80);
    drivePID(700);
    wait(1200, msec);
    turnPID(86);
    drivePID(1450);
}

//3+1 red
void auton7() {
    Inertial.setHeading(0, degrees);
    mogo.set(true);
    //preload on mogo
    drivekp = 0.4;
    drivePID(-800);
    turnPID(-30);
    drivekp = 0.3;
    drivePID(-500);
    mogo.set(false);
    //stack ring
    drivekp = 0.5;
    turnPID(134);
    intake2.spin(fwd, 90, pct);
    intake1.spin(fwd, 90, pct);
    drivePID(1100);
    wait(400, msec);
    //protected ring
    drivekp = 0.6;
    drivePID(-400);
    turnPID(72);
    drivePID(500);
    wait(300, msec);
    //drop bottom blue
    turnPID(-35);
    drivePID(1700);
    turnPID(-90);
    mogo.set(true);
    drivePID(900);
    wait(500, msec);
    intake2.stop();
    //intake red
    drivePID(600);
    wait(300, msec);
    drivePID(-1000);
    //alliance stake and bar touch
    doinker.set(false);
    turnPID(30);
    turnPID(175);
    doinker.set(true);
    drivekp = 0.4;
    drivePID(-700);
    intake2.spin(fwd, 90, pct);
    drivekp = 0.5;
    wait(500, msec);
    drivePID(1200);

}

//alliance stake blue left
void auton8() {
    Inertial.setHeading(0, degrees);
    //alliance stake
    drivekp = 0.4;
    drivePID(-750);
    turnPID(90);
    drivePID(-450);
    intake2.spin(fwd, 90, pct);
    wait(500, msec);
    intake2.spin(reverse, 90, pct);
    wait(200, msec);
    //mogo
    drivePID(450);
    intake2.stop();
    turnPID(0);
    drivePID(500);
    turnPID(-123);
    drivekp = 0.25;
    drivePID(-1900);
    mogo.set(true);
    //ring
    turnPID(-2);
    drivekp = 0.4;
    intake1.spin(fwd, 90, pct);
    intake2.spin(fwd, 90, pct);
    drivePID(400);
    wait(800, msec);
    drivePID(-300);
    //bar touch
    //turnPID(180);
    //drivePID(1400);
}
//blue alliance stake
void auton9() {
    Inertial.setHeading(0, degrees);
    //alliance stake
    drivekp = 0.4;
    drivePID(-700);
    turnPID(-90);
    drivePID(-450);
    intake2.spin(fwd, 90, pct);
    wait(500, msec);
    intake2.spin(reverse, 90, pct);
    wait(200, msec);
    //mogo
    drivePID(450);
    intake2.stop();
    turnPID(0);
    drivePID(500);
    turnPID(120);
    drivekp = 0.25;
    drivePID(-1900);
    mogo.set(true);
    //ring
    turnPID(0);
    intake1.spin(fwd, 90, pct);
    intake2.spin(fwd, 90, pct);
    drivekp = 0.4;
    drivePID(400);
    wait(700, msec);
    drivePID(-300);
    //bar touch
    //turnPID(180);
    //drivePID(1400);
}

//goal rush red
void auton10() {
    Inertial.setHeading(0, degrees);
    //alliance stake
    drivekp = 0.6;
    turnPID(-17);
    intake1.spin(fwd, 80, pct);
    drivePID(2000);
    doinker.set(true);
    drivekp = 0.4;
    wait(300, msec);
    drivePID(-800);
    doinker.set(false);
    turnPID(175);
    mogo.set(true);
    drivekp = 0.3;
    drivePID(-800);
    mogo.set(false);
    wait(200, msec);
    drivekp = 0.5;
    intake1.stop();
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    intake2.stop();
    turnPID(0);
    drivePID(-700);
    mogo.set(true);
    drivePID(200);
    turnPID(90);
    drivekp = 0.3;
    drivePID(-1400);
    mogo.set(false);
    wait(300, msec);
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    turnPID(-90);
    drivePID(700);

}
//blue goal rush
void auton11() {
    Inertial.setHeading(0, degrees);
    //alliance stake
    drivekp = 0.6;
    turnPID(-17);
    intake1.spin(fwd, 80, pct);
    drivePID(2000);
    doinker.set(true);
    drivekp = 0.4;
    wait(300, msec);
    drivePID(-800);
    doinker.set(false);
    turnPID(-175);
    mogo.set(true);
    drivekp = 0.3;
    drivePID(-800);
    mogo.set(false);
    wait(200, msec);
    drivekp = 0.5;
    intake1.stop();
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    intake2.stop();
    turnPID(0);
    drivePID(-700);
    mogo.set(true);
    drivePID(200);
    turnPID(-90);
    drivekp = 0.3;
    drivePID(-1400);
    mogo.set(false);
    wait(300, msec);
    intake2.spin(fwd, 80, pct);
    wait(400, msec);
    turnPID(90);
    drivePID(700);
}

void auton12() {
    //auton5
    turnPID(180);
}


