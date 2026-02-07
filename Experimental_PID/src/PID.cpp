#include "vex.h"

//New re-structured PID that utilizes class usage (Still in development):
//PID_Tag is used to find all areas to add onto for additional PID with Ctrl F
//PID_Tag is in the return type functions file

//Variable declaration:
double error;
double integral;
double derivative;


double prevError;
double pwr;
double prevPwr;
double dt;
double prevTime;
double storedTrackingMeasurements;
double resetCurrentPosition;
double storedHeading;
double startTimer;
double wheelRad = 1.0;
double acceleration;
double prevDerivative;

double minimum;
double maximum;

//Deriviative filter constant for high frequency gain
double kTime = 0.0;
double turnKTime = 0.0;


double rampRate = 1;

double integralCap = 30;

double output;

//Tuning constants:
double kP = 0.1;
double kI;
double kD;

double turnKP;
double turnKI;
double turnKD;

double driveAlpha;
double turnAlpha;

double beta;

//The constant that sets the constant speed of the untuned PID to the desired velocity for feed forward:
double kV = 0.0;
double turnKV;

//The constant that sets the acceleration of the feed forward:
double kA = 0.0;
double turnKA = 0.0;

//The constant that overcomes friction for feed forward:
double kS = 0.0;
double turnKS = 0.0;


double feedBack;


//Other tuning constants above <PID_Tag>:

double driveTolerance = 0.1;
double turnTolerance = 0.1;

//Other tolerances above <PID_Tag>:

const char* driveID = "Drive";
const char* turnID = "Turn";

//Other IDs above <PID_Tag>:

double returnSpeed;
double returnRate;

double slewLimit = 20;



PID::PID(double P, double I, double D, double TurnP, double TurnI, double TurnD) {
    p = P;
    i = I;
    d = D;
    turnP = TurnP;
    turnI = TurnI;
    turnD = TurnD;
}

//Turn PID:
void PID::turnToHeading(double desiredValue, double vel) {
    //Turn PID loop with proper fwd/rev directional math and drivetrain motor control:
    storeValues();
    resetID();
    while (true) {
        output = PID_math(desiredValue, turnID, turnP, turnI, turnD, vel);//output + feedForward
        LeftDriveSmart.spin(fwd, output, pct);
        RightDriveSmart.spin(reverse, output, pct);
        if (error >= -turnTolerance && error <= turnTolerance) break;
        prevUpdate();
        wait (10, msec);
    }
    stopMotors();
}



//Drive PID loop with proper fwd/fwd directional math and drivetrain motor control:
void PID::drive(double desiredValue, double vel) {
    storeValues();
    resetID();
    while (true) {
        resetCurrentPosition = ((frontTracking.position(turns)) * (wheelRad * 2) * M_PI) - storedTrackingMeasurements;
        output = PID_math(desiredValue, driveID, p, i, d, vel);
        if (std::round(error) == 6) {
            printAtTop(Drivetrain.velocity(pct));
        }
        LeftDriveSmart.spin(fwd, output + constrainAngle(storedHeading - Inertial1.heading(deg)) * 0.1, pct);
        RightDriveSmart.spin(fwd, output - constrainAngle(storedHeading - Inertial1.heading(deg)) * 0.1, pct);
        if (error >= -driveTolerance && error <= driveTolerance) break;
        prevUpdate();
        wait (10, msec);
    }
    stopMotors();
}

//Turn PID with timer:
void PID::turnToHeading(double desiredValue, double vel, double timePeriod) {
    //Turn PID loop with proper fwd/rev directional math and drivetrain motor control:
    storeValues();
    resetID();
    while (true) {
        output = PID_math(desiredValue, turnID, turnP, turnI, turnD, vel);
        LeftDriveSmart.spin(fwd, output, pct);
        RightDriveSmart.spin(reverse, output, pct);
        if (Brain.timer(sec) >= timePeriod) break;
        prevUpdate();
        wait (10, msec);
    }
    stopMotors();
}

//Drive PID loop with proper fwd/fwd directional math and drivetrain motor control with timer:
void PID::drive(double desiredValue, double vel, double timePeriod) {
    storeValues();
    resetID();
    while (true) {
        resetCurrentPosition = ((frontTracking.position(turns)) * (wheelRad * 2) * M_PI) - storedTrackingMeasurements;
        output = PID_math(desiredValue, driveID, p, i, d, vel);
        LeftDriveSmart.spin(fwd, output + constrainAngle(storedHeading - Inertial1.heading(deg)) * 0.1, pct);
        RightDriveSmart.spin(fwd, output - constrainAngle(storedHeading - Inertial1.heading(deg)) * 0.1, pct);
        if (Brain.timer(sec) >= timePeriod) break;
        prevUpdate();
        wait (10, msec);
    }
    stopMotors();
}


void PID::driveWithPiston(double desiredValue, double vel, double deployRange) {
    storeValues();
    resetID();
    while (true) {
        resetCurrentPosition = ((frontTracking.position(turns)) * (wheelRad * 2) * M_PI) - storedTrackingMeasurements;
        output = PID_math(desiredValue, driveID, p, i, d, vel);
        LeftDriveSmart.spin(fwd, output, pct);
        RightDriveSmart.spin(fwd, output, pct);
        if (error <= deployRange) DescorePiston.set(true);
        if (error >= -driveTolerance && error <= driveTolerance) break;
        printAtTop(frontTracking.position(turns));
        prevUpdate();
        wait (10, msec);
    }
    stopMotors();
}

//Other functions above <PID_Tag>:
PID chassis = PID(kP, kI, kD, turnKP, turnKI, turnKD);