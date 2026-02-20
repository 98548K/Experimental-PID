#include "vex.h"

//While loop delay in miliseconds
int updateTime = 20;
double wheelRad = 1.0;

//Tolerances
double driveTolerance = 0.0;
double turnTolerance = 0.0;

//Tune gains:
chassis::chassis(double KP, double KI, double KD, double KS, double KA, double KV, double KT, double DriveIntegralCap, double DriveSlewLimit, double KAccel, double KVel, double TurnKP, double TurnKI, double TurnKD, double TurnKS, double TurnKA, double TurnKV, double TurnKT, double TurnIntegralCap, double TurnSlewLimit, double TurnKAccel, double TurnKVel) {
    //Drive PID gains
    KP = kP;
    KI = kI;
    KD = kD;
    //Drive feedforward gains
    KS = kS;
    KA = kA;
    KV = kV;
    //Drive filtered derivative gain (Time constant)
    KT = kT;
    //Drive integral capacity
    DriveIntegralCap = driveIntegralCap;
    //Drive slew rate
    DriveSlewLimit = driveSlewLimit;
    //Max acceleration gain for drive ramp up
    KAccel = kAccel;
    //Max velocity gain for drive ramp up
    KVel = kVel;
    //Turn PID gains
    TurnKP = turnKP;
    TurnKI = turnKI;
    TurnKD = turnKD;
    //Turn feedforward gains
    TurnKS = turnKS;
    TurnKA = turnKA;
    TurnKV = turnKV;
    //Turn filtered derivative gain (Time constant)
    TurnKT = turnKT;
    //Turn integral capacity
    TurnIntegralCap = turnIntegralCap;
    //Turn slew rate
    TurnSlewLimit = turnSlewLimit;
    //Max acceleration gain for turn ramp up
    TurnKAccel = turnKAccel;
    //Max velocity gain for turn ramp up
    TurnKVel = turnKVel;
};

pid turnPID = pid(0, 0, 0, 0, 0, 0, 0, 0, 0);
pid drivePID = pid(0, 0, 0, 0, 0, 0, 0, 0, 0);


//Turn PID function with tolerance
void chassis::turnToHeading(double desiredHeading, double speed, double timer) {
    turnPID.Reset(turnError);
    while (true) {
        //Sets the turn PID construct
        turnPID = pid(turnIntegralCap, turnSlewLimit, turnKP, turnKI, turnKD, turnKS, turnKA, turnKV, turnKT);

        //Turn error
        turnError = constrainAngle(turnPID.RampUp(desiredHeading, turnKAccel, turnKVel) - Inertial1.heading(deg));

        //Raw speed calculation
        rawTurnOutput = turnPID.Speed(turnError, desiredHeading) + turnPID.Feedforward(speed, turnError);

        //Speed calculation
        turnOutput = turnPID.SlewRate(rawTurnOutput, turnPID.PrevPwr(), turnError, desiredHeading);

        //Spin drivetrain motors
        LeftDriveSmart.spin(fwd, turnOutput, pct);
        RightDriveSmart.spin(reverse, turnOutput, pct);

        turnPID.Update(turnError, turnOutput);

        //Turn exit condition
        if (turnError > -turnTolerance && turnError < turnTolerance && timer == 0) break;
        else if (Brain.timer(sec) >= timer && timer != 0) break;

        //Refresh loop and prevent buildup
        wait (updateTime, msec);
    }
    Drivetrain.stop(hold);
}

//Drive PID function
void chassis::driveDist(double desiredDist, double speed, double timer) {
    //Store sensor values
    storedTrackingMeasurements = (frontTracking.position(turns)) * (wheelRad * 2) * M_PI;
    storedHeading = Inertial1.heading(deg);
    drivePID.Reset(driveError);
    turnPID.Reset(turnError);
    while (true) {
        resetCurrentPosition = desiredDist - (((frontTracking.position(turns)) * (wheelRad * 2) * M_PI) - storedTrackingMeasurements);
        //Sets drive PID constructs
        drivePID = pid(driveIntegralCap, driveSlewLimit, kP, kI, kD, kS, kA, kV, kT);

        //Sets turn PID constructs for anti drift
        turnPID = pid(turnIntegralCap, turnSlewLimit, turnKP, turnKI, turnKD, turnKS, turnKA, turnKV, turnKT);

        //Errors
        driveError = drivePID.RampUp(desiredDist, kAccel, kVel) - resetCurrentPosition;
        turnError = constrainAngle(turnPID.RampUp(storedHeading, turnKAccel, turnKVel) - Inertial1.heading(deg));

        //Raw speed calculations
        rawDriveOutput = drivePID.Speed(driveError, desiredDist) + drivePID.Feedforward(speed, driveError);
        rawTurnOutput = turnPID.Speed(turnError, storedHeading);

        //Speed calculations
        driveOutput = drivePID.SlewRate(rawDriveOutput, drivePID.PrevPwr(), driveError, desiredDist);
        turnOutput = turnPID.SlewRate(rawTurnOutput, turnPID.PrevPwr(), turnError, storedHeading);

        //Spin drivetrain motors
        LeftDriveSmart.spin(fwd, (driveOutput + turnOutput), pct);
        RightDriveSmart.spin(fwd, (driveOutput - turnOutput), pct);

        drivePID.Update(driveError, driveOutput);
        turnPID.Update(turnError, turnOutput);

        //Drive exit condition
        if (driveError > -driveTolerance && driveError < driveTolerance && timer == 0) break;
        else if (Brain.timer(sec) >= timer && timer != 0) break;

        //Refresh loop and prevent buildup
        wait (updateTime, msec);
    }
    Drivetrain.stop(hold);
}

void chassis::findCurvature(double desiredX, double desiredY) {

}