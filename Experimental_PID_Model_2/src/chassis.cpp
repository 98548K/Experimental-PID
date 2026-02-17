#include "vex.h"

//While loop delay in miliseconds
int updateTime = 20;
double wheelRad = 1.0;

//Tune gains:
chassis::chassis(double KP, double KI, double KD, double KS, double KA, double KV, double KT, double DriveRampRate, double DriveIntegralCap, double DriveSlewLimit, double TurnKP, double TurnKI, double TurnKD, double TurnKS, double TurnKA, double TurnKV, double TurnKT, double TurnRampRate, double TurnIntegralCap, double TurnSlewLimit) {
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
    //Drive ramp up rate
    DriveRampRate = driveRampRate;
    //Drive integral capacity
    DriveIntegralCap = driveIntegralCap;
    //Drive slew rate
    DriveSlewLimit = driveSlewLimit;
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
    //Turn ramp up rate
    TurnRampRate = turnRampRate;
    //Turn integral capacity
    TurnIntegralCap = turnIntegralCap;
    //Turn slew rate
    TurnSlewLimit = turnSlewLimit;
};

pid turnPID;
pid antiDriftPID;
pid drivePID;

//Turn PID function
void chassis::turnToHeading(double desiredHeading, double speed) {
    turnPID.reset(turnError);
    while (true) {
        //Sets the turn PID construct
        turnPID = pid(turnIntegralCap, turnRampRate, turnSlewLimit, turnKP, turnKI, turnKD, turnKS, turnKA, turnKV, turnKT);

        //Turn error
        turnError = constrainAngle(desiredHeading - turnPID.RampUp(Inertial1.heading(deg), desiredHeading));

        //Raw speed calculation
        rawTurnOutput = turnPID.Speed(turnError) + turnPID.Feedforward(speed, turnError);

        //Speed calculation
        turnOutput = turnPID.SlewRate(rawTurnOutput, turnPID.PrevPwr(), turnError, desiredHeading);

        //Spin drivetrain motors
        LeftDriveSmart.spin(fwd, turnOutput, pct);
        RightDriveSmart.spin(reverse, turnOutput, pct);

        turnPID.Update(turnError, turnOutput);
        //Refresh loop and prevent buildup
        wait (updateTime, msec);
    }
}

//Drive PID function
void chassis::driveDist(double desiredDist, double speed) {
    //Store sensor values
    storedTrackingMeasurements = (frontTracking.position(turns)) * (wheelRad * 2) * M_PI;
    storedHeading = Inertial1.heading(deg);
    drivePID.reset(driveError);
    antiDriftPID.reset(turnError);
    while (true) {
        resetCurrentPosition = desiredDist - (((frontTracking.position(turns)) * (wheelRad * 2) * M_PI) - storedTrackingMeasurements);
        //Sets drive PID constructs
        drivePID = pid(driveIntegralCap, driveRampRate, driveSlewLimit, kP, kI, kD, kS, kA, kV, kT);
        
        //Sets turn PID constructs for anti drift
        antiDriftPID = pid(turnIntegralCap, turnRampRate, turnSlewLimit, turnKP, turnKI, turnKD, turnKS, turnKA, turnKV, turnKT);

        //Errors
        driveError = desiredDist - drivePID.RampUp(resetCurrentPosition, desiredDist);
        turnError = constrainAngle(storedHeading - antiDriftPID.RampUp(Inertial1.heading(deg), storedHeading));

        //Raw speed calculations
        rawDriveOutput = drivePID.Speed(driveError) + drivePID.Feedforward(speed, driveError);
        rawTurnOutput = antiDriftPID.Speed(turnError);

        //Speed calculations
        driveOutput = drivePID.SlewRate(rawDriveOutput, drivePID.PrevPwr(), driveError, desiredDist);
        turnOutput = antiDriftPID.SlewRate(rawTurnOutput, antiDriftPID.PrevPwr(), turnError, storedHeading);

        //Spin drivetrain motors
        LeftDriveSmart.spin(fwd, (driveOutput + turnOutput), pct);
        RightDriveSmart.spin(fwd, (driveOutput - turnOutput), pct);

        drivePID.Update(driveError, driveOutput);
        antiDriftPID.Update(turnError, turnOutput);
        //Refresh loop and prevent buildup
        wait (updateTime, msec);
    }
}