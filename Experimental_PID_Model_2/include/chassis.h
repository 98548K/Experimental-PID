#pragma once

class chassis {
    private:
    //PID gains
    double kP, kI, kD, turnKP, turnKI, turnKD;
    //Feedforward gains
    double kS, kA, kV, turnKS, turnKA, turnKV;
    //Derivative filtering time constants
    double kT, turnKT;
    //Ramp up rate gains
    double driveRampRate, turnRampRate;
    //Integral capacity in percentage
    double driveIntegralCap, turnIntegralCap;
    //Maximum slew
    double driveSlewLimit, turnSlewLimit;
    //Used for multiple instances of the "Error" argument
    double driveError;
    double turnError;
    //Output balancing variable
    double turnOutput;
    double driveOutput;
    //Outputs without the slew rate
    double rawTurnOutput;
    double rawDriveOutput;
    //Sensor values
    double storedTrackingMeasurements, storedHeading;
    double resetCurrentPosition;
    public:
    chassis(double KP, double KI, double KD, double KS, double KA, double KV, double KT, double DriveRampRate, double DriveIntegralCap, double DriveSlewLimit, double TurnKP, double TurnKI, double TurnKD, double TurnKS, double TurnKA, double TurnKV, double TurnKT, double TurnRampRate, double TurnIntegralCap, double turnSlewLimit);
    void driveDist(double desiredDist, double speed);
    void turnToHeading(double desiredHeading, double speed);
};