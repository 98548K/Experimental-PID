#pragma once

class chassis {
    private:
    //PID gains
    double kP, kI, kD, turnKP, turnKI, turnKD;
    //Feedforward gains
    double kS, kA, kV, turnKS, turnKA, turnKV;
    //Derivative filtering time constants
    double kT, turnKT;
    //Ramp up acceleration gains
    double turnKAccel, turnKVel;
    //Ramp up max velocity gains
    double kAccel, kVel;
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
    //Curvature
    double calculatedHeading, linearVel, turnVel;
    public:
    chassis(double KP, double KI, double KD, double KS, double KA, double KV, double KT, double DriveIntegralCap, double DriveSlewLimit, double KAccel, double KVel, double TurnKP, double TurnKI, double TurnKD, double TurnKS, double TurnKA, double TurnKV, double TurnKT, double TurnIntegralCap, double TurnSlewLimit, double TurnKAccel, double TurnKVel);
    void turnToHeading(double desiredHeading, double speed, double timer = 0);
    void driveDist(double desiredDist, double speed, double timer = 0);
    void findCurvature(double desiredX, double desiredY);
};

extern pid turnPID;
extern pid drivePID;
extern double wheelRad;