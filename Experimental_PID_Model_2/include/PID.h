#pragma once

class pid {
    private:
    double integralCap, slew, p, i, d, s, a, v, t;
    double integral;
    double derivative;
    double pwr;
    double prevError;
    double prevPwr;
    double prevTime;
    double prevDerivative;
    double acceleration;
    double alpha;
    double beta;
    double dt;
    double returnSpeed;
    //Profiled models for ramp code
    double profiledSetpoint;
    double profiledError;
    double currentVelocity;
    double stoppingDistance;
    double direction;
    public:
    pid(double IntegralCap, double Slew, double P, double I, double D, double S, double A, double V, double T);
    double Speed(double error, double setPoint);
    double Feedforward(double desiredSpeed, double error);
    double RampUp(double setPoint, double maxAccel, double maxVel);
    double SlewRate(double output, double prevOutput, double error, double desiredValue);
    double PrevPwr();
    void Update(double error, double output);
    void Reset(double error);
};