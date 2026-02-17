#pragma once

class pid {
    private:
    double integralCap, ramp, slew, p, i, d, s, a, v, t;
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
    public:
    pid(double IntegralCap, double Ramp, double Slew, double P, double I, double D, double S, double A, double V, double T);
    double Speed(double error);
    double Feedforward(double desiredSpeed, double error);
    double RampUp(double currentPosition, double setPoint);
    double SlewRate(double output, double prevOutput, double error, double desiredValue);
    double PrevPwr();
    void Update(double error, double output);
    void reset(double error);
};