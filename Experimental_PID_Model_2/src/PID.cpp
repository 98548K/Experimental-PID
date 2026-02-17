#include "vex.h"

pid::pid(double IntegralCap, double Ramp, double Slew, double P, double I, double D, double S, double A, double V, double T) {
    IntegralCap = integralCap;
    Ramp = ramp;
    Slew = slew;
    //PID gains
    P = p;
    I = i;
    D = d;
    //Feedforward gains
    S = s;
    A = a;
    V = v;
    //Filtered derivative gain
    T = t;
};


double pid::Speed(double error) {
    dt = Brain.timer(sec) - prevTime;
    //Update alpha and beta for filtered derivative
    alpha = t / (t + dt);
    beta = 1 - alpha;
    //Update the derivative
    derivative = alpha * prevDerivative + beta * (error - prevError);

    //Update the integral
    integral += error * dt;
    if (std::abs(integral * i) > integralCap) {
        integral = integralCap / i * sgn(error);
    }
    
    //Normal PID without extensions
    pwr = (error * p) + (integral * i) + (derivative * d);

    return pwr;
}

double pid::Feedforward(double setSpeed, double error) {
    acceleration = (pwr - prevPwr) / dt;
    return (setSpeed * v) + (acceleration * a) + (s) * sgn(error);
}

double pid::RampUp(double setPoint, double currentSetpoint) {
    if (currentSetpoint < setPoint) currentSetpoint = min(currentSetpoint + ramp * dt, setPoint);
    else if (currentSetpoint >= setPoint) currentSetpoint = max(currentSetpoint - ramp * dt, setPoint);
    return currentSetpoint;
}

//If the delta speed is greater than the slew rate, the speed will be set to the previous speed plus slew rate cap:
double pid::SlewRate(double output, double prevOutput, double error, double desiredValue) {
    //Slew rate is slewLimit% velocity
    if (output > prevOutput + slew && std::round(error) == desiredValue) {
        returnSpeed = prevOutput + slew;
    }
    else if (output < prevOutput - slew && std::round(error) == desiredValue) {
        returnSpeed = prevOutput - slew;
    }
    else {
        returnSpeed = output;
    }
    return returnSpeed;
}

double pid::PrevPwr() {
    return prevPwr;
}

void pid::Update(double error, double output) {
    //Update previous values
    prevError = error;
    prevPwr = output;
    prevDerivative = derivative;
    prevTime = Brain.timer(sec);
}

void pid::Reset(double error) {
    error = 0;
    integral = 0;
}