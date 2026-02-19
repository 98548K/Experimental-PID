#include "vex.h"

pid::pid(double IntegralCap, double Slew, double P, double I, double D, double S, double A, double V, double T) {
    IntegralCap = integralCap;
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

double pid::RampUp(double setPoint, double maxAccel, double maxVel) {
    //Remaining distance to get to the destination
    profiledError = setPoint - profiledSetpoint;
    //Squares the current velocity and divides that amount by double the max acceleration
    stoppingDistance = (currentVelocity * currentVelocity) / (2 * maxAccel);
    //Finds out if the direction is positive or negative and returns a positive or negative one depending on if the condition is true or false
    direction = (profiledError > 0) ? 1.0 : -1.0;
    //Ramps up if the ramped error is not close to the stopping distance
    if (std::abs(profiledError) > stoppingDistance) {
        currentVelocity += maxAccel * dt;
    }
    //Ramps down if the ramped error is close to the stopping distance
    else {
        currentVelocity -= maxAccel * dt;
    }
    //Constrains the curent velocity to the maximum velocity
    if (currentVelocity > maxVel) currentVelocity = maxVel;
    //Ensures that the current velocity does not dip below 0
    if (currentVelocity < 0) currentVelocity = 0;
    
    profiledSetpoint += currentVelocity * direction * dt;
    if (std::abs(profiledError) < 0.05) {
        profiledSetpoint = setPoint;
        currentVelocity = 0;
    }
    return profiledSetpoint;
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