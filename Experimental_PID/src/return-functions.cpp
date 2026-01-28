#include "vex.h"


//Formula = ouput + feed forward
double feedForward(double iAmSpeed, double constant) {
    return iAmSpeed * constant * sgn(error);
}

//Function that returns the output value:
double PID_math(double desiredValue, const char* assigned, double KP, double KI, double KD, double vel) {
    //Error definition using the control ID:
    //The integral capacity is 30% velocity
    if (assigned == driveID) {
        error = desiredValue - resetCurrentPosition;
        //Updates integral and derivative
        orderID(kI);
        feedBack = feedForward(vel, kV);
    }
    else if (assigned == turnID) {
        error = constrainAngle((desiredValue-  Inertial1.heading(deg)));
        //Updates integral and derivative
        orderID(turnKI);
        feedBack = feedForward(vel, turnKV);
    }

    //Other error updates above <PID_Tag>:

    //Motor output calculation:
    pwr = slewRate((error * KP) + (integral * KI) + (derivative * KD), prevPwr, error, desiredValue);


    //Return the speed:
    return pwr/* + feedBack*/;
}


//If the delta speed is greater than the slew rate, the speed will be set to the previous speed plus slew rate cap:
double slewRate(double output, double prevOutput, double error, double desiredValue) {
    //Slew rate is slewLimit% velocity
    if (output > prevOutput + slewLimit && std::round(error) == desiredValue) {
        returnSpeed = prevOutput + (slewLimit);
    }
    else if (output < prevOutput - slewLimit && std::round(error) == desiredValue) {
        returnSpeed = prevOutput - (slewLimit);
    }
    else {
        returnSpeed = output;
    }
    return returnSpeed;
}

double rampUp(double setPoint, double currentPoint) {
    if (setPoint - currentPoint >= maxRamp) {
        returnRate = maxRamp;
    }
    else if (setPoint - currentPoint < -maxRamp) {
        returnRate = -maxRamp;
    }
    else {
        returnRate = setPoint - currentPoint;
    }
    currentPoint += returnRate;
    return currentPoint;
}


//Function for determining the turn direction. Credit to Caleb Carlson for making this function easy to find (https://www.vexforum.com/t/turning-with-pid-how-to-find-the-shortest-turn/110258/6):
double constrainAngle(double x) {
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}


int PosNeg;
int sgn(double value) {
    if (value < 0) {
        PosNeg = -1;
    }
    else {
        PosNeg = 1;
    }
    return PosNeg;
}