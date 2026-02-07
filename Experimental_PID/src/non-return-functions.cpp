#include "vex.h"


//Basic, Long distance, Short distance:
void schedule(const char* gainSchedule) {
    if (gainSchedule == "Basic") {
        kP = 0.0;
        kI = 0.0;
        kD = 0.0;
        turnKP = 0.0;
        turnKI = 0.0;
        turnKD = 0.0;
    }

    //Other stored constants above <PID_Tag>
}


void prevUpdate() {
    prevPwr = output;
    prevError = error;
    prevTime = Brain.timer(sec);
    prevDerivative = derivative;
}

void storeValues() {
    storedTrackingMeasurements = (frontTracking.position(turns)) * (wheelRad * 2) * M_PI;
    storedHeading = Inertial1.heading(deg);

    //Other stored values above <PID_Tag>
    startTimer = Brain.timer(sec);
    Brain.resetTimer();
}

void stopMotors() {
    LeftDriveSmart.stop(hold);
    RightDriveSmart.stop(hold);
    Brain.setTimer(Brain.timer(sec) + startTimer, sec);
}

void printAtTop(double value) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.print(value);
}


void printAtTop(const char * value) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.print(value);
}


void orderID(double IConstant, double timeConstant) {
    dt = Brain.timer(sec) - prevTime;
    //First-order IIR (Infinite Impulse Response)

    if (IConstant == kI) {
        driveAlpha = timeConstant / (timeConstant + dt);
        beta = 1 - driveAlpha;
        derivative = driveAlpha * prevDerivative + beta * (error - prevError);
    }
    else if (IConstant == turnKI) {
        turnAlpha = timeConstant / (timeConstant + dt);
        beta = 1 - turnAlpha;
        derivative = turnAlpha * prevDerivative + beta * (error - prevError);
    }

    //Other derivatives above <PID_Tag>
    integral += error * dt;
    if (std::abs(integral * IConstant) > integralCap) {
        integral = integralCap / IConstant * sgn(error);
    }
}

void resetID() {
    integral = 0;
    error = 0;
}