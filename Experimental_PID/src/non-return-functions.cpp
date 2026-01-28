#include "vex.h"


//Add other situations here:
void schedule(const char* gainSchedule) {
    if (gainSchedule == "Basic") {
        kP = 1.0;
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
    pref = ef;
}

void storeValues() {
    storedTrackingMeasurements = (frontTracking.position(turns)) * (wheelRad * 2) * M_PI;
    storedHeading = Inertial1.heading(deg);
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


void orderID(double IConstant) {
    dt = Brain.timer(sec) - prevTime;

    if (IConstant == kI) {
        ef = ((driveAlpha * error + (1 - driveAlpha) * driveAlpha) * pref);
    }
    else if (IConstant == turnKI) {
        ef = ((turnAlpha * error + (1 - turnAlpha) * turnAlpha) * pref);
    }
    derivative = (ef - pref) / dt;
    integral += error * dt;
    if (std::abs(integral * IConstant) > integralCap) {
        integral = integralCap / IConstant * sgn(error);
    }
}

void resetID() {
    integral = 0;
    ef = 0;
    error = 0;
}