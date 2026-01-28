#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen:
brain  Brain;

// VEXcode device constructors:
controller Controller1 = controller(primary);

motor LF = motor(PORT15, ratio6_1, true);
motor LM = motor(PORT16, ratio6_1, true);
motor LB = motor(PORT17, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(LF, LM, LB);
motor RF = motor(PORT18, ratio6_1, false);
motor RM = motor(PORT19, ratio6_1, false);
motor RB = motor(PORT20, ratio6_1, false);
motor_group RightDriveSmart = motor_group(RF, RM, RB);
inertial Inertial1 = inertial(PORT12);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 320, 40, mm, 0.75);
rotation frontTracking = rotation(PORT13, true);
rotation sideTracking = rotation(PORT14, false);
motor BackIntake = motor(PORT10, ratio18_1, true);
motor FrontIntake = motor(PORT21, ratio18_1, false);
motor MiddleIntake = motor(PORT9, ratio18_1, false);
digital_out BottomIntakePiston = digital_out(Brain.ThreeWirePort.F);
digital_out TopIntakePiston = digital_out(Brain.ThreeWirePort.C);
digital_out DescorePiston = digital_out(Brain.ThreeWirePort.H);
digital_out BruteForce = digital_out(Brain.ThreeWirePort.A);
digital_out Wing = digital_out(Brain.ThreeWirePort.G);
//Don't ask...
digital_out trackingWheelPiston = digital_out(Brain.ThreeWirePort.D);
optical BorderControl = optical(PORT22);
gps GPS1 = gps(PORT2, 87);
gps GPS2 = gps(PORT6, 270);



bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs:
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

int drivetrainLeftSideSpeed;
int drivetrainRightSideSpeed;

// define a task that will handle monitoring inputs from Controller1:
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds:
  // update the motors based on the input values:
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies:
      // left = Axis3
      // right = Axis2
      drivetrainLeftSideSpeed = Controller1.Axis3.position();
      drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range:
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped:
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor:
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped:
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range:
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range:
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped:
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor:
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped:
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range:
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range:
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range:
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

    }
    // wait before repeating the process:
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */

void auto_run() {
  Inertial1.calibrate();
  GPS1.calibrate();
  GPS2.calibrate();
  while (Inertial1.isCalibrating() || GPS1.isCalibrating() || GPS2.isCalibrating()) {
    Inertial1.setHeading(149, deg);
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.print("Calibrating");
  }
}


void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  //Starting setup code here:
  frontTracking.setPosition(0, turns);
  sideTracking.setPosition(0, turns);
  Controller1.Screen.clearScreen();
  //auto_run();
  Controller1.Screen.clearScreen();
}