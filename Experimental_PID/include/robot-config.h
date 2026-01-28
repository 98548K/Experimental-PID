using namespace vex;

extern brain Brain;

extern controller Controller1;
extern drivetrain Drivetrain;
extern rotation frontTracking;
extern rotation sideTracking;
extern inertial Inertial1;
//If necessary add these in:
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;

extern motor BackIntake;
extern motor FrontIntake;
extern motor MiddleIntake;


extern digital_out BottomIntakePiston;
extern digital_out TopIntakePiston;
extern digital_out DescorePiston;
extern digital_out BruteForce;
extern digital_out Wing;
extern digital_out trackingWheelPiston;

extern digital_out No_Piston;
extern motor No_Motor;

extern optical BorderControl;
extern gps GPS1;
extern gps GPS2;

extern bool RemoteControlCodeEnabled;

extern int drivetrainLeftSideSpeed;
extern int drivetrainRightSideSpeed;

void  vexcodeInit( void );