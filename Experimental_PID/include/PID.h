extern double wheelRad;

extern double resetCurrentPosition;


//Tuning constants:
extern double kP;
extern double kI;
extern double kD;

extern double turnKP;
extern double turnKI;
extern double turnKD;


extern double kV;
extern double turnKV;
extern double feedBack;

void printAtTop(double value);

void orderID(double IConstant);

class PID {
    private:
    double p;
    double i;
    double d;
    double turnP;
    double turnI;
    double turnD;

    public:

    PID(double P, double I, double D, double TurnP, double TurnI, double TurnD);
    void turnToHeading(double desiredValue, double vel);
    void drive(double desiredValue, double vel);
    void turnToHeading(double desiredValue, double vel, double timePeriod);
    void drive(double desiredValue, double vel, double timePeriod);
    void driveWithPiston(double desiredValue, double vel, double deployRange);
};

extern PID chassis;

extern double error;
extern double integral;
extern double derivative;
extern double prevError;
extern double pwr;
extern double prevPwr;
extern double dt;
extern double prevTime;
extern double storedTrackingMeasurements;
extern double resetCurrentPosition;
extern double storedHeading;
extern double startTimer;

extern double maxRamp;

extern double integralCap;

extern double output;

extern double driveTolerance;
extern double turnTolerance;


extern const char* driveID;
extern const char* turnID;


extern double returnSpeed;
extern double returnRate;

extern double slewLimit;

extern double ef;
extern double pref;
extern double driveAlpha;
extern double turnAlpha;