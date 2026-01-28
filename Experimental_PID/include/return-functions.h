//Function that returns the output value:
double PID_math(double desiredValue, const char* assigned, double KP, double KI, double KD, double vel);


//If the delta speed is greater than the slew rate, the speed will be set to the previous speed plus slew rate cap:
double slewRate(double output, double prevOutput, double error, double desiredValue);

double rampUp(double setPoint, double currentPoint);


//Function for determining the turn direction. Credit to Caleb Carlson for making this function easy to find (https://www.vexforum.com/t/turning-with-pid-how-to-find-the-shortest-turn/110258/6):
double constrainAngle(double x);


int sgn(double value);
