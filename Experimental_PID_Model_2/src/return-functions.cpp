#include "vex.h"

//Returns positive one for positive values, and negative for negative
int PosNeg;
double minimum;
double maximum;

int sgn(double value) {
    if (value < 0) {
        PosNeg = -1;
    }
    else {
        PosNeg = 1;
    }
    return PosNeg;
}

//Function for determining the turn direction. Credit to Caleb Carlson for making this function easy to find (https://www.vexforum.com/t/turning-with-pid-how-to-find-the-shortest-turn/110258/6):
double constrainAngle(double x) {
    x = fmod(x + 180, 360);
    if (x < 0)
        x += 360;
    return x - 180;
}

double validateHeading(double bestHeading) {
    if (bestHeading < 0) bestHeading += 360;
    if (bestHeading > 360) bestHeading -= 360;
    return bestHeading;
}

double min(double uno, double dos) {
    if (uno <= dos) minimum = uno;
    else if (dos < uno) minimum = dos;
    return minimum;
}

double max(double uno, double dos) {
    if (uno >= dos) maximum = uno;
    else if (dos > uno) maximum = dos;
    return maximum;
}
