#include "mip.h"


void setup() {
    Serial.begin(115200);

    mip::initArms(A0, A1, A2, A3, A4);
}

void loop() {
    mip::softServoMove(200, 100, 100, 0, 50, 70);
    mip::softServoMove(250, 0, 200, 0, 50, 70);
}