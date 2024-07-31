#include "StepperMotor.h"
#include <math.h>
#include <Servo.h>

#define offsetArm1 100 * 1.5
#define offsetArm2 50 * 1.5
#define offsetArm3 90 * 1.5
#define offsetArm4 145 * 1.5
#define offsetArm5 150 * 1.5

#define	START_ANGEL_ARM3 ((0 + offsetArm3) / 1.5)
#define	START_ANGEL_ARM5 ((0 + offsetArm5) /  1.5)

namespace mip {
	Servo arm1;
	Servo arm2;
	Servo arm3;
	Servo arm4;
	Servo arm5;


    int angelArm1 = 0;
    int angelArm2 = 0;
    int angelArm4 = 0;
    int stepperAngel = 0;

    float x0 = 250;
    float y0 = 0;
    float z0 = 0;
    float Fi0 = 0;

    const float l1 = 153;
    const float l2 = 100;
    const float l3 = 137;
    const float l4 = 119;

	void initArms(int pinArm1, int pinArm2, int pinArm3, int pinArm4, int pinArm5) {
		arm1.attach(pinArm1);
		arm2.attach(pinArm2);
		arm3.attach(pinArm3);
        arm4.attach(pinArm4);
        arm5.attach(pinArm5);

        arm1.write(angelArm1);
        arm2.write(angelArm2);
        arm4.write(angelArm4);

        arm3.write(START_ANGEL_ARM3);
        arm5.write(START_ANGEL_ARM5);

        stepper.init(12, 11, 10, 1600);

        stepper.enable();
        stepper.setAngleDegrees(0);
	}

    void setArms(float j1, float j2, float j3, float j4) {

        float angelOutJ2 = ((-j2 + offsetArm1) / 1.5);
        float angelOutJ3 = ((j3 + offsetArm2) / 1.5);
        float angelOutJ4 = ((j4 + offsetArm4) / 1.5);
        float angelOutStepper = j1 * 2.4;

        arm1.write(angelOutJ2);
        arm2.write(angelOutJ3);
        arm4.write(angelOutJ4);
        
        const double servoMaxSpeed = 5;
        const double servoAcceleration = 1;
        const int servoAngelNear = 3;

        stepper.setAngleDegrees(angelOutStepper);

        angelArm1 = angelOutJ2;
        angelArm2 = angelOutJ3;
        angelArm4 = angelOutJ4;
        stepperAngel = angelOutStepper;
    }

    bool isCrashed(float j1, float j2, float j3, float j4) {
        float Fi = 90 - j4 - j2 - j3;
        float Betta = 180 - j2 - j3;

        float xh0 = 950;
        float z0 = 550;

        float sx = 500;
        float sy = 550;



        float xh1 = xh0 + 0;
        float z1 = z0 - l1;
        float xh2 = xh1 + sin(j2 / 180 * PI) * l2;
        float z2 = z1 - cos(j2 / 180 * PI) * l2;
        float xh3 = xh2 + sin(Betta / 180 * PI) * l3;
        float z3 = z2 + cos(Betta / 180 * PI) * l3;
        float xh4 = xh3 + cos(Fi / 180 * PI) * l4;
        float z4 = z3 - sin(Fi / 180 * PI) * l4;

        float x = sin(j1 / 180 * PI) * (xh4 - xh0);
        float y = cos(j1 / 180 * PI) * (xh4 - xh0);
        float z = z4;


        float xg2 = sx + sin(j1 / 180 * PI) * (xh2 - xh1);
        float xg3 = xg2 + sin(j1 / 180 * PI) * (xh3 - xh2);
        float xg4 = xg3 + sin(j1 / 180 * PI) * (xh4 - xh3);

        float yg2 = sy + cos(j1 / 180 * PI) * (xh2 - xh1);
        float yg3 = yg2 + cos(j1 / 180 * PI) * (xh3 - xh2);
        float yg4 = yg3 + cos(j1 / 180 * PI) * (xh4 - xh3);

        if (isnan(x) || isnan(y) || isnan(z)) {
            return true;
        }
        else {
            return false;
        }
    }

    void runIKP(float x, float y, float z, float Fi)
    {
        float xh4 = sqrt(x * x + y * y);
        float xh3 = xh4 - cos(Fi / 180 * PI) * l4;
        float z3 = z - sin(Fi / 180 * PI) * l4;
        float L = sqrt(xh3 * xh3 + pow((z3 - l1), 2));
        float Betta = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3)) * 180 / PI;
        float Gamma = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L)) * 180 / PI;
        float Tetta = atan2((z3 - l1), xh3) * 180 / PI;


        if (!(l2 < (l3 + L) || l3 < (l2 + L) || L < (l2 + l3))) {
            return;
        }

        float j1 = atan2(x, y) * 180 / PI;
        float j2 = 90 - Gamma - Tetta;
        float j3 = 180 - Betta;
        float j4 = 90 - Fi - j2 - j3;


        if (isCrashed(j1, j2, j3, j4)) {
            return;
        }

        Serial.println(String(j1) + "\t" + String(j2) + "\t" + String(j3));

        setArms(j1, j2, j3, j4);
    }


    void softServoMove(double x1, double y1, float z1, float Fi1, float a, double Vmax) {

        if (x1 < 50 && y1 < 150)
            return;
        
        float L = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2) + pow((z1 - z0), 2) + pow((Fi1 - Fi0), 2));


        float k0 = 0;
        float k1 = L;

        double t0 = millis() / 1000.0;
        double t1 = (Vmax / a) + ((abs(k1 - k0)) / Vmax) + t0;
        double t2 = t0 + (Vmax / a);
        double t3 = (abs(k1 - k0) / Vmax) + t0;
        double t4 = 2 * sqrt((abs(k1 - k0) / a)) + t0;
        double t5 = sqrt(abs(k1 - k0) / a) + t0;

        double Vmax2 = a * (t5 - t0);
        float direction = 0;

        if ((k1 - k0) == 0)
            direction = 0;
        else if ((k1 - k0) > 0)
            direction = 1;
        else if ((k1 - k0) < 0)
            direction = -1;

        double k = k0;
        double t = millis() / 1000.0;

        if (t3 < t2) {
            while (t < t1) {
                t = millis() / 1000.0;


                if (t5 <= t && t <= t4) {
                    k = k0 + (((a * pow((t5 - t0), 2) / 2) + (Vmax2 * (t - t5)) - ((a * pow((t - t5), 2) / 2)))) * direction;
                }
                else if (t0 <= t && t <= t5) {
                    k = k0 + ((a * pow((t - t0), 2)) / 2) * direction;
                }

                float xt = x0 + (x1 - x0) * k / L;
                float yt = y0 + (y1 - y0) * k / L;
                float zt = z0 + (z1 - z0) * k / L;
                float Fi_t = Fi0 + (Fi1 - Fi0) * k / L;


                runIKP(xt, yt, zt, Fi_t);
            }
        }
        else
        {
            while (t < t1) {
                t = millis() / 1000.0;

                if (t0 <= t && t <= t2) {
                    k = k0 + ((a * pow((t - t0), 2)) / 2) * direction;
                }
                else if (t2 <= t && t <= t3) {
                    k = k0 + (((pow(Vmax, 2)) / (2 * a)) + Vmax * (t - t2)) * direction;
                }
                else if (t3 <= t && t <= t1) {
                    k = k0 + (((pow(Vmax, 2)) / (2 * a)) + Vmax * (t3 - t2) + Vmax * (t - t3) - ((a * pow((t - t3), 2)) / 2)) * direction;
                }

                float xt = x0 + (x1 - x0) * k / L;
                float yt = y0 + (y1 - y0) * k / L;
                float zt = z0 + (z1 - z0) * k / L;
                float Fi_t = Fi0 + (Fi1 - Fi0) * k / L;

                runIKP(xt, yt, zt, Fi_t);
            }
        }

        x0 = x1;
        y0 = y1;
        z0 = z1;
        Fi0 = Fi1;
    }
}