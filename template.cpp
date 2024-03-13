#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30
#define MICROSTEPS 1

// Motor 1
#define M1DIR 27
#define M1STEP 26
BasicStepperDriver stepper1(MOTOR_STEPS, M1DIR, M1STEP);

// Motor 2
#define M2DIR 25
#define M2STEP 33
BasicStepperDriver stepper2(MOTOR_STEPS, M2DIR, M2STEP);

#define limit1 22
#define limit2 23

#define buzzer 4

int pos_max = 0;
int pos_min = 0;
int pos_curr = 0;

bool check_limit(int pin)
{
    return digitalRead(pin) != false;
}

int get_current_position()
{
    return pos_curr;
}

void set_current_position(int pos)
{
    pos_curr = pos;
}

void move_to_pos(BasicStepperDriver motor, int pos)
{
    if (check_limit(limit1) || check_limit(limit2))
    {
        motor.startBrake();
    }
}

void calibrate()
{
    stepper2.move(200);
    stepper2.startBrake();
    delay(500);
    stepper2.move(-50);
    stepper2.startBrake();
    delay(500);

    while (check_limit(limit1))
    {
        stepper1.move(2);
        delay(200);
    }

    while (check_limit(limit2))
    {
        stepper1.move(-2);
        delay(200);
    }
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ; // Wait until Serial is ready - important for Leonardo/Micro

    pinMode(limit1, INPUT_PULLUP);
    pinMode(limit2, INPUT_PULLUP);

    stepper1.begin(RPM, MICROSTEPS);
    stepper2.begin(RPM, MICROSTEPS);

    calibrate();
}

void loop()
{
    // if (Serial.available() > 0) {
    //     String command = Serial.readStringUntil('\n');
    //     controlMotor(command);
    // }

    // stepper2.move(100);
    // delay(500);
    // stepper2.move(0);
    // delay(500);

    // stepper1.move(100);
    // delay(500);
    // stepper1.move(-100);
    // delay(500);
}

void controlMotor(String command)
{
    // Command format: m<number> <steps>
    // Example: m1 100, m2 -150

    int motorNumber = command.charAt(1) - '0'; // Convert character to number
    int steps = command.substring(3).toInt();  // Extract number of steps

    if (motorNumber == 1)
    {
        stepper1.move(steps);
    }
    else if (motorNumber == 2)
    {
        stepper2.move(steps);
    }
    else
    {
        Serial.println("Invalid motor number");
    }
}
