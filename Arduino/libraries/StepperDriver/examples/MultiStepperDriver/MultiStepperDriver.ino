/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP_1, DIR_1 as indicated
 *
 * Copyright (C)2015 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 513

// All the wires needed for full functionality
#define DIR_1 8
#define DIR_2 7
#define DIR_3 4
#define DIR_4 2

#define STEP_1 9
#define STEP_2 10
#define STEP_3 11
#define STEP_4 6
//Uncomment line to use enable/disable functionality
#define ENBL 12

// Since microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1


// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper_1(MOTOR_STEPS, DIR_1, STEP_1, ENBL);
BasicStepperDriver stepper_2(MOTOR_STEPS, DIR_2, STEP_2, ENBL);
BasicStepperDriver stepper_3(MOTOR_STEPS, DIR_3, STEP_3, ENBL);
BasicStepperDriver stepper_4(MOTOR_STEPS, DIR_4, STEP_4, ENBL);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR_1, STEP_1, ENBL);

void setup() {
    /*
     * Set target motor RPM.
     * These motors can do up to about 200rpm.
     * Too high will result in a high pitched whine and the motor does not move.
     */
    stepper_1.setRPM(120);
    stepper_2.setRPM(120);
    stepper_3.setRPM(120);
    stepper_4.setRPM(120);

    /*
     * Tell the driver the microstep level we selected.
     * If mismatched, the motor will move at a different RPM than chosen.
     */
    stepper_1.setMicrostep(MICROSTEPS);
    stepper_2.setMicrostep(MICROSTEPS);
    stepper_3.setMicrostep(MICROSTEPS);
    stepper_4.setMicrostep(MICROSTEPS);

    Serial.begin(9600);
}

void loop() {

    /*
     * Moving motor one full revolution using the degree notation
     */
    // stepper.rotate(360);

    /*
     * Moving motor to original position using steps
     */

    stepper_1.run();
    stepper_2.run();
    stepper_3.run();
    stepper_4.run();

    if (stepper_1.distToGoal() == 0) {
        // pause and allow the motor to be moved by hand
        stepper_1.disable();

        delay(2000);

        if (stepper_1.getGoalStep() == MOTOR_STEPS * 4) {
            stepper_1.setGoal(0);
            stepper_2.setGoal(0);
            stepper_3.setGoal(0);
            stepper_4.setGoal(0);
        }
        else {
            stepper_1.setGoal(MOTOR_STEPS * 4);
            stepper_2.setGoal(MOTOR_STEPS * 4);
            stepper_3.setGoal(MOTOR_STEPS * 4);
            stepper_4.setGoal(MOTOR_STEPS * 4);
        }

        // energize coils - the motor will hold position
        stepper_1.enable();

        Serial.println(stepper_1.getGoalStep());
        Serial.println(stepper_1.getCurrentStep());
        Serial.println();
    }
}
