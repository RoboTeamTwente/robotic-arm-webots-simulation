/*
 * File:            my_arm_controller.c
 * Date:            09/04/2026
 * Description:     simple test for webots robotic arm
 * Author:          me
 * Modifications:   made this file
 */

#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <webots/keyboard.h>

#include "control_arm_manual_webots.h"
#include "rtwtypes.h"

#define TIME_STEP 64
#define SAMPLE_PERIOD 10

int main(int argc, char **argv){
    (void)argc;
    (void)argv;
    /* necessary to initialize webots stuff */
    wb_robot_init();

    //initializing control system
    control_arm_manual_webots_initialize();

    //setting tags for motors
    WbDeviceTag baseRotationMotor    = wb_robot_get_device("base_rotation_motor");
    WbDeviceTag baseRightMotor       = wb_robot_get_device("bottom_sandwich_motor");
    WbDeviceTag baseLeftMotor        = wb_robot_get_device("short_linkage_motor");
    WbDeviceTag gripperPitchMotor    = wb_robot_get_device("gripper_pitch_motor");
    WbDeviceTag gripperRotationMotor = wb_robot_get_device("gripper_rotation_motor");

    WbDeviceTag topJawsMotor = wb_robot_get_device("top_jaw_motor");
    WbDeviceTag bottomJawsMotor = wb_robot_get_device("bottom_jaw_motor");
    WbDeviceTag leftJawsMotor = wb_robot_get_device("left_jaw_motor");
    WbDeviceTag rightJawsMotor = wb_robot_get_device("right_jaw_motor");
    printf("initialized motor device tags\n");

    //setting tags for 
    WbDeviceTag baseRotationSensor    = wb_robot_get_device("base_rotation_sensor");
    WbDeviceTag baseRightSensor       = wb_robot_get_device("bottom_sandwich_sensor");
    WbDeviceTag baseLeftSensor        = wb_robot_get_device("short_linkage_sensor");
    WbDeviceTag gripperPitchSensor    = wb_robot_get_device("gripper_pitch_sensor");
    WbDeviceTag gripperRotationSensor = wb_robot_get_device("gripper_rotation_sensor");
    printf("initialized rotation sensor device tags\n");

    //enabling the position sensors
    wb_position_sensor_enable(baseRotationSensor, SAMPLE_PERIOD);
    wb_position_sensor_enable(baseRightSensor, SAMPLE_PERIOD);
    wb_position_sensor_enable(baseLeftSensor, SAMPLE_PERIOD);
    wb_position_sensor_enable(gripperPitchSensor, SAMPLE_PERIOD);
    wb_position_sensor_enable(gripperRotationSensor, SAMPLE_PERIOD);
    printf("enabled position sensors\n");

    //setting speed to max for fast responses
    wb_motor_set_position(baseRotationMotor,    0);
    wb_motor_set_velocity(baseRotationMotor,    1);
    wb_motor_set_position(baseRightMotor,       0);
    wb_motor_set_velocity(baseRightMotor,       1);
    wb_motor_set_position(baseLeftMotor,        0);
    wb_motor_set_velocity(baseLeftMotor,        1);
    wb_motor_set_position(gripperPitchMotor,    0);
    wb_motor_set_velocity(gripperPitchMotor,    1);
    wb_motor_set_position(gripperRotationMotor, 0);
    wb_motor_set_velocity(gripperRotationMotor, 1);

    wb_motor_set_position(topJawsMotor,    0);
    wb_motor_set_velocity(topJawsMotor,    1);
    wb_motor_set_position(bottomJawsMotor, 0);
    wb_motor_set_velocity(bottomJawsMotor, 1);
    wb_motor_set_position(leftJawsMotor,   0);
    wb_motor_set_velocity(leftJawsMotor,   1);
    wb_motor_set_position(rightJawsMotor,  0);
    wb_motor_set_velocity(rightJawsMotor,  1);

    printf("initialized motor positions\n");

    //positioning variables
    real_T x          = 0.5;
    real_T y          = 0.2;
    real_T z          = 0;
    real_T gripperAng = 90;

    //initializing control variables
    real_T jawDesPos             = 0;
    real_T gripperRotationDesPos = 0;
    real_T jawActPos             = 0;
    real_T gripperRotationActPos = 0;
    real_T gripperPitchActPos    = 0;
    real_T baseRotationActPos    = 0;
    real_T baseLeftActPos        = 0;
    real_T baseRightActPos       = 0;

    real_T gripperJawsDesPos = 0;

    float deltaX = 0.005;
    float deltaY = 0.005;
    float deltaZ = 0.005;

    //enableing manual control
    wb_keyboard_enable(SAMPLE_PERIOD);

    while(wb_robot_step(TIME_STEP) != -1){
        int key = wb_keyboard_get_key();
        switch(key){
        case 87://W
            x += deltaX;
            break;
        case 83://S
            x -= deltaX;
            break;
        case 65://A
            y += deltaY;
            break;
        case 68://D
            y -= deltaY;
            break;
        case 69://E
            z += deltaZ;
            break;
        case 81://Q
            z -= deltaZ;
            break;
        case WB_KEYBOARD_UP:
            gripperAng += 2;
            break;
        case WB_KEYBOARD_DOWN:
            gripperAng -= 2;
            break;
        case WB_KEYBOARD_LEFT:
            gripperRotationDesPos -= 4;
            break;
        case WB_KEYBOARD_RIGHT:
            gripperRotationDesPos += 4;
            break;
        case 82://R
            gripperJawsDesPos += 0.02;
            break;
        case 70://F
            gripperJawsDesPos -= 0.02;
            break;
        default:
            break;
        }

        //reading sensors
        baseRotationActPos    = wb_position_sensor_get_value(baseRotationSensor);
        baseRightActPos       = wb_position_sensor_get_value(baseRightSensor);
        baseLeftActPos        = wb_position_sensor_get_value(baseLeftSensor);
        gripperPitchActPos    = wb_position_sensor_get_value(gripperPitchSensor);
        gripperRotationActPos = wb_position_sensor_get_value(gripperRotationSensor);

        //setting control inputs
        rtU.x                              = x;
        rtU.y                              = y;
        rtU.z                              = z;
        rtU.gripperAng                     = gripperAng*(M_PI/180);
        rtU.desiredposition                = jawDesPos;
        rtU.gripperRotationDesiredPosition = gripperRotationDesPos*(M_PI/180);
        rtU.actualposition                 = jawActPos;
        rtU.gripperRotationActualPosition  = gripperRotationActPos;
        rtU.gripperPitchActualPosition     = gripperPitchActPos;
        rtU.baseActualPosition             = baseRotationActPos;
        rtU.stepperLeftActualPosition      = baseLeftActPos;
        rtU.stepperRightActualPosition     = baseRightActPos;
        rtU.deltaTime                      = 0.001;

        control_arm_manual_webots_step();

        //updating motor positions
        wb_motor_set_position(baseRotationMotor,    rtY.controlBase);
        wb_motor_set_position(baseRightMotor,       rtY.controlStepperRight);
        wb_motor_set_position(baseLeftMotor,        rtY.controlStepperLeft);
        wb_motor_set_position(gripperPitchMotor,    rtY.controlGripperPitch);
        wb_motor_set_position(gripperRotationMotor, rtY.controlWristRotation);

        printf("angles : %f %f %f %f %f\n", rtY.controlBase, rtY.controlStepperRight, rtY.controlStepperLeft, rtY.controlGripperPitch, rtY.controlWristRotation);
        printf("test : %f\n", rtY.testProbe);

        wb_motor_set_position(topJawsMotor, gripperJawsDesPos);
        wb_motor_set_position(bottomJawsMotor, gripperJawsDesPos);
        wb_motor_set_position(leftJawsMotor, gripperJawsDesPos);
        wb_motor_set_position(rightJawsMotor, gripperJawsDesPos);

        //printf("desired position : [%f, %f, %f] actual position : [%f, %f, %f]\n", x, y, z, rtY.actualX, rtY.actualY, rtY.actualZ);
    };

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();
    wb_keyboard_disable();

    return 0;
}
