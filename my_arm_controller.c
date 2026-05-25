/*
 * File:            my_arm_controller.c
 * Date:            09/04/2026
 * Description:     simple test for webots robotic arm
 * Author:          me
 * Modifications:   made this file
 */

#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <webots/keyboard.h>
#include <webots/joystick.h>

#include "control_arm_manual_webots.h"
#include "rtwtypes.h"

#define TIME_STEP 64
#define SAMPLE_PERIOD 10

int main(int argc, char **argv){
    (void)argc; (void)argv;
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

    WbDeviceTag topJawsMotor    = wb_robot_get_device("top_jaw_motor");
    WbDeviceTag bottomJawsMotor = wb_robot_get_device("bottom_jaw_motor");
    WbDeviceTag leftJawsMotor   = wb_robot_get_device("left_jaw_motor");
    WbDeviceTag rightJawsMotor  = wb_robot_get_device("right_jaw_motor");
    printf("initialized motor device tags\n");

    //setting tags for 
    WbDeviceTag baseRotationSensor    = wb_robot_get_device("base_rotation_sensor");
    WbDeviceTag baseRightSensor       = wb_robot_get_device("bottom_sandwich_sensor");
    WbDeviceTag baseLeftSensor        = wb_robot_get_device("short_linkage_sensor");
    WbDeviceTag gripperPitchSensor    = wb_robot_get_device("gripper_pitch_sensor");
    WbDeviceTag gripperRotationSensor = wb_robot_get_device("gripper_rotation_sensor");
    printf("initialized rotation sensor device tags\n");

    //enabling the position sensors
    wb_position_sensor_enable(baseRotationSensor,    SAMPLE_PERIOD);
    wb_position_sensor_enable(baseRightSensor,       SAMPLE_PERIOD);
    wb_position_sensor_enable(baseLeftSensor,        SAMPLE_PERIOD);
    wb_position_sensor_enable(gripperPitchSensor,    SAMPLE_PERIOD);
    wb_position_sensor_enable(gripperRotationSensor, SAMPLE_PERIOD);
    printf("enabled position sensors with %dms sample period\n", SAMPLE_PERIOD);

    //initializing positions, and setting velocities to 1
    wb_motor_set_position(baseRotationMotor,    0); wb_motor_set_velocity(baseRotationMotor,    1);
    wb_motor_set_position(baseRightMotor,       0); wb_motor_set_velocity(baseRightMotor,       1);
    wb_motor_set_position(baseLeftMotor,        0); wb_motor_set_velocity(baseLeftMotor,        1);
    wb_motor_set_position(gripperPitchMotor,    0); wb_motor_set_velocity(gripperPitchMotor,    1);
    wb_motor_set_position(gripperRotationMotor, 0); wb_motor_set_velocity(gripperRotationMotor, 1);

    wb_motor_set_position(topJawsMotor,    0); wb_motor_set_velocity(topJawsMotor,    1);
    wb_motor_set_position(bottomJawsMotor, 0); wb_motor_set_velocity(bottomJawsMotor, 1);
    wb_motor_set_position(leftJawsMotor,   0); wb_motor_set_velocity(leftJawsMotor,   1);
    wb_motor_set_position(rightJawsMotor,  0); wb_motor_set_velocity(rightJawsMotor,  1);
    printf("initialized motor positions and speeds to 0rad and 1rad/s\n");

    //positioning variables initialized to starting position
    real_T x          = 0.795;
    real_T y          = 0.0;
    real_T z          = 0.322;
    real_T gripperAng = 5.0;

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

    double deltaX = 0.005;
    double deltaY = 0.005;
    double deltaZ = 0.005;

    //enableing manual control
    wb_keyboard_enable(SAMPLE_PERIOD);
    wb_joystick_enable(SAMPLE_PERIOD);

    //main program loop
    while(wb_robot_step(TIME_STEP) != -1){
        //processing keyboard control
        switch(wb_keyboard_get_key()){
            //positioning controls
            case 87: x += deltaX; break;//W
            case 83: x -= deltaX; break;//S
            case 65: y += deltaY; break;//A
            case 68: y -= deltaY; break;//D
            case 69: z += deltaZ; break;//E
            case 81: z -= deltaZ; break;//Q
            //gripper angle controls
            case WB_KEYBOARD_UP:   gripperAng += 2; break;//up arrow
            case WB_KEYBOARD_DOWN: gripperAng -= 2; break;//down arrow
            //gripper rotation controls
            case WB_KEYBOARD_LEFT:  gripperRotationDesPos += 4; break;//left arrow
            case WB_KEYBOARD_RIGHT: gripperRotationDesPos -= 4; break;//right arrow
            //gripper opening/closing controls
            case 82: gripperJawsDesPos += 0.02; break;//R
            case 70: gripperJawsDesPos -= 0.02; break;//F
            default: break;
        }

        //processing joystick controls if a joystick is enabled
        if(wb_joystick_get_sampling_period() != 0){
            //processing joystick controls if a joystick is connected
            if(wb_joystick_is_connected()){
                int leftH   = wb_joystick_get_axis_value(0);
                int leftV   = wb_joystick_get_axis_value(1);
                int leftTr  = wb_joystick_get_axis_value(2);
                int rightH  = wb_joystick_get_axis_value(3);
                int rightV  = wb_joystick_get_axis_value(4);
                int rightTr = wb_joystick_get_axis_value(5);
                int dpad    = wb_joystick_get_pov_value(0);
                //left joystick
                if(abs(leftV) > 10)  x -= ((float)leftV / 32768.0) * deltaX;
                if(abs(leftH) > 10)  y -= ((float)leftH / 32768.0) * deltaY;
                //right joystick
                if(abs(rightV) > 10) z -= ((float)rightV / 32768.0) * deltaZ;
                if(abs(rightH) > 10) gripperRotationDesPos -= ((float)rightH / 32768.0) * 4;
                //triggers
                if(rightTr > 0) gripperJawsDesPos += ((float)rightTr / 32768.0) * 0.02;
                else if(leftTr > 0) gripperJawsDesPos -= ((float)leftTr / 32768.0) * 0.02;
                //dpad
                switch(dpad){
                    case 1:    gripperAng += 2; break; //up
                    case 16:   gripperAng -= 2; break; //down
                    case 256:  break; //right
                    case 4096: break; //left
                    default: break;
                }
            }
            //disableing if nothing is connected
            else{
                wb_joystick_disable();
            }
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
        rtU.jawDesiredPosition             = jawDesPos;
        rtU.gripperRotationDesiredPosition = gripperRotationDesPos*(M_PI/180);
        rtU.jawActualPosition              = jawActPos;
        rtU.gripperRotationActualPosition  = gripperRotationActPos;
        rtU.gripperPitchActualPosition     = gripperPitchActPos;
        rtU.baseActualPosition             = baseRotationActPos;
        rtU.stepperLeftActualPosition      = baseLeftActPos;
        rtU.stepperRightActualPosition     = baseRightActPos;
        rtU.deltaTime                      = TIME_STEP * 0.001;//TIME_STEP is in milliseconds

        control_arm_manual_webots_step();

        //updating motor positions
        wb_motor_set_position(baseRotationMotor,    rtY.controlBase);
        wb_motor_set_position(baseRightMotor,       rtY.controlStepperRight);
        wb_motor_set_position(baseLeftMotor,        rtY.controlStepperLeft);
        wb_motor_set_position(gripperPitchMotor,    rtY.controlGripperPitch);
        wb_motor_set_position(gripperRotationMotor, rtY.controlWristRotation);

        wb_motor_set_position(topJawsMotor,    gripperJawsDesPos);
        wb_motor_set_position(bottomJawsMotor, gripperJawsDesPos);
        wb_motor_set_position(leftJawsMotor,   gripperJawsDesPos);
        wb_motor_set_position(rightJawsMotor,  gripperJawsDesPos);

        //sending info about positioning
        printf("desired position : [%.3f, %.3f, %.3f] actual position : [%.3f, %.3f, %.3f] test : [%f]\n", x, y, z, rtY.actualX, rtY.actualY, rtY.actualZ, rtY.test);
    };

    //disalbing manual control
    wb_keyboard_disable();
    wb_joystick_disable();

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
