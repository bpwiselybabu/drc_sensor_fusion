/**
 *****************************************************************************
 * @file    car_commands.h
 * @brief   Defines enums for controlling the car
 * @details 
 *****************************************************************************
 */
#ifndef CAR_COMMANDS_H
#define CAR_COMMANDS_H

enum CAR_COMMAND {
    CAR_COMMAND_THROTTLE_SERVO_OFF,
    CAR_COMMAND_THROTTLE_SERVO_ON,
    CAR_COMMAND_GAZEBO_HACKS_OFF,
    CAR_COMMAND_GAZEBO_HACKS_ON,
    CAR_COMMAND_HANDBRAKE_OFF,
    CAR_COMMAND_HANDBRAKE_ON,
    CAR_COMMAND_GEAR_NEUTRAL,
    CAR_COMMAND_GEAR_REVERSE,
    CAR_COMMAND_GEAR_FORWARD,
    CAR_COMMAND_TELEPORT_RELEASE,
    CAR_COMMAND_TELEPORT,
    CAR_COMMAND_LEFT_HAND_PRE_STEER,
    CAR_COMMAND_LEFT_HAND_STEERING,
    CAR_COMMAND_SEATED_POSE,
    CAR_COMMAND_STEERING_WHEEL_IK
};

#endif
