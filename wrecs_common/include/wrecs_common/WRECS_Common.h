#ifndef WRECS_COMMON_H
#define WRECS_COMMON_H

#include <atlas_msgs/AtlasCommand.h>
#include <wrecs_msgs/field_command.h>
#include <bdi_api/AtlasTypes.h>

// Pass in the desired description
// returns the full path with date pre-pended to the description
// example
// input
//   sometext.txt
// return value
//   /logs/2013_08_01_15_33_24_sometext.txt
const std::string nameLogFile(const char* description_dot_csv);

// returns the current time as a double
double getTimeDouble();

// Correctly sizes the arrays in an AtlasCommand
void resizeAtlasCommand(atlas_msgs::AtlasCommand *ac);

// Zeroes out an AtlasCommand
void zeroAtlasCommand(atlas_msgs::AtlasCommand *ac);

// Sets PID gains for the joints
void setJointParameters(atlas_msgs::AtlasCommand *jc);

// Sets PID gains for the joints back to the defaults
void setJointParametersDefault(atlas_msgs::AtlasCommand *jc);

// Sets PID gains for the joints - lower gains to reduce
// chatter and sliding while sitting in vehicle
void setJointParameters_lowerGains(atlas_msgs::AtlasCommand *jc);

// Sets PID gains for the joints - three joints in left arm off
void setJointParameters_leftArmOff(atlas_msgs::AtlasCommand *jc);

// Sets PID gains for the joints - arms relaxed to allow determine good steering wheel angles
void setJointParameters_arms_relaxed(atlas_msgs::AtlasCommand *jc);

// Sets the PID gains for right arm driving
void setJointParameters_right_arm_driving(atlas_msgs::AtlasCommand *jc);

// Initializes / zeroes out a field_command message
void zeroFieldCommand(wrecs_msgs::field_command *fieldCommand);

// some joint definitions to make code more readable
// these may also be available elsewhere...
const static int back_lbz  = 0;
const static int back_mby  = 1;
const static int back_ubx  = 2;
const static int neck_ay   = 3;
const static int l_leg_uhz = 4;
const static int l_leg_mhx = 5;
const static int l_leg_lhy = 6;
const static int l_leg_kny = 7;
const static int l_leg_uay = 8;
const static int l_leg_lax = 9;
const static int r_leg_uhz = 10;
const static int r_leg_mhx = 11;
const static int r_leg_lhy = 12;
const static int r_leg_kny = 13;
const static int r_leg_uay = 14;
const static int r_leg_lax = 15;
const static int l_arm_usy = 16;
const static int l_arm_shx = 17;
const static int l_arm_ely = 18;
const static int l_arm_elx = 19;
const static int l_arm_uwy = 20;
const static int l_arm_mwx = 21;
const static int r_arm_usy = 22;
const static int r_arm_shx = 23;
const static int r_arm_ely = 24;
const static int r_arm_elx = 25;
const static int r_arm_uwy = 26;
const static int r_arm_mwx = 27;



enum GAINS {
    GAINS_DEFAULT,      // default gain set 0.4 sec acc, 0.8 sec dec
    GAINS_1,
    GAINS_2,
    GAINS_3,
    GAINS_4,
    GAINS_5,
    GAINS_VFAST_ACC,    // 50mS acceleration / deceleration
    GAINS_FAST_ACC,     // 0.2 sec acc, 0.3 sec dec
    GAINS_SLOW_ACC,     // 0.5 sec acc, 1.1 sec dec
    GAINS_9,
    GAINS_10,
    GAINS_11,
    GAINS_12,
    GAINS_13,
    GAINS_14,
    GAINS_15,
    GAINS_16,
    GAINS_17,
    GAINS_18,
    GAINS_19,
    GAINS_LAD_DS_STIFF,  // ladder gains, double support, stiff arms
    GAINS_LAD_DS_RAIL,   // Double support, two arms holding railing
    GAINS_LAD_LFSS_RAIL, // left foot single support, arms on railings
    GAINS_LAD_RFSS_RAIL, // right foot single support, arms on railings
    GAINS_LAD_DS_COMPLI, // double support, compliant arms
    GAINS_LADDER_LATER,  // reserved for future use
    GAINS_26,
    GAINS_27,
    GAINS_28,
    GAINS_29,
    GAINS_30,
    GAINS_MOVEIT,
    GAINS_32
};


#endif
