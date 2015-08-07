#include <math.h>
#include <sstream>
#include <termio.h>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>
#include <atlas_msgs/AtlasCommand.h>
#include <wrecs_common/WRECS_Common.h>


double getTimeDouble()
{
    
    // A lower level alternative is rdtsc
    timeval tv1;  // members are tv_sec and tv_usec
    struct  timezone tz1;
    tz1.tz_dsttime = 0;
    tz1.tz_minuteswest = 0;
    gettimeofday(&tv1, &tz1);
    return (((double)tv1.tv_sec) + 0.000001 * tv1.tv_usec);
    

    /*
    // Alternate implementation
    struct timespec ts1;
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    return (((double)ts1.tv_sec) + 0.000000001 * ts1.tv_nsec);
    */

    // Alternate 2
    //struct timespec ts1;
    //clock_gettime(CLOCK_MONOTONIC_RAW, &ts1);
    //return (((double)ts1.tv_sec) + 0.000000001 * ts1.tv_nsec);

}

// Pass in the desired description
// returns the full path with date pre-pended to the description
// example
// input
//   sometext.txt
// return value
//   /logs/2013_08_01_15_33_24_sometext.txt
const std::string nameLogFile(const char* description_dot_csv)
{
    char tmpString[2000];
    time_t     now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    strftime(tmpString, sizeof(tmpString), "/logs/%Y_%m_%d_%H_%M_%S_", &tstruct);
    strncat(tmpString, description_dot_csv, 1000);

    return tmpString;
}



// Correctly sizes the arrays in an AtlasCommand
void resizeAtlasCommand(atlas_msgs::AtlasCommand *ac)
{
    const static unsigned int n = 28;
    ac->position.resize(n);
    ac->velocity.resize(n);
    ac->effort.resize(n);
    ac->kp_position.resize(n);
    ac->ki_position.resize(n);
    ac->kd_position.resize(n);
    ac->kp_velocity.resize(n);
    ac->i_effort_min.resize(n);
    ac->i_effort_max.resize(n);
    ac->k_effort.resize(n);
}


void zeroAtlasCommand(atlas_msgs::AtlasCommand *ac)
{
    for (int i = 0; i<28; i++) {
	ac->position[i] = 0.0;
	ac->velocity[i] = 0.0;
	ac->effort[i] = 0.0;
	ac->kp_position[i] = 0.0;
	ac->ki_position[i] = 0.0;
	ac->kd_position[i] = 0.0;
	ac->kp_velocity[i] = 0.0;
	ac->i_effort_min[i] = 0.0;
	ac->i_effort_max[i] = 0.0;
	ac->k_effort[i] = 255; // This is the default value
	//ac->k_effort[i] = 0;
    }
}

// Sets PID gains for the joints
void setJointParameters(atlas_msgs::AtlasCommand *jc)
{
    // body
    jc->kp_position[0] = 4.0*20.0; // was 20.0
    jc->kp_position[1] = 4000.0;
    jc->kp_position[2] = 1000.0;
    jc->kp_position[3] = 4.0*20.0;

    jc->ki_position[0] = 40.0;
    jc->ki_position[1] = 2000.0;
    jc->ki_position[2] = 500.0;
    jc->ki_position[3] = 10.0;

    jc->i_effort_max[0] = 124.0; // base values are max effort from urdf
    jc->i_effort_max[1] = 206.0;
    jc->i_effort_max[2] = 94.0;
    jc->i_effort_max[3] = 5.0;

    jc->kd_position[0] = 100.0; // was 1.0;
    jc->kd_position[1] = 100.0; // was 2.0;
    jc->kd_position[2] = 100.0; // 1.0;
    jc->kd_position[3] = 10.0; // was 0.1

    jc->kp_velocity[0] = 10.0;
    jc->kp_velocity[1] = 400.0;
    jc->kp_velocity[2] = 100.0;
    jc->kp_velocity[3] = 0.0;

    jc->velocity[0] = 0.2;
    jc->velocity[1] = 0.2;
    jc->velocity[2] = 0.2;
    jc->velocity[3] = 2.0;

    // left leg
    jc->kp_position[4] = 100.0;   // was 5.0
    jc->kp_position[5] = 8.0*100.0;
    jc->kp_position[6] = 2000.0;
    jc->kp_position[7] = 1000.0;
    jc->kp_position[8] = 900.0;
    jc->kp_position[9] = 2.0*300.0;

    jc->ki_position[4] = 50.0;
    jc->ki_position[5] = 100.0;
    jc->ki_position[6] = 1000.0;
    jc->ki_position[7] = 500.0;
    jc->ki_position[8] = 30.0;
    jc->ki_position[9] = 20.0;

    jc->i_effort_max[4] = 0.2*110.0; // base values are max effort from urdf
    jc->i_effort_max[5] = 0.2*180.0;
    jc->i_effort_max[6] = 0.2*260.0;
    jc->i_effort_max[7] = 0.2*220.0;
    jc->i_effort_max[8] = 0.2*220.0;
    jc->i_effort_max[9] = 0.2*90.0;

    jc->kd_position[4] = 0.8; // was 0.01; 1.0
    jc->kd_position[5] = 4.0; // was 1.0; 5.0
    jc->kd_position[6] = 100.0; // was 10.0;
    jc->kd_position[7] = 100.0; // was 10.0;
    jc->kd_position[8] = 16.0; // was 2.0; - unstable at 20.0 with no weight no feet
    jc->kd_position[9] = 8.0; // was 1.0; - unstable at 10.0 with no weight on feet

    jc->kp_velocity[4] = 0.0;
    jc->kp_velocity[5] = 0.0;
    jc->kp_velocity[6] = 0.0;
    jc->kp_velocity[7] = 0.0;
    jc->kp_velocity[8] = 0.0;
    jc->kp_velocity[9] = 0.0;

    jc->velocity[4] = 0.3;
    jc->velocity[5] = 0.3;
    jc->velocity[6] = 0.3;
    jc->velocity[7] = 0.3;
    jc->velocity[8] = 0.1;
    jc->velocity[9] = 0.1;

    // left arm
    jc->kp_position[16] = 2000.0;
    jc->kp_position[17] = 1000.0; // unstable - 3Hz at 2000, somewhat unstable at 1000 with i=0,d=0
    jc->kp_position[18] = 200.0;
    jc->kp_position[19] = 200.0;
    jc->kp_position[20] = 50.0;
    jc->kp_position[21] = 100.0;

    jc->ki_position[16] = 1000.0;
    jc->ki_position[17] = 500.0;
    jc->ki_position[18] = 100.0; // was 100.0
    jc->ki_position[19] = 100.0;
    jc->ki_position[20] = 25.0;
    jc->ki_position[21] = 50.0;

    jc->i_effort_max[16] = 212.0; // base values are max effort from urdf
    jc->i_effort_max[17] = 170.0;
    jc->i_effort_max[18] = 114.0; // was 114.0
    jc->i_effort_max[19] = 114.0;
    jc->i_effort_max[20] = 114.0;
    jc->i_effort_max[21] = 60.0;

    jc->kd_position[16] = 20.0; // was 3.0;
    jc->kd_position[17] = 40.0; // was 20.0;
    jc->kd_position[18] = 10.0; // was 3.0;
    jc->kd_position[19] = 10.0; // was 3.0;
    jc->kd_position[20] =  2.0; // was 0.1;
    jc->kd_position[21] =  3.0; // was 0.2;

    jc->kp_velocity[16] = 400.0;
    jc->kp_velocity[17] = 200.0;
    jc->kp_velocity[18] = 40.0;
    jc->kp_velocity[19] = 40.0;
    jc->kp_velocity[20] = 0.0;
    jc->kp_velocity[21] = 0.0;

    jc->velocity[16] = 0.5;
    jc->velocity[17] = 0.5;
    jc->velocity[18] = 0.6;
    jc->velocity[19] = 0.6;
    jc->velocity[20] = 1.9;
    jc->velocity[21] = 1.9;

    // Copy left leg values to right leg
    for (int i = 4; i < 10; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy left arm values to right arm
    for (int i = 16; i < 22; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy i_effort_max (and invert) to i_effort_min
    // Zero out kp_velocity - causes jitters
    for (int i = 0; i < 28; i++) {
	jc->kp_velocity[i] = 0.0;

	jc->i_effort_min[i] = -jc->i_effort_max[i];
    }
}


// Sets PID gains for the joints
void setJointParameters_leftArmOff(atlas_msgs::AtlasCommand *jc)
{
    setJointParameters_lowerGains(jc);

    // left arm
    /*
    jc->kp_position[16] = 0.0;
    jc->kp_position[17] = 0.0;
    jc->kp_position[18] = 0.0;

    jc->ki_position[16] = 0.0;
    jc->ki_position[17] = 0.0;
    jc->ki_position[18] = 0.0;

    jc->kd_position[16] =  0.0;
    jc->kd_position[17] =  0.0;
    jc->kd_position[18] =  0.0;

    jc->i_effort_max[16] = 0.0;
    jc->i_effort_max[17] = 0.0;
    jc->i_effort_max[18] = 0.0;

    jc->i_effort_min[16] = 0.0;
    jc->i_effort_min[17] = 0.0;
    jc->i_effort_min[18] = 0.0;
    */

    
    for (int i = 22; i < 28; i++) {
	jc->kp_position[i] = 0.0;
	jc->ki_position[i] = 0.0;
	jc->kd_position[i] =  0.0;
	jc->i_effort_max[i] = 0.0;
	jc->i_effort_min[i] = 0.0;
	jc->kp_velocity[i] = 0.0;
    }
    

}

// Sets PID gains for the joints
void setJointParameters_lowerGains(atlas_msgs::AtlasCommand *jc)
{
    // body
    jc->kp_position[0] = 20.0;
    jc->kp_position[1] = 4000.0;
    jc->kp_position[2] = 1000.0;
    jc->kp_position[3] = 20.0;

    jc->ki_position[0] = 40.0;
    jc->ki_position[1] = 2000.0;
    jc->ki_position[2] = 500.0;
    jc->ki_position[3] = 5.0;

    jc->i_effort_max[0] = 124.0; // base values are max effort from urdf
    jc->i_effort_max[1] = 206.0;
    jc->i_effort_max[2] = 94.0;
    jc->i_effort_max[3] = 5.0;

    jc->kd_position[0] = 30.0; // was 1.0;
    jc->kd_position[1] = 280.0; // was 2.0;
    jc->kd_position[2] = 30.0; // 1.0;
    jc->kd_position[3] = 1.0; // was 0.1

    jc->kp_velocity[0] = 10.0;
    jc->kp_velocity[1] = 400.0;
    jc->kp_velocity[2] = 100.0;
    jc->kp_velocity[3] = 0.0;

    jc->velocity[0] = 0.2;
    jc->velocity[1] = 0.2;
    jc->velocity[2] = 0.2;
    jc->velocity[3] = 2.0;

    // left leg
    jc->kp_position[4] = 50.0;   // was 5.0
    jc->kp_position[5] = 2.0*100.0;
    jc->kp_position[6] = 2000.0;
    jc->kp_position[7] = 1000.0;
    jc->kp_position[8] = 900.0;
    jc->kp_position[9] = 300.0;

    jc->ki_position[4] = 20.0;
    jc->ki_position[5] = 100.0;
    jc->ki_position[6] = 1000.0;
    jc->ki_position[7] = 500.0;
    jc->ki_position[8] = 30.0;
    jc->ki_position[9] = 20.0;

    jc->i_effort_max[4] = 0.2*110.0; // base values are max effort from urdf
    jc->i_effort_max[5] = 0.2*180.0;
    jc->i_effort_max[6] = 0.2*260.0;
    jc->i_effort_max[7] = 0.2*220.0;
    jc->i_effort_max[8] = 0.2*220.0;
    jc->i_effort_max[9] = 0.2*90.0;

    jc->kd_position[4] = 0.2; // was 0.01;
    jc->kd_position[5] = 1.0;
    jc->kd_position[6] = 10.0;
    jc->kd_position[7] = 10.0;
    jc->kd_position[8] = 2.0;
    jc->kd_position[9] = 1.0;

    jc->kp_velocity[4] = 0.0;
    jc->kp_velocity[5] = 0.0;
    jc->kp_velocity[6] = 0.0;
    jc->kp_velocity[7] = 0.0;
    jc->kp_velocity[8] = 0.0;
    jc->kp_velocity[9] = 0.0;

    jc->velocity[4] = 0.3;
    jc->velocity[5] = 0.3;
    jc->velocity[6] = 0.3;
    jc->velocity[7] = 0.3;
    jc->velocity[8] = 0.3;
    jc->velocity[9] = 0.3;

    // left arm
    jc->kp_position[16] = 2000.0;
    jc->kp_position[17] = 1000.0; // unstable - 3Hz at 2000, somewhat unstable at 1000 with i=0,d=0
    jc->kp_position[18] = 200.0;
    jc->kp_position[19] = 200.0;
    jc->kp_position[20] = 50.0;
    jc->kp_position[21] = 100.0;

    jc->ki_position[16] = 1000.0;
    jc->ki_position[17] = 500.0;
    jc->ki_position[18] = 100.0;
    jc->ki_position[19] = 100.0;
    jc->ki_position[20] = 25.0;
    jc->ki_position[21] = 50.0;

    jc->i_effort_max[16] = 212.0; // base values are max effort from urdf
    jc->i_effort_max[17] = 170.0;
    jc->i_effort_max[18] = 114.0; // was 114.0
    jc->i_effort_max[19] = 114.0;
    jc->i_effort_max[20] = 114.0;
    jc->i_effort_max[21] = 60.0;

    jc->kd_position[16] = 10.0; // was 3.0;
    jc->kd_position[17] = 40.0; // was 20.0;
    jc->kd_position[18] =  6.0; // was 3.0;
    jc->kd_position[19] =  6.0; // was 3.0;
    jc->kd_position[20] =  1.0; // was 0.1;
    jc->kd_position[21] =  2.0; // was 0.2;

    jc->kp_velocity[16] = 400.0;
    jc->kp_velocity[17] = 200.0;
    jc->kp_velocity[18] = 40.0;
    jc->kp_velocity[19] = 40.0;
    jc->kp_velocity[20] = 0.0;
    jc->kp_velocity[21] = 0.0;

    jc->velocity[16] = 0.5;
    jc->velocity[17] = 0.5;
    jc->velocity[18] = 0.6;
    jc->velocity[19] = 0.6;
    jc->velocity[20] = 1.9;
    jc->velocity[21] = 1.9;

    // Copy left leg values to right leg
    for (int i = 4; i < 10; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy left arm values to right arm
    for (int i = 16; i < 22; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy i_effort_max (and invert) to i_effort_min
    // Zero out kp_velocity - causes jitters
    for (int i = 0; i < 28; i++) {
	jc->kp_velocity[i] = 0.0;

	jc->i_effort_min[i] = -jc->i_effort_max[i];
    }
}

// Sets PID gains for the joints
void setJointParameters_arms_relaxed(atlas_msgs::AtlasCommand *jc)
{
    setJointParameters(jc);
    jc->i_effort_max[l_leg_uhz] = 0.7*110.0;
    jc->i_effort_min[l_leg_uhz] = -0.7*110.0;

    /*
    jc->effort[l_arm_usy] =  0.1;
    jc->effort[l_arm_shx] =  0.1;
    jc->effort[l_arm_ely] = -0.1;
    jc->effort[l_arm_elx] = -0.1;

    jc->effort[r_arm_usy] =  0.1;
    jc->effort[r_arm_shx] = -0.1;
    jc->effort[r_arm_ely] = -0.1;
    jc->effort[r_arm_elx] =  0.1;
    */

    // Copy left arm values to right arm
    for (int i = 16; i < 27; i++) {
	jc->kp_position[i]  *= 0.3;
	jc->ki_position[i]  *= 0.3;
	jc->i_effort_max[i] *= 0.3;
	jc->i_effort_min[i] *= 0.3;
	jc->kd_position[i]  *= 0.3;
	jc->effort[i]       = 0.0;
	jc->k_effort[i]     = 255;
    }    
    for (int i = 16; i < 27; i++) {
	jc->kp_position[i]   = 0.0;
	jc->ki_position[i]   = 0.0;
	jc->i_effort_max[i]  = 0.0;
	jc->i_effort_min[i]  = 0.0;
	jc->kd_position[i]   = 0.0;
	jc->effort[i]        = 0.0;
	jc->k_effort[i]      = 255;
    }
}


void setJointParameters_right_arm_driving(atlas_msgs::AtlasCommand *jc)
{
    setJointParameters(jc);
    jc->i_effort_max[l_leg_uhz] = 0.7*110.0;
    jc->i_effort_min[l_leg_uhz] = -0.7*110.0;


    /*
    jc->ki_position[r_arm_usy] *= 0.5;
    jc->ki_position[r_arm_shx] *= 0.5;
    jc->ki_position[r_arm_ely] *= 0.5;
    jc->ki_position[r_arm_elx] *= 0.5;
    jc->ki_position[r_arm_uwy] *= 0.5;
    jc->ki_position[r_arm_mwx] *= 0.5;

    jc->i_effort_max[r_arm_usy] = 0.3;
    jc->i_effort_max[r_arm_shx] = 0.4;
    jc->i_effort_max[r_arm_ely] = 0.2;
    jc->i_effort_max[r_arm_elx] = 0.2;
    jc->i_effort_max[r_arm_uwy] = 0.1;
    jc->i_effort_max[r_arm_mwx] = 0.1;

    jc->i_effort_min[r_arm_usy] = 0.3;
    jc->i_effort_min[r_arm_shx] = 0.4;
    jc->i_effort_min[r_arm_ely] = 0.2;
    jc->i_effort_min[r_arm_elx] = 0.2;
    jc->i_effort_min[r_arm_uwy] = 0.1;
    jc->i_effort_min[r_arm_mwx] = 0.1;
    */
}

// Sets PID gains for the joints back to the defaults
void setJointParametersDefault(atlas_msgs::AtlasCommand *jc)
{
    // body
    jc->kp_position[0] = 20.0;
    jc->kp_position[1] = 4000.0;
    jc->kp_position[2] = 2000.0;
    jc->kp_position[3] = 20.0;

    jc->kd_position[0] = 0.1;
    jc->kd_position[1] = 2.0;
    jc->kd_position[2] = 1.0;
    jc->kd_position[3] = 1.0;

    // left leg
    jc->kp_position[4] = 5.0;
    jc->kp_position[5] = 100.0;
    jc->kp_position[6] = 2000.0;
    jc->kp_position[7] = 1000.0;
    jc->kp_position[8] = 900.0;
    jc->kp_position[9] = 300.0;

    jc->kd_position[4] = 0.01;
    jc->kd_position[5] = 1.0;
    jc->kd_position[6] = 10.0;
    jc->kd_position[7] = 10.0;
    jc->kd_position[8] = 2.0;
    jc->kd_position[9] = 1.0;

    // left arm
    jc->kp_position[16] = 2000.0;
    jc->kp_position[17] = 1000.0;
    jc->kp_position[18] = 200.0;
    jc->kp_position[19] = 200.0;
    jc->kp_position[20] = 50.0;
    jc->kp_position[21] = 100.0;

    jc->kd_position[16] = 3.0;
    jc->kd_position[17] = 20.0;
    jc->kd_position[18] = 3.0;
    jc->kd_position[19] = 3.0;
    jc->kd_position[20] = 0.1;
    jc->kd_position[21] = 0.2;

    // Copy left leg values to right leg
    for (int i = 4; i < 10; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy left arm values to right arm
    for (int i = 16; i < 22; i++) {
	jc->kp_position[i+6]  = jc->kp_position[i];
	jc->ki_position[i+6]  = jc->ki_position[i];
	jc->i_effort_max[i+6] = jc->i_effort_max[i];
	jc->kd_position[i+6]  = jc->kd_position[i];
	jc->kp_velocity[i+6]  = jc->kp_velocity[i];
	jc->velocity[i+6]     = jc->velocity[i];
    }
    // Copy i_effort_max (and invert) to i_effort_min
    // Zero out kp_velocity - causes jitters
    for (int i = 0; i < 28; i++) {
	jc->kp_velocity[i] = 0.0;
	jc->velocity[i] = 100.0; // This is not default, but bypasses the velocity loop
	jc->ki_position[i] = 0.0;
	jc->i_effort_max[i] = 0.0;

	jc->i_effort_min[i] = -jc->i_effort_max[i];
    }
}


void zeroFieldCommand(wrecs_msgs::field_command *fieldCommand)
{
    fieldCommand->sendJointCommand = 0;
    fieldCommand->set_gain_set = 0;
    fieldCommand->gain_set = 0;
    fieldCommand->setBDImode = 0;
    fieldCommand->BDImode = 0;
    fieldCommand->set_control_mode = 0;
    fieldCommand->control_mode = 0;

    for (int i = 0; i < 28; i++) {
        fieldCommand->setThisJoint[i] = 0;
        fieldCommand->position[i] = 0.0;
        fieldCommand->velocity[i] = 0.0;
        fieldCommand->effort[i] = 0.0;
    }
}

