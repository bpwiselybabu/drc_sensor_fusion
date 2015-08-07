#include <math.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termio.h>
#include <stdio.h>

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>

#include <wrecs_common/KDLwrapper.h>
#include <kdl/frames_io.hpp>

#include <time.h>



KDLwrapper kdl_wrapper;



// Test out the functions in the KDL wrapper
// Test a few forward kinematics for each joint
// Test a few reverse kinematics for each joint
// Test a few COM states

// Necessary further test - initial position different than final position

// Further tests:
// Random frames - frame to pose and back to frame
// Random poses - within min & max - pose to frame and back
// fixed frames
// timing (how long it takes in seconds)

static void zeroJoints(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "zeroJoints" << std::endl;
    for (int i = 0; i < 28; i++) {
	as1->position[i] = 0.0;
    }
}

static void minJoints(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "minJoints" << std::endl;
    as1->position[0] = -0.610685;
    as1->position[1] = -1.2;
    as1->position[2] = -0.790809;
    as1->position[3] = -0.610865238;

    as1->position[4] = -0.32;
    as1->position[5] = -0.47;
    as1->position[6] = -1.75;
    as1->position[7] = 0.0;
    as1->position[8] = -0.698;
    as1->position[9] = -0.436;

    as1->position[10] = -1.14;
    as1->position[11] = -0.495;
    as1->position[12] = -0.524;
    as1->position[13] = 0.0;
    as1->position[14] = -0.698;
    as1->position[15] = -0.436;

    as1->position[16] = -1.96;
    as1->position[17] = -1.4;
    as1->position[18] = 0.0;
    as1->position[19] = 0.0;
    as1->position[20] = -1.571;
    as1->position[21] = -0.436;

    as1->position[22] = -1.96;
    as1->position[23] = -1.75;
    as1->position[24] = 0.0;
    as1->position[25] = -2.36;
    as1->position[26] = -1.571;
    as1->position[27] = -1.571;
}

static void maxJoints(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "maxJoints" << std::endl;
    as1->position[0] = 0.610865;
    as1->position[1] = 1.28;
    as1->position[2] = 0.790809;
    as1->position[3] = 1.13446401;

    as1->position[4] = 1.14;
    as1->position[5] = 0.495;
    as1->position[6] = 0.524;
    as1->position[7] = 2.45;
    as1->position[8] = 0.698;
    as1->position[9] = 0.436;

    as1->position[10] = 0.32;
    as1->position[11] = 0.47;
    as1->position[12] = 1.745;
    as1->position[13] = 2.45;
    as1->position[14] = 0.698;
    as1->position[15] = 0.436;

    as1->position[16] = 1.96;
    as1->position[17] = 1.75;
    as1->position[18] = 3.14159;
    as1->position[19] = 2.36;
    as1->position[20] = 1.571;
    as1->position[21] = 1.571;

    as1->position[22] = 1.96;
    as1->position[23] = 1.4;
    as1->position[24] = 3.14159;
    as1->position[25] = 0.0;
    as1->position[26] = 1.571;
    as1->position[27] = 0.436;
}

static void bdistand(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "bdistand joints" << std::endl;
    as1->position[0] = 0.0;
    as1->position[1] = 0.015;
    as1->position[2] = 0.0;
    as1->position[3] = 0.0;
    as1->position[4] = 0.0;
    as1->position[5] = 0.062;
    as1->position[6] = -0.233;
    as1->position[7] = 0.518;
    as1->position[8] = -0.276;
    as1->position[9] = 0.062;
    as1->position[10] = 0.0;
    as1->position[11] = -0.062;
    as1->position[12] = -0.233;
    as1->position[13] = 0.518;
    as1->position[14] = -0.276;
    as1->position[15] = 0.062;
    as1->position[16] = 0.3;
    as1->position[17] = -1.3;
    as1->position[18] = 2.0;
    as1->position[19] = 0.5;
    as1->position[20] = 0.0;
    as1->position[21] = 0.0;
    as1->position[22] = 0.3;
    as1->position[23] = 1.3;
    as1->position[24] = 2.0;
    as1->position[25] = -0.5;
    as1->position[26] = 0.0;
    as1->position[27] = 0.0;
}

static void handsfront(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "handsfront joints" << std::endl;
    as1->position[0] =  0.0;
    as1->position[1] = -0.1;
    as1->position[2] = 0.0;
    as1->position[3] = 0.0;
    as1->position[4] = 0.0;
    as1->position[5] = 0.0;
    as1->position[6] = 0.0;
    as1->position[7] = 0.0;
    as1->position[8] = 0.0;
    as1->position[9] = 0.0;
    as1->position[10] = 0.0;
    as1->position[11] = 0.0;
    as1->position[12] = 0.0;
    as1->position[13] = 0.0;
    as1->position[14] = 0.0;
    as1->position[15] = 0.0;
    as1->position[16] = -1.4;
    as1->position[17] = -1.0;
    as1->position[18] = 1.2;
    as1->position[19] = 0.6;
    as1->position[20] = -0.6;
    as1->position[21] = 0.5;
    as1->position[22] = -1.4;
    as1->position[23] = 1.0;
    as1->position[24] = 1.2;
    as1->position[25] = -0.6;
    as1->position[26] = -0.6;
    as1->position[27] = -0.5;
}


static void random1(atlas_msgs::AtlasState *as1)
{
    std::cout << std::endl <<  std::endl << std::endl << "random1 joints" << std::endl;

    as1->position[0] = 0.1;
    as1->position[1] = 0.3;
    as1->position[2] = -0.04;
    as1->position[3] = 0.0;

    as1->position[4] = -0.3;
    as1->position[5] = -0.2;
    as1->position[6] = 0.3;
    as1->position[7] = 2.0;
    as1->position[8] = -0.3;
    as1->position[9] = 0.2;

    as1->position[10] = 0.3;
    as1->position[11] = 0.4;
    as1->position[12] = 0.1;
    as1->position[13] = 0.0;
    as1->position[14] = 0.5;
    as1->position[15] = 0.3;

    as1->position[16] = 0.4;
    as1->position[17] = -1.0;
    as1->position[18] = 0.3;
    as1->position[19] = 0.3;
    as1->position[20] = -0.1;
    as1->position[21] = 0.3;
    as1->position[22] = -1.1;
    as1->position[23] = 0.1;
    as1->position[24] = 1.0;
    as1->position[25] = -0.3;
    as1->position[26] = 0.3;
    as1->position[27] = 0.4;
}

static void printPosition(atlas_msgs::AtlasState as1)
{
    for (int i = 0; i < 28; i++) {
	printf("%lf, ", as1.position[i]);
	if ((i == 3) || (i == 9) || (i == 15) || (i == 21) || (i == 27)) {
	    printf("\n");
	}
    }
}


static double getTimeDouble()
{
    // A lower level alternative is rdtsc
    timeval tv1;  // members are tv_sec and tv_usec
    struct  timezone tz1;
    tz1.tz_dsttime = 0;
    tz1.tz_minuteswest = 0;
    gettimeofday(&tv1, &tz1);
    return (((double)tv1.tv_sec) + 0.000001 * tv1.tv_usec);
}





// First parameter is used to test forward kinematics and as final inverse kinematics goal
// second parameter is used as initial position for inverse kinematics
static void ktest(atlas_msgs::AtlasState as1, atlas_msgs::AtlasState as2)
{
    KDL::Frame f1, f2, f3, f4;
    KDL::Vector v1;
    double t1, t2, t3, t4, t5, t6, t7, t8;

    // Body joints need to match between initial and final as the inverse
    // kinematics we are testing don't adjust body joints
    as1.position[0] = as2.position[0];
    as1.position[1] = as2.position[1];
    as1.position[2] = as2.position[2];
    as1.position[3] = as2.position[3];

    printPosition(as1);

    t1 = getTimeDouble();
    kdl_wrapper.calculateCenterOfMass(as1, &v1);
    t2 = getTimeDouble();
    std::cout << "Center of Mass " << v1 << std::endl;


    kdl_wrapper.leftLegFK(as1, &f1); 
    t3 = getTimeDouble();
    std::cout << f1 << std::endl;
    kdl_wrapper.rightLegFK(as1, &f2);
    std::cout << f2 << std::endl;
    kdl_wrapper.leftArmFK(as1, &f3);
    std::cout << f3 << std::endl;
    t4 = getTimeDouble();
    kdl_wrapper.rightArmFK(as1, &f4);
    t5 = getTimeDouble();
    std::cout << f4 << std::endl;

    atlas_msgs::AtlasCommand ac1;
    ac1.position.resize(28);
    ac1.position[0] = 0.0;
    ac1.position[1] = 0.0;
    ac1.position[2] = 0.0;
    ac1.position[3] = 0.0;

    kdl_wrapper.leftLegIK(as2, f1, &ac1);
    t6 = getTimeDouble();
    kdl_wrapper.rightLegIK(as2, f2, &ac1);
    t7 = getTimeDouble();
    kdl_wrapper.leftArmIK(as2, f3, &ac1);
    t8 = getTimeDouble();
    kdl_wrapper.rightArmIK(as2, f4, &ac1);

    atlas_msgs::AtlasState as3;
    as3.position.resize(28);

    atlas_msgs::AtlasState as4;
    as4.position.resize(28);

    for (int i = 0; i < 28; i++) {
	as3.position[i] = ac1.position[i];
	as4.position[i] = ac1.position[i] - as1.position[i];
    }
    printf("Times %lf %lf %lf %lf %lf\n", t2-t1, t3-t2, t5-t4, t7-t6, t8-t7);
    printPosition(as2);
    printPosition(as3);
    printPosition(as4);
}

int main(int argc, char** argv)
{
    //KDLwrapper kdl_wrapper;
    
    atlas_msgs::AtlasState as1;
    as1.position.resize(28);
    atlas_msgs::AtlasState as2;
    as2.position.resize(28);

    zeroJoints(&as1);
    zeroJoints(&as2);
    ktest(as1, as2);

    minJoints(&as1);
    minJoints(&as2);
    ktest(as1, as2);

    maxJoints(&as1);
    maxJoints(&as2);
    ktest(as1, as2);

    bdistand(&as1);
    bdistand(&as2);
    ktest(as1, as2);

    handsfront(&as1);
    handsfront(&as2);
    ktest(as1, as2);

    random1(&as1);
    random1(&as2);
    ktest(as1, as2);


    zeroJoints(&as1);
    random1(&as2);
    ktest(as1, as2);

    minJoints(&as1);
    zeroJoints(&as2);
    ktest(as1, as2);

    maxJoints(&as1);
    bdistand(&as2);
    ktest(as1, as2);

    bdistand(&as1);
    random1(&as2);
    ktest(as1, as2);

    handsfront(&as1);
    maxJoints(&as2);
    ktest(as1, as2);

    random1(&as1);
    minJoints(&as2);
    ktest(as1, as2);

    return 0;
}
