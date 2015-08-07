// The purpose of this simple node is to demonstrate how to control the robot
// It switchs to manipulate mode, moves the arms up and rotates
// an imaginary valve a partial turn

// This node will run correctly offline if the robot is in stand mode

// Running with the robot you will need to:
// start the pump
// change to stand prep
// lower the robot
// change to stand
// Then you can run this node

#include <ros/ros.h>
#include <wrecs_msgs/field_command.h>
#include <wrecs_msgs/field_state.h>
#include <WRECS_Common.h>

static ros::Publisher pub_field_command;
static wrecs_msgs::field_state G_fieldState;
static void send_then_wait(const char * comments, wrecs_msgs::field_command *wfc);

    
// Move the arms using the field_command ROS interface
static void moveArms()
{
    wrecs_msgs::field_command myFieldCommand;
    zeroFieldCommand(&myFieldCommand);

    // Change the robot mode to user mode
    myFieldCommand.setBDImode = true;
    myFieldCommand.BDImode = AtlasRobot::BEHAVIOR_MANIPULATE;
    pub_field_command.publish(myFieldCommand);
    myFieldCommand.setBDImode = false;
    usleep(2000000); // wait for robot to switch to manipulate mode

    // Move the arms up
    myFieldCommand.position[l_arm_shx] = -0.7;
    myFieldCommand.position[r_arm_shx] =  0.7;
    send_then_wait("Move arms up", &myFieldCommand);

    // Move arms forward
    myFieldCommand.position[l_arm_usy] = -1.5;
    myFieldCommand.position[l_arm_ely] =  0.5;
    myFieldCommand.position[l_arm_elx] =  0.3;
    myFieldCommand.position[r_arm_usy] = -1.5;
    myFieldCommand.position[r_arm_ely] =  0.5;
    myFieldCommand.position[r_arm_elx] = -0.3;
    send_then_wait("Move arms forward", &myFieldCommand);

    // Rotate imaginary valve
    myFieldCommand.position[l_arm_usy] = -1.6;
    myFieldCommand.position[l_arm_shx] = -0.5;
    myFieldCommand.position[l_arm_ely] =  1.5;
    myFieldCommand.velocity[l_arm_ely] =  0.7;
    myFieldCommand.position[l_arm_uwy] = -0.2;
    myFieldCommand.velocity[l_arm_uwy] =  0.5;

    myFieldCommand.position[r_arm_usy] = -1.5;
    myFieldCommand.position[r_arm_shx] =  1.1;
    myFieldCommand.position[r_arm_ely] =  0.1;
    myFieldCommand.position[r_arm_uwy] = -0.3;    
    myFieldCommand.velocity[r_arm_uwy] =  0.5;    
    send_then_wait("Rotate Imaginary Valve", &myFieldCommand);
}

static void fieldStateCallback(wrecs_msgs::field_state msg)
{
  G_fieldState = msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;
    pub_field_command = n.advertise<wrecs_msgs::field_command>("/field_command", 10);
    ros::Subscriber substate = n.subscribe("/field_state", 1, fieldStateCallback);

    usleep(1000000); // wait for subscriptions to start up

    // Move the arms
    moveArms();

    return 0;
}


// Code beneath this point consists of some helper functions

// Make sure the field command message is set to a known state
void zeroFieldCommand(wrecs_msgs::field_command *fieldCommand)
{
    fieldCommand->sendJointCommand = 0;
    fieldCommand->set_gain_set = 0;
    fieldCommand->gain_set = 0;
    fieldCommand->setBDImode = 0;
    fieldCommand->BDImode = 0;
    fieldCommand->which_foot = -1;
    fieldCommand->x = 0.0;
    fieldCommand->y = 0.0;
    fieldCommand->z = 0.0;
    fieldCommand->qx = 0.0;
    fieldCommand->qy = 0.0;
    fieldCommand->qz = 0.0;
    fieldCommand->qw = 0.0;
    fieldCommand->set_control_mode = 0;
    fieldCommand->control_mode = 0;
    fieldCommand->pelvis_height = 0.8;
    fieldCommand->pelvis_yaw = 0.0;
    fieldCommand->pelvis_lat = 0.0;

    for (int i = 0; i < 28; i++) {
	fieldCommand->setThisJoint[i] = 0;
	fieldCommand->position[i] = 0.0;
	fieldCommand->velocity[i] = 0.0;
	fieldCommand->effort[i] = 0.0;
    }
}

// Wait for the velocity controller to indicate a movement is complete
static void wait_for_movement_complete()
{
    int zero_msg = G_fieldState.header.seq;
    while(((G_fieldState.header.seq - zero_msg) < 2) |
	  !G_fieldState.movement_complete) {
	ros::spinOnce();
	usleep(1000);
    }
}

// Send data to the velocity controller without waiting for it to complete
static void send_no_wait(const char * comments, wrecs_msgs::field_command *wfc)
{
    // Set this so the commands are actually sent by the  field_state process
    wfc->sendJointCommand = 1;

    std::cout << comments << std::endl;
    //sendString(comments);
    for (int i = 0; i < 28 ; i++) {
	if (fabs(wfc->position[i]) > 0.0001) {
	    wfc->k_effort[i] = 255;
	    wfc->setThisJoint[i] = 1;
	}
    }
    wfc->header.stamp = ros::Time::now();
    pub_field_command.publish(*wfc);    
}


// Send a message to the velocity controller and wait for it to complete
static void send_then_wait(const char * comments, wrecs_msgs::field_command *wfc)
{
    wait_for_movement_complete();
    send_no_wait(comments, wfc);
    wait_for_movement_complete();
}
