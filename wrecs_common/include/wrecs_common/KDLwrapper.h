#ifndef KDLWRAPPER_H
#define KDLWRAPPER_H

#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include <string>

enum KDL_HAND_TYPE {
    KDL_HAND_TYPE_ROBOTIQ,    // centered on palm of Robotiq hand
    KDL_HAND_TYPE_DRC_TRIALS, // 1.5cm tube centered on wrist as used in trials
    KDL_HAND_TYPE_SPIKE,      // 20cm x 2cm Al cylinder held in Robtiq hand - same orientation as trials
    KDL_HAND_TYPE_EXT_20CM,   // Steering extension on side, held in Robotiq hand
    KDL_HAND_TYPE_SIDE_PIPE5   // Pipe held in hand - comes out side - hand_suffix:="_driving_hands5"
};


// Wrappers for some KDL and KDL derived functions
class KDLwrapper
{
public:
    KDLwrapper();
    ~KDLwrapper();
    KDLwrapper(std::string urdfLocation);

    // Forward kinematics functions
    void leftCameraFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame);
    void rightLegFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame);
    void leftLegFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame);
    void rightArmFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame, KDL_HAND_TYPE hand_type = KDL_HAND_TYPE_ROBOTIQ);
    void leftArmFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame, KDL_HAND_TYPE hand_type = KDL_HAND_TYPE_ROBOTIQ);
    void calculateCenterOfMass(atlas_msgs::AtlasState initState, KDL::Vector *com);

    // Inverse kinematics functions
    double rightLegIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out);
    double leftLegIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out);
    double rightArmIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type = KDL_HAND_TYPE_ROBOTIQ, int mode = 0);
    double leftArmIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type = KDL_HAND_TYPE_ROBOTIQ, int mode = 0);
    double leftWristIK_steering(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type = KDL_HAND_TYPE_ROBOTIQ, int mode = 0);


private:
    void handTypeAdjustment(KDL::Vector &hand_vector, KDL_HAND_TYPE hand_type);
    KDL::Tree my_tree;
    int my_tree_initialized;
};

#endif

