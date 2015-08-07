#include <wrecs_common/KDLwrapper.h>
#include <wrecs_common/AtlasLimits.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <wrecs_common/chainiksolverpos_nr_jl2.h>
#include <kdl/frames_io.hpp>
#include <kdl/treefksolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/package.h>

using namespace KDL;
static Vector g_footvector(0.094, 0.0, -0.084);



KDLwrapper::KDLwrapper()
{
    //std::string urdffile1 = ros::package::getPath("wrecs_common") + "/urdf/atlas_v4.urdf";
    std::string urdffile1 = ros::package::getPath("wrecs_common") + "/urdf/atlas_v4_for_kdl.urdf";
    my_tree_initialized = 1; 
    if (!kdl_parser::treeFromFile(urdffile1, my_tree)) {
        std::cout << "Failed to construct kdl tree" << std::endl;
	my_tree_initialized = 0;
	//return false;
    }
}

KDLwrapper::KDLwrapper(std::string urdfLocation)
{
    my_tree_initialized = 1;
    if (!kdl_parser::treeFromFile(urdfLocation, my_tree)){
	std::cout << "Failed to construct kdl tree" << std::endl;
	my_tree_initialized = 0;
	//return false;
    }
}


KDLwrapper::~KDLwrapper()
{
}

void KDLwrapper::handTypeAdjustment(Vector &hand_vector, KDL_HAND_TYPE hand_type)
{
    hand_vector = Vector(0.0, 0.2, 0.0); // Set a default value;
    if (hand_type == KDL_HAND_TYPE_ROBOTIQ) {
	hand_vector = Vector(0.0, 0.225, 0.0);
    }
    else if (hand_type == KDL_HAND_TYPE_DRC_TRIALS) {
	hand_vector = Vector(0.0, 0.2, 0.0);
    }
    else if (hand_type == KDL_HAND_TYPE_SPIKE) {
	hand_vector = Vector(0.0, 0.30, 0.08);
    }
    else if (hand_type == KDL_HAND_TYPE_EXT_20CM) {
	hand_vector = Vector(0.0, 0.31, -0.28);
    }
    else if (hand_type == KDL_HAND_TYPE_SIDE_PIPE5) {
	hand_vector = Vector(0.0, 0.24, 0.15);
	//hand_vector = Vector(0.15, 0.24, 0.0);
    }
    else {
	std::cout << __PRETTY_FUNCTION__ << " Unknown hand_type " << hand_type << std::endl;
    }
}




// The camera rotation matrix has some extras in there to swap around x/y/z
// We don't want that, so use the head rotation matrix with the camera x/y/z
void KDLwrapper::leftCameraFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame)
{
    KDL::Chain leftCamChain;
    KDL::Chain headChain;
    my_tree.getChain("pelvis", "left_camera_optical_frame", leftCamChain);
    my_tree.getChain("pelvis", "head", headChain);

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(leftCamChain);//Forward position solver
    ChainFkSolverPos_recursive fksolverhead(headChain);//Forward position solver
 
    //Creation of jntarrays:
    JntArray q_init(leftCamChain.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[0]; 
    q_init(1) = initState.position[1]; 
    q_init(2) = initState.position[2]; 
    q_init(3) = initState.position[3];

    // Starting Frame
    fksolver1.JntToCart(q_init, *dest_frame);
    double x,y,z;
    x = dest_frame->p.x();
    y = dest_frame->p.y();
    z = dest_frame->p.z();
    fksolverhead.JntToCart(q_init, *dest_frame);
    dest_frame->p.x(x);
    dest_frame->p.y(y);
    dest_frame->p.z(z);
}


void KDLwrapper::leftLegFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame)
{
    KDL::Chain rightArmChain2;
    my_tree.getChain("pelvis", "l_foot", rightArmChain2);
    Frame myframe(g_footvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain2.addSegment(myadd);


    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain2);//Forward position solver
 
    //Creation of jntarrays:
    JntArray q_init(rightArmChain2.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[4]; 
    q_init(1) = initState.position[5]; 
    q_init(2) = initState.position[6]; 
    q_init(3) = initState.position[7]; 
    q_init(4) = initState.position[8]; 
    q_init(5) = initState.position[9]; 


    // Starting Frame
    fksolver1.JntToCart(q_init, *dest_frame);
}


void KDLwrapper::rightLegFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame)
{
    KDL::Chain rightArmChain2;
    my_tree.getChain("pelvis", "r_foot", rightArmChain2);
    Frame myframe(g_footvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain2.addSegment(myadd);

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain2);//Forward position solver
 
    //Creation of jntarrays:
    JntArray q_init(rightArmChain2.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[10]; 
    q_init(1) = initState.position[11]; 
    q_init(2) = initState.position[12]; 
    q_init(3) = initState.position[13]; 
    q_init(4) = initState.position[14]; 
    q_init(5) = initState.position[15]; 

    // Starting Frame
    fksolver1.JntToCart(q_init, *dest_frame);
}

void KDLwrapper::rightArmFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame, KDL_HAND_TYPE hand_type)
{
    KDL::Chain rightArmChain2;
    //std::cout << my_tree.getChain("pelvis", "r_hand", rightArmChain2) << std::endl;
    my_tree.getChain("pelvis", "r_hand", rightArmChain2);
    //std::cout << my_tree.getChain("pelvis", "r_palm", rightArmChain2) << std::endl;
    //std::cout << my_tree.getNrOfJoints() << " " << my_tree.getNrOfSegments() << std::endl;

    
    Vector myvector(0.0, -0.2, 0.0);
    Frame myframe(myvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain2.addSegment(myadd);
    
    /*
    //std::cout << rightArmChain2.getNrOfJoints() << " " << rightArmChain2.getNrOfSegments() << std::endl;

    Segment sa;
    Frame f;
    Joint j;
    Frame f2;
    Frame f3;
    Frame f4;
    for (unsigned int i = 0; i < rightArmChain2.getNrOfSegments(); i++) {
      sa =  rightArmChain2.getSegment(i);
      f = sa.getFrameToTip();
      j = sa.getJoint();
      f2 = sa.pose(0);
      f3 = sa.pose(0.2);
      f4 = j.pose(0.2);
      //std::cout << f << std::endl;
      //std::cout << f2 << std::endl;
      //std::cout << f3 << std::endl;
      //std::cout << f4 << std::endl;
      //std::cout << j.JointAxis() << " "<< j.JointOrigin() << " "<< j.getType()  << std::endl;
    }

    
    Twist t;
    SegmentMap s2 = rightArmChain2.getSegments();
    SegmentMap::const_iterator itr;
    //std::cout << "first loop " << s2.size() <<std::endl;
    for (itr = s2.begin(); itr != s2.end(); ++itr) {
      std::cout << itr->second.segment.getJoint().getName() << "," << itr->second.q_nr   << ",";
      std::cout << "origin, " << itr->second.segment.getJoint().JointOrigin();
      
      }
    */


    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain2);//Forward position solver
 
    //Creation of jntarrays:
    JntArray q_init(rightArmChain2.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[0]; 
    q_init(1) = initState.position[1]; 
    q_init(2) = initState.position[2]; 
    q_init(3) = initState.position[22]; 
    q_init(4) = initState.position[23]; 
    q_init(5) = initState.position[24]; 
    q_init(6) = initState.position[25]; 
    q_init(7) = initState.position[26]; 
    q_init(8) = initState.position[27]; 

    // Starting Frame
    fksolver1.JntToCart(q_init, *dest_frame);
}

void KDLwrapper::leftArmFK(atlas_msgs::AtlasState initState, KDL::Frame *dest_frame, KDL_HAND_TYPE hand_type)
{
    KDL::Chain rightArmChain2;
    my_tree.getChain("pelvis", "l_hand", rightArmChain2);
    
    Vector myvector;
    handTypeAdjustment(myvector, hand_type);
    Frame myframe(myvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain2.addSegment(myadd);
 
    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain2);//Forward position solver
 
    //Creation of jntarrays:
    JntArray q_init(rightArmChain2.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[0]; 
    q_init(1) = initState.position[1]; 
    q_init(2) = initState.position[2]; 
    q_init(3) = initState.position[16]; 
    q_init(4) = initState.position[17]; 
    q_init(5) = initState.position[18]; 
    q_init(6) = initState.position[19]; 
    q_init(7) = initState.position[20]; 
    q_init(8) = initState.position[21]; 

    // Starting Frame
    fksolver1.JntToCart(q_init, *dest_frame);
}













double KDLwrapper::rightLegIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out)
{
    KDL::Chain rightArmChain;
    my_tree.getChain("pelvis", "r_foot", rightArmChain);

    // This added vector is needed as the urdf is too short (doesn't include foot)
    // AND - the final joint includes a zero length tip (possible KDL bug), this adds a real tip
    Frame myframe(g_footvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain.addSegment(myadd);

    //Creation of jntarrays:
    JntArray q(rightArmChain.getNrOfJoints());
    JntArray q_min(rightArmChain.getNrOfJoints());
    JntArray q_max(rightArmChain.getNrOfJoints());
    JntArray q_init(rightArmChain.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[10]; 
    q_init(1) = initState.position[11]; 
    q_init(2) = initState.position[12]; 
    q_init(3) = initState.position[13]; 
    q_init(4) = initState.position[14]; 
    q_init(5) = initState.position[15]; 
    q_max(0) = 0.18;
    q_min(0) = -1.21;
    q_max(1) = 0.35;
    q_min(1) = -0.46;
    q_max(2) =  0.52;
    q_min(2) = -1.75;
    q_max(3) = 2.39;
    q_min(3) = 0.0;
    q_max(4) = 0.72;
    q_min(4) = -1.0;
    q_max(5) = 0.61;
    q_min(5) = -0.61;

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(rightArmChain);//Inverse velocity solver
    ChainIkSolverPos_NR_JL2 iksolver2(rightArmChain,q_min, q_max, fksolver1,iksolver1v);
    double ret = iksolver2.CartToJnt(q_init,dest_frame,q);

    // Copy output back to joint positions
    joints_out->position[10] = q(0);
    joints_out->position[11] = q(1);
    joints_out->position[12] = q(2);
    joints_out->position[13] = q(3);
    joints_out->position[14] = q(4);
    joints_out->position[15] = q(5);

    return ret;
}

double KDLwrapper::leftLegIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out)
{
    KDL::Chain rightArmChain;
    my_tree.getChain("pelvis", "l_foot", rightArmChain);

    // This added vector is needed as the urdf is too short (doesn't include foot)
    // AND - the final joint includes a zero length tip (possible KDL bug), this adds a real tip
    Frame myframe(g_footvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain.addSegment(myadd);

    //Creation of jntarrays:
    JntArray q(rightArmChain.getNrOfJoints());
    JntArray q_min(rightArmChain.getNrOfJoints());
    JntArray q_max(rightArmChain.getNrOfJoints());
    JntArray q_init(rightArmChain.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init(0) = initState.position[4]; 
    q_init(1) = initState.position[5]; 
    q_init(2) = initState.position[6]; 
    q_init(3) = initState.position[7]; 
    q_init(4) = initState.position[8]; 
    q_init(5) = initState.position[9]; 
    q_max(0) = 1.22;
    q_min(0) = -0.17;
    q_max(1) = 0.53;
    q_min(1) = -0.34;
    q_max(2) = 0.54;
    q_min(2) = -1.75;
    q_max(3) = 2.35;
    q_min(3) = 0;
    q_max(4) = 0.72;
    q_min(4) = -1.0;
    q_max(5) = 0.61;
    q_min(5) = -0.61;

    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(rightArmChain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(rightArmChain);//Inverse velocity solver
    ChainIkSolverPos_NR_JL2 iksolver2(rightArmChain,q_min, q_max, fksolver1,iksolver1v);


    double ret = iksolver2.CartToJnt(q_init,dest_frame,q);

    // Copy output back to joint positions
    joints_out->position[4] = q(0);
    joints_out->position[5] = q(1);
    joints_out->position[6] = q(2);
    joints_out->position[7] = q(3);
    joints_out->position[8] = q(4);
    joints_out->position[9] = q(5);
    return ret;
}

// Performs inverse kinematics for the right arm
// Input - initial joint state (initState), destination position/orientation relative to pelvis (dest_frame)
// Output - updated six right arm joints in joints_out
// Return value is how large error is relative to the desired dest_frame
// Does best effort rather than failing
double KDLwrapper::rightArmIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type, int mode)
{
    Chain bodyChain;
    Chain rightArmChain;
    my_tree.getChain("pelvis", "utorso", bodyChain);
    my_tree.getChain("utorso", "r_hand", rightArmChain);
    //std::cout << my_tree.getNrOfJoints() << " " << my_tree.getNrOfSegments() << std::endl;

    // This added vector is needed as the urdf is too short (doesn't include hand)
    // AND - the final joint includes a zero length tip (possible KDL bug), this adds a real tip
    Vector myvector(0.0, -0.2, 0.0);
    Frame myframe(myvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    rightArmChain.addSegment(myadd);

    //std::cout << rightArmChain.getNrOfJoints() << " " << rightArmChain.getNrOfSegments() << std::endl;


    //Creation of the solvers:
    ChainFkSolverPos_recursive bodysolver(bodyChain);//Forward position solver
    ChainFkSolverPos_recursive fksolver1(rightArmChain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(rightArmChain);//Inverse velocity solver
    //ChainIkSolverPos_NR iksolver1(rightArmChain,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
 
    //Creation of jntarrays:
    JntArray q(rightArmChain.getNrOfJoints());
    JntArray q_min(rightArmChain.getNrOfJoints());
    JntArray q_max(rightArmChain.getNrOfJoints());
    JntArray q_init(rightArmChain.getNrOfJoints());
    JntArray q_init_body(bodyChain.getNrOfJoints());
  
    // Fill in initial array
    // Hand created map
    q_init_body(0) = initState.position[0]; 
    q_init_body(1) = initState.position[1]; 
    q_init_body(2) = initState.position[2]; 

    q_init(0) = initState.position[22]; 
    q_init(1) = initState.position[23]; 
    q_init(2) = initState.position[24]; 
    q_init(3) = initState.position[25]; 
    q_init(4) = initState.position[26]; 
    q_init(5) = initState.position[27]; 
    if (hand_type == KDL_HAND_TYPE_ROBOTIQ) {
	q_max(0) = R_USY_POS_MAX;
	q_min(0) = R_USY_POS_MIN;
	q_max(1) = R_SHX_POS_MAX;
	q_min(1) = R_SHX_POS_MIN;
	q_max(2) = R_ELY_POS_MAX;
	q_min(2) = R_ELY_POS_MIN;
	q_max(3) = R_ELX_POS_MAX;
	q_min(3) = R_ELX_POS_MIN;
	q_max(4) = R_UWY_POS_MAX;
	q_min(4) = R_UWY_POS_MIN;
	q_max(5) = R_MWX_POS_MAX;
	q_min(5) = R_MWX_POS_MIN;
    }
    else {
	q_max(0) = std::min(R_USY_POS_MAX, initState.position[22] + 0.05);
	q_min(0) = std::max(R_USY_POS_MIN, initState.position[22] - 0.05);
	q_max(1) = std::min(R_SHX_POS_MAX, initState.position[23] + 0.05);
	q_min(1) = std::max(R_SHX_POS_MIN, initState.position[23] - 0.05);
	q_max(2) = std::min(R_ELY_POS_MAX, initState.position[24] + 0.1);
	q_min(2) = std::max(R_ELY_POS_MIN, initState.position[24] - 0.1);
	q_max(3) = std::min(R_ELX_POS_MAX, initState.position[25] + 0.1);
	q_min(3) = std::max(R_ELX_POS_MIN, initState.position[25] - 0.1);
	q_max(4) = std::min(R_UWY_POS_MAX, initState.position[26] + 0.2);
	q_min(4) = std::max(R_UWY_POS_MIN, initState.position[26] - 0.2);
	q_max(5) = std::min(R_MWX_POS_MAX, initState.position[27] + 0.1);
	q_min(5) = std::max(R_MWX_POS_MIN, initState.position[27] - 0.1);
    }

    ChainIkSolverPos_NR_JL2 iksolver2(rightArmChain,q_min, q_max, fksolver1,iksolver1v);



    // Starting Frame
    Frame F_start_body;
    bodysolver.JntToCart(q_init_body, F_start_body);
    Frame F_dest_real = F_start_body.Inverse() * dest_frame;
    //std::cout << "dest_frame " << std::endl << dest_frame << std::endl;
    //std::cout << "F_dest_real " << std::endl << F_dest_real << std::endl;
    //std::cout << "F_start_body " << std::endl << F_start_body << std::endl;


    Frame F_start;
    fksolver1.JntToCart(q_init, F_start);
    //std::cout << "Start Frame " << std::endl << F_start << std::endl;
  
    //int ret = iksolver1.CartToJnt(q_init,F_dest_real,q);
    double ret = iksolver2.CartToJnt(q_init,F_dest_real,q, mode);

    //Frame F_actual;
    //fksolver1.JntToCart(q, F_actual);
    //std::cout << "Result Frame " << std::endl << F_actual << std::endl;

    //std::cout << ret << " " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << std::endl;

    // Copy output back to joint positions
    joints_out->position[22] = q(0);
    joints_out->position[23] = q(1);
    joints_out->position[24] = q(2);
    joints_out->position[25] = q(3);
    joints_out->position[26] = q(4);
    joints_out->position[27] = q(5);
    return ret;
}


double KDLwrapper::leftArmIK(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type, int mode)
{
    KDL::Chain bodyChain;
    KDL::Chain leftArmChain;
    my_tree.getChain("pelvis", "utorso", bodyChain);
    my_tree.getChain("utorso", "l_hand", leftArmChain);

    Vector myvector;
    handTypeAdjustment(myvector, hand_type);
    Frame myframe(myvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    leftArmChain.addSegment(myadd);

    //Creation of jntarrays:
    JntArray q(leftArmChain.getNrOfJoints());
    JntArray q_min(leftArmChain.getNrOfJoints());
    JntArray q_max(leftArmChain.getNrOfJoints());
    JntArray q_init(leftArmChain.getNrOfJoints());
    JntArray q_init_body(bodyChain.getNrOfJoints());

    // Fill in initial array
    // Hand created map
    q_init_body(0) = initState.position[0]; 
    q_init_body(1) = initState.position[1]; 
    q_init_body(2) = initState.position[2]; 


    // fixme - this will need to change for v5
    q_init(0) = initState.position[16];
    q_init(1) = initState.position[17];
    q_init(2) = initState.position[18];
    q_init(3) = initState.position[19];
    q_init(4) = initState.position[20];
    q_init(5) = initState.position[21];
    if (hand_type == KDL_HAND_TYPE_ROBOTIQ) {
	q_max(0) = L_USY_POS_MAX;
	q_min(0) = L_USY_POS_MIN;
	q_max(1) = L_SHX_POS_MAX;
	q_min(1) = L_SHX_POS_MIN;
	q_max(2) = L_ELY_POS_MAX;
	q_min(2) = L_ELY_POS_MIN;
	q_max(3) = L_ELX_POS_MAX;
	q_min(3) = L_ELX_POS_MIN;
	q_max(4) = L_UWY_POS_MAX;
	q_min(4) = L_UWY_POS_MIN;
	q_max(5) = L_MWX_POS_MAX;
	q_min(5) = L_MWX_POS_MIN;
    }
    else {
	q_max(0) = std::min(L_USY_POS_MAX, initState.position[16] + 0.05);
	q_min(0) = std::max(L_USY_POS_MIN, initState.position[16] - 0.05);
	q_max(1) = std::min(L_SHX_POS_MAX, initState.position[17] + 0.05);
	q_min(1) = std::max(L_SHX_POS_MIN, initState.position[17] - 0.05);
	q_max(2) = std::min(L_ELY_POS_MAX, initState.position[18] + 0.1);
	q_min(2) = std::max(L_ELY_POS_MIN, initState.position[18] - 0.1);
	q_max(3) = std::min(L_ELX_POS_MAX, initState.position[19] + 0.1);
	q_min(3) = std::max(L_ELX_POS_MIN, initState.position[19] - 0.1);
	q_max(4) = std::min(L_UWY_POS_MAX, initState.position[20] + 0.2);
	q_min(4) = std::max(L_UWY_POS_MIN, initState.position[20] - 0.2);
	q_max(5) = std::min(L_MWX_POS_MAX, initState.position[21] + 0.1);
	q_min(5) = std::max(L_MWX_POS_MIN, initState.position[21] - 0.1);
    }


    //Creation of the solvers:
    ChainFkSolverPos_recursive bodysolver(bodyChain);//Forward position solver
    ChainFkSolverPos_recursive fksolver1(leftArmChain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(leftArmChain);//Inverse velocity solver
    ChainIkSolverPos_NR_JL2 iksolver2(leftArmChain,q_min, q_max, fksolver1,iksolver1v);



    // Remove body offset, so destination is relative to upper torso
    Frame F_start_body;
    bodysolver.JntToCart(q_init_body, F_start_body, mode);
    Frame F_dest_real = F_start_body.Inverse() * dest_frame;

    //std::cout << "debug " << F_start_body << std::endl << F_dest_real << std::endl;

    double ret = iksolver2.CartToJnt(q_init,F_dest_real,q, mode);

    // Copy output back to joint positions
    joints_out->position[16] = q(0);
    joints_out->position[17] = q(1);
    joints_out->position[18] = q(2);
    joints_out->position[19] = q(3);
    joints_out->position[20] = q(4);
    joints_out->position[21] = q(5);
    return ret;
}

double KDLwrapper::leftWristIK_steering(atlas_msgs::AtlasState initState, KDL::Frame dest_frame, atlas_msgs::AtlasCommand *joints_out, KDL_HAND_TYPE hand_type, int mode)
{
    KDL::Chain pelvisToWristChain;
    KDL::Chain leftWristChain;
    my_tree.getChain("pelvis", "l_larm", pelvisToWristChain);
    my_tree.getChain("l_larm", "l_hand", leftWristChain);


    Vector myvector;
    handTypeAdjustment(myvector, hand_type);
    Frame myframe(myvector);
    Segment myadd("myadd", Joint(Joint::None), myframe);
    leftWristChain.addSegment(myadd);

    //Creation of jntarrays:
    JntArray q(leftWristChain.getNrOfJoints());
    JntArray q_min(leftWristChain.getNrOfJoints());
    JntArray q_max(leftWristChain.getNrOfJoints());
    JntArray q_init(leftWristChain.getNrOfJoints());
    JntArray q_init_body(pelvisToWristChain.getNrOfJoints());

    // Fill in initial array
    // Hand created map
    q_init_body(0) = initState.position[0]; 
    q_init_body(1) = initState.position[1]; 
    q_init_body(2) = initState.position[2]; 
    q_init_body(3) = initState.position[16]; 
    q_init_body(4) = initState.position[17]; 
    q_init_body(5) = initState.position[18]; 
    q_init_body(6) = initState.position[19]; 


    // fixme - this will need to change for v5
    q_init(0) = initState.position[20];
    q_init(1) = initState.position[21];
    q_max(0) = std::min(L_UWY_POS_MAX, initState.position[20] + 0.2);
    q_min(0) = std::max(L_UWY_POS_MIN, initState.position[20] - 0.2);
    q_max(1) = std::min(L_MWX_POS_MAX, initState.position[21] + 0.1);
    q_min(1) = std::max(L_MWX_POS_MIN, initState.position[21] - 0.1);

    //Creation of the solvers:
    ChainFkSolverPos_recursive bodysolver(pelvisToWristChain);//Forward position solver
    ChainFkSolverPos_recursive fksolver1(leftWristChain);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(leftWristChain);//Inverse velocity solver
    ChainIkSolverPos_NR_JL2 iksolver2(leftWristChain,q_min, q_max, fksolver1, iksolver1v);


    // Remove body offset, so destination is relative to upper torso
    Frame F_start_body;
    bodysolver.JntToCart(q_init_body, F_start_body);
    Frame F_dest_real = F_start_body.Inverse() * dest_frame;

    //std::cerr << "debug " << F_start_body << std::endl << F_dest_real << std::endl;

    double ret = iksolver2.CartToJntWrist(q_init,F_dest_real,q, mode);

    // Copy output back to joint positions
    joints_out->position[20] = q(0);
    joints_out->position[21] = q(1);
    //fprintf(stderr, "wr9 %lf %lf %lf %lf\n", q(0), q(1), initState.position[20], initState.position[21]);

    return ret;
}








static void calc_link(std::string whichlink, KDL::Tree my_tree, KDL::TreeFkSolverPos_recursive *tfksolver, KDL::JntArray *tjointpositions, double *sumx, double *sumy, double *sumz, double *totalmass)
{
    //std::cout << whichlink << std::endl;
    // Create the frame that will contain the results
    Frame tcartpos;
    // Calculate forward position kinematics
    bool tkinematics_status;
    tkinematics_status = tfksolver->JntToCart(*tjointpositions,tcartpos, whichlink);
    if(tkinematics_status>=0){
	//std::cout << whichlink << " " << tcartpos.p <<std::endl;
	//printf("%s \n","Tree Success, thanks KDL!");
    }
    else{
	printf("%s \n","Error: could not calculate tree forward kinematics :(");
    }


    std::map<std::string,KDL::TreeElement>::const_iterator s1 = my_tree.getSegment(whichlink);
    RigidBodyInertia r1;
    r1 = s1->second.segment.getInertia();

    *totalmass = *totalmass + r1.getMass();


    Vector v1 = r1.getCOG();
    Vector v2 = tcartpos * v1;

    *sumx = *sumx + v2.x() * r1.getMass();
    *sumy = *sumy + v2.y() * r1.getMass();
    *sumz = *sumz + v2.z() * r1.getMass();
    //std::cout << whichlink << " " << r1.getMass() << " " << v2.x() << " " << v2.y()  << " " << v2.z();
    //std::cout << " " << tcartpos(0,3) << " " << tcartpos(1,3) << " " << tcartpos(2,3) << std::endl;

    //Vector v6 = tcartpos * s1->second.segment.getJoint().JointOrigin();
    //std::cout << ",COG," << std::endl << v1 << ",translated COG," << std::endl << v2 << std::endl;
    //std::cout << " translated joint origin, " << std::endl << v6 << ",cartarray," << std::endl << tcartpos <<std::endl;


  
}













// For each link
//      find the mass
//      add mass to total
//      run kinematics
//      use matrix to determine global coord of link mass center
//      add link mass * x, to total
//      print out link info
// divide the sumx, sumy, sumz by total mass - this is the center of mass
// print this out along with foot positions
// test - move center of mass outside of feet in each direction 

void KDLwrapper::calculateCenterOfMass(atlas_msgs::AtlasState initState, KDL::Vector *com)
{
  double totalmass = 0.0;
  double sumx = 0.0;
  double sumy = 0.0;
  double sumz = 0.0;
  // Find for all of the joints...

  // Create solver based on kinematic chain
  TreeFkSolverPos_recursive tfksolver = TreeFkSolverPos_recursive(my_tree);
 
  // Create joint array
  unsigned int tnj = my_tree.getNrOfJoints();
  JntArray tjointpositions = JntArray(tnj);

  // Assign some values to the joint positions
  // This is a hand-made map, not a 1:1 - may change - fixme
  //for(unsigned int i=0;i<tnj;i++){
    //float myinput;
    //printf ("Enter the position of joint %i: ",i);
    //scanf ("%e",&myinput);
    //tjointpositions(i)=goal1.trajectory.points[0].positions[i];
  //}


  // For use with atlas hands
  tjointpositions(0)=initState.position[0]; // lbz
  tjointpositions(1)=initState.position[1]; // mby
  tjointpositions(2)=initState.position[2]; // ubx
  tjointpositions(3)=initState.position[16]; // l usy
  tjointpositions(4)=initState.position[17]; // l shx
  tjointpositions(5)=initState.position[18]; // l ely
  tjointpositions(6)=initState.position[19]; // l elx
  tjointpositions(7)=initState.position[20]; // l uwy
  tjointpositions(8)=initState.position[21]; // l mwx
  // joints 9-20 are left finger joints
  tjointpositions(21)=initState.position[3];  // neck
  tjointpositions(22)=0.0; // hokuyo_joint
  tjointpositions(23)=initState.position[22]; // right usy joint
  tjointpositions(24)=initState.position[23];
  tjointpositions(25)=initState.position[24];
  tjointpositions(26)=initState.position[25];
  tjointpositions(27)=initState.position[26];
  tjointpositions(28)=initState.position[27];
  // joints 29-40 are right finger joints
  tjointpositions(41)=initState.position[4];  // uhz
  tjointpositions(42)=initState.position[5];
  tjointpositions(43)=initState.position[6];
  tjointpositions(44)=initState.position[7];
  tjointpositions(45)=initState.position[8];
  tjointpositions(46)=initState.position[9];
  tjointpositions(47)=initState.position[10];
  tjointpositions(48)=initState.position[11];
  tjointpositions(49)=initState.position[12];
  tjointpositions(50)=initState.position[13];
  tjointpositions(51)=initState.position[14];
  tjointpositions(52)=initState.position[15];

  /*
  // For use without atlas hands
  tjointpositions(0)=initState.position[0]; // lbz
  tjointpositions(1)=initState.position[1]; // mby
  tjointpositions(2)=initState.position[2]; // ubx
  tjointpositions(3)=initState.position[16]; // l usy
  tjointpositions(4)=initState.position[17]; // l shx
  tjointpositions(5)=initState.position[18]; // l ely
  tjointpositions(6)=initState.position[19]; // l elx
  tjointpositions(7)=initState.position[20]; // l uwy
  tjointpositions(8)=initState.position[21]; // l mwx
  tjointpositions(9)=initState.position[3];  // neck
  tjointpositions(10)=0.0; // hokuyo_joint
  tjointpositions(11)=initState.position[22]; // reversal of right usy joint - not needed as 11APR2013
  tjointpositions(12)=initState.position[23];
  tjointpositions(13)=initState.position[24];
  tjointpositions(14)=initState.position[25];
  tjointpositions(15)=initState.position[26];
  tjointpositions(16)=initState.position[27];
  tjointpositions(17)=initState.position[4];  // uhz
  tjointpositions(18)=initState.position[5];
  tjointpositions(19)=initState.position[6];
  tjointpositions(20)=initState.position[7];
  tjointpositions(21)=initState.position[8];
  tjointpositions(22)=initState.position[9];
  tjointpositions(23)=initState.position[10];
  tjointpositions(24)=initState.position[11];
  tjointpositions(25)=initState.position[12];
  tjointpositions(26)=initState.position[13];
  tjointpositions(27)=initState.position[14];
  tjointpositions(28)=initState.position[15];
  */


  //std::cout << "joints " << my_tree.getNrOfJoints() << std::endl;
  //std::cout << "segments " << my_tree.getNrOfSegments() << std::endl;


  SegmentMap s2 = my_tree.getSegments();
  SegmentMap::const_iterator itr;

  //std::cout << "first loop " << s2.size() <<std::endl;
  for (itr = s2.begin(); itr != s2.end(); ++itr) {
      //std::cout << itr->second.segment.getJoint().getName() << "," << itr->second.q_nr   << "," << std::endl;
      //std::cout << "origin, " << itr->second.segment.getJoint().JointOrigin();

	  calc_link(itr->first, my_tree, &tfksolver, &tjointpositions, &sumx, &sumy, &sumz, &totalmass);

  }


  //std::cout << "Total mass " << totalmass << std::endl;

  // final calculations
  sumx = sumx / totalmass;
  sumy = sumy / totalmass;
  sumz = sumz / totalmass;


  Frame tcartposleft;
  // Calculate forward position kinematics
  bool tkinematics_status;
  tkinematics_status = tfksolver.JntToCart(tjointpositions,tcartposleft, "l_foot");
  if(tkinematics_status>=0){
	  //std::cout << whichlink << " " << tcartpos <<std::endl;
	  //printf("%s \n","Tree Success, thanks KDL!");
  }else{
	  printf("%s \n","Error: could not calculate tree forward kinematics :(");
  }

  Frame tcartposright;
  tkinematics_status = tfksolver.JntToCart(tjointpositions,tcartposright, "r_foot");
  if(tkinematics_status>=0){
	  //std::cout << whichlink << " " << tcartpos <<std::endl;
	  //printf("%s \n","Tree Success, thanks KDL!");
  }else{
	  printf("%s \n","Error: could not calculate tree forward kinematics :(");
  }

  Frame tcartposhead;
  tkinematics_status = tfksolver.JntToCart(tjointpositions,tcartposhead, "head");
  if(tkinematics_status>=0){
	  //std::cout << whichlink << " " << tcartpos <<std::endl;
	  //printf("%s \n","Tree Success, thanks KDL!");
  }else{
	  printf("%s \n","Error: could not calculate tree forward kinematics :(");
  }




  fprintf(stderr, "x %.3lf y %.3lf z %.3lf m %.2lf l %.3lf %.3lf %.3lf r %.3lf %.3lf %.3lf\n",
  		  sumx, sumy, sumz, totalmass, tcartposleft.p.x(), tcartposleft.p.y(),tcartposleft.p.z(),
  		  tcartposright.p.x(),tcartposright.p.y(),tcartposright.p.z());
  printf("Lx %.2lf Ly %.2lf Rx %.2lf Ry %.2lf Cx %.2lf Cy %.2lf", tcartposleft.p.x(), tcartposleft.p.y(),
	 tcartposright.p.x(), tcartposright.p.y(), sumx, sumy);
  printf("Head %.2lf %.2lf %.2lf\n", tcartposhead.p.x(),  tcartposhead.p.y(),  tcartposhead.p.z());
  com->x(sumx);
  com->y(sumy);
  com->z(sumz);
}



