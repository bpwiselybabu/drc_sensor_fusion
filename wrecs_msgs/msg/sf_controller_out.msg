Header header

bool isIdle
bool isPause
int32 smIndex

geometry_msgs/Vector3[2] wrist2hand

# ik res
float64[28] ik_joints
geometry_msgs/Pose ik_sdf_pelv
geometry_msgs/Vector3 ik_com


# ikcmd 
geometry_msgs/Vector3 ikcmd_com
geometry_msgs/Pose[2] ikcmd_hands
geometry_msgs/Pose[2] ikcmd_feet
geometry_msgs/Pose ikcmd_sdf_pelv

int32[2] ikcmd_hand_mode

# id  
#float64[4] id_foot_b_cop
#float64[12] id_b_grf
