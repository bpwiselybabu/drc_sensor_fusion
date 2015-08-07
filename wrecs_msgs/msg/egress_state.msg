Header header

geometry_msgs/Pose  root
geometry_msgs/Twist rootd
geometry_msgs/Pose  utorso
geometry_msgs/Pose left_foot
geometry_msgs/Pose right_foot
geometry_msgs/Pose left_hand
geometry_msgs/Pose right_hand

geometry_msgs/Pose root_d
geometry_msgs/Pose utorso_d
geometry_msgs/Pose left_foot_d
geometry_msgs/Pose right_foot_d
geometry_msgs/Pose left_hand_d
geometry_msgs/Pose right_hand_d
geometry_msgs/Point left_elbow_d
geometry_msgs/Point right_elbow_d
geometry_msgs/Point left_knee_d
geometry_msgs/Point right_knee_d

geometry_msgs/Point COM
geometry_msgs/Point COM_d

float32[6] left_foot_wrench
float32[6] right_foot_wrench

float32[6] left_hand_wrench
float32[6] right_hand_wrench

float32[2] l_foot_b_cop
float32[2] r_foot_b_cop
float32[2] w_cop
float32[2] l_foot_w_cop
float32[2] r_foot_w_cop
	   
float32[35] rconfig_d
float32[35] rconfig

int32 contactState
int32 handContact

string info


int32 DSc = 0
int32 SSL = 1
int32 SSR = 2
int32 PELVIS_FIX = 6
int32 NO_HANDS = 0
int32 LEFT_HAND = 1
int32 RIGHT_HAND = 2
int32 BOTH_HANDS = 3
