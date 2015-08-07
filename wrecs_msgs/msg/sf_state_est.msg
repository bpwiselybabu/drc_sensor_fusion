Header header

int64 dfr_seq_id
int64 dfr_timestamp
int64 dfr_sensor_head_pps_timestamp

# sdfast pelvis 
geometry_msgs/Pose sdf_pelv
geometry_msgs/Vector3 sdf_pelvd
geometry_msgs/Vector3 pelvw

# pelvis angular velocity in body frame
geometry_msgs/Vector3 pelv_b_w

# utorso orientation
geometry_msgs/Quaternion utorsoq

float64[28] joints
float64[28] jointsd

# hand offsets
geometry_msgs/Vector3[2] wrist2hand

# sdfast model com 
float64[3] model_com
# model com + estimated com offset, cop, capture point
float64[3] com_est
float64[2] cop_est
float64[2] cp_est

# support polygon point, first one is number of points in support polygon
float64[8] support_polygon_x
float64[8] support_polygon_y

# force
geometry_msgs/Vector3[2] foot_w_F
geometry_msgs/Vector3[2] foot_w_Tau
geometry_msgs/Vector3[2] hand_w_F
geometry_msgs/Vector3[2] hand_w_Tau

# neck
geometry_msgs/Pose neck
geometry_msgs/Vector3 neckd
geometry_msgs/Vector3 neckw

# middle of the bottom of the foot
geometry_msgs/Pose[2] sdf_mid_foot
geometry_msgs/Vector3[2] sdf_mid_footd
geometry_msgs/Vector3[2] mid_footw

geometry_msgs/Pose[2] hand
geometry_msgs/Vector3[2] handd
geometry_msgs/Vector3[2] handw

# urdf pelv location
geometry_msgs/Vector3 urdf_pelv
geometry_msgs/Vector3 urdf_pelvd
# urdf foot location (at the ankles)
geometry_msgs/Vector3[2] urdf_foot
geometry_msgs/Vector3[2] urdf_footd

int32 contact_state

int32 DSc = 0
int32 SSL = 1
int32 SSR = 2
int32 AIR = 3
