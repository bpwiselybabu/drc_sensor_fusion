# Message for representing Door Task FSM States

# Controller/GUI
# sender_id = 0 (Controller)
# sender_id = 1 (GUI)
int8 sender_id

# FSM State
# Only if sender_id = 0
int8 fsm_state 
# Only if sender_id = 0 or 1
int8 fsm_event
# Only if sender_id = 0
int8 fsm_action

# Ask for validation
# 0 = Disable, 1 = Enable (Only if sender_id = 0)
int8 validate_enable
# 0 = No, 1 = Yes (Only if sender_id = 1)
int8 validation_response

# FSM Status
# Only if sender_id = 0
string fsm_status

# Walking
# Plane Ax+By+Cz+D = 0
std_msgs/Header plane_header
shape_msgs/Plane plane
bool walk

# Door point
geometry_msgs/Pose door_pt

# Door type (1 for LEFT, 2 for RIGHT, 0 for UNKNOWN)
int8 door_type