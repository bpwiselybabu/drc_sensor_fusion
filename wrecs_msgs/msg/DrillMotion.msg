# Message constants
int8 CUTTING_START=1
int8 CUTTING_STOP=2
int8 CUTTING_PAUSE=3
int8 CUTTING_RESUME=4
int8 CUTTING_CHNGSTATE=5
int8 CUTTING_USER=6

int8 CSTATE_STANDBY=11
int8 CSTATE_AUTO=12
int8 CSTATE_MANUAL=13
int8 CSTATE_PAUSED=14
int8 CSTATE_NONE=15

int8 CSTATE_AUTO_SETPREP=21
int8 CSTATE_AUTO_SETSTART=22
int8 CSTATE_AUTO_PUSHIN=23
int8 CSTATE_AUTO_NUDGE=24
int8 CSTATE_AUTO_PULLOUT=25
int8 CSTATE_AUTO_SETINIT=26
int8 CSTATE_AUTO_FTCUT=27
int8 CSTATE_AUTO_WAITCONFIRM=28
int8 CSTATE_AUTO_PUNCH=29

int8 USER_CMD_PUSHIN=31
int8 USER_CMD_PULLOUT=32
int8 USER_CMD_TIMEBACK=33
int8 USER_CMD_TIMEFORWARD=34
int8 USER_CMD_CONFIRMCUT=35
int8 USER_CMD_PUNCH=36
int8 USER_CMD_SKIPPUNCH=37

# Cutting control
int8 cutting_cmd
int8 user_cmd
bool use_left_hand
bool use_ft_servo

# Cutting state machine
int8 cutting_state
int8 auto_fsm_state

#Message for representing Atlas Behavior States

#The behavior
geometry_msgs/PointStamped[] drill_points

#Plane Ax+By+Cz+D = 0
std_msgs/Header plane_header
shape_msgs/Plane plane
bool walk
bool knock
bool startsignal
bool pausesignal
float32 plungeDistance
float32 repeatTimes


# Extra feedback information from robot
