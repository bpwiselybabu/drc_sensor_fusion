#Message for representing Atlas Behavior States and the position of the valve

#The opsition of Valve

geometry_msgs/PoseStamped ValvePose

float32 Radius
float32 SpeedFactor
int8 Turns
float32 PrepDistance
float32 Duration
float32 crouchHeight
# type 0 = gate valve, type 1 = ball
int8 valvetype

bool LeftHand
bool RightHand
bool BothHands
bool walkflag
bool crouchFlag
bool retrieveFlag
bool isUser
bool m_hand_left
bool m_side_left
bool isRobot
bool closeLimitZone
bool side_button_clicked
bool hand_button_clicked

std_msgs/Header plane_header
shape_msgs/Plane plane