#Message for representing Atlas Behavior States and the position of the debris

#The opsition of Debris

geometry_msgs/PoseStamped DebrisPose

float32 Length
float32 Radius
float32 PrepDistance

bool LeftHand
bool RightHand
bool BothHands

bool walkflag


bool isRobot
bool isUser

std_msgs/Header plane_header
shape_msgs/Plane plane