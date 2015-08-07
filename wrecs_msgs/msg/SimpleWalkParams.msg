#Message for generating steps
Header header

# What walking/stepping interface to use
int32 walk_mode
int32 WALK_MODE_NO_CHANGE=0       # Don't change the walk mode
int32 WALK_MODE_BDI_WALK=1
int32 WALK_MODE_BDI_STEP=2
int32 WALK_MODE_CMU_WALK=3
int32 WALK_MODE_CMU_STEP=4

# If true, clear old steps out of queue
# if false, append new steps to old steps currently in queue
bool clear_old_steps

# How many steps to take
# When walking sideways Atlas will take 2x num_steps
# When turning in palce num_steps is ignored
# When walking forwards/backwards Atlas will take num_steps+1 (to align feet)
#
# Open issue - sometimes we don't want a last step when appending steps
int32 num_steps

# Which type of steps to generate
int32 step_type
int32 STEP_TYPE_FORWARD = 0
int32 STEP_TYPE_BACKWARD = 1
int32 STEP_TYPE_LEFT = 2
int32 STEP_TYPE_RIGHT = 3
int32 STEP_TYPE_IN_PLACE = 4

# amount to turn (in radians)
float32 turn_amount

# some general and BDI specific parameters
# meters, seconds, meters, meters
float32 swingHeight
float32 stepDuration
float32 stepWidth
float32 stepSize

# Demo walk parameter for BDI walk
bool use_demo_walk

# take smaller steps at end of walk to slow down
bool endSmall
