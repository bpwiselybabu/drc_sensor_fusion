# Status message from the step_generator2 node

# Is the planner busy planning?
bool planning

# Did the planner succeed?
# Only valid if planning is false (IE the planner is done)
bool success

# Is the robot walking?
bool walking

# How many steps are left?
int32   remaining_steps
