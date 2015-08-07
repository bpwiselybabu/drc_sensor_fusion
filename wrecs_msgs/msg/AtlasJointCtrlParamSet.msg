########################
# Message for representing a set of Joint Control Parameters
########################

Header header

string[]                              name      #The set of joint names that the parameter set belongs to
AtlasJointCtrlParams[] param_set #The set of joint control parameters. Must be keyed to mach the joint name entry in names