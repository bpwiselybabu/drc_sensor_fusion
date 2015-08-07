#Message for representing feedback about behavior transitions
Header header

AtlasBehavior from  #Behavior being transitioned out of
AtlasBehavior to    #Behavior being transitioned into

bool transition_in_progress #A transition is in progress
bool transition_success     #Successful transition.
bool transition_illegal     #Transition failed due to an illegal transition request (like walk->manipulate)
bool com_pos                #Transition failed due to the position of the COM being too far from the center of support
bool com_vel                #Transition failed due to COM velocity being too high
bool joint_vel              #Transition failed due to joint velocities being too high
bool auto_transition        #Transition occurred automatically
bool com_failure            #COM below acceptable threshold, cannot recover.