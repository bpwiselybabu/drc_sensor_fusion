#Message for representing the feedback for the Walk behavior
Header header

#Data
float32 t_ste_rem          #Estimated step time remaining before new step data is needed. 
int32   current_step_index #Current step index
int32   next_step_index    #Next step index

AtlasStepData[] desired_step_saturated #Set of desired steps modified by any constraints (saturated)

#Status flags
bool walk_substate_swaying       #Walk is preparing to take a its first step by swaying the body. This flag does not latch. Only one of WALK_SUBSTATE_SWAYING, WALK_SUBSTATE_STEPPING, or WALK_SUBSTATE_CATCHING will be set at any given time.
bool walk_substate_stepping      #Walk is actively stepping. This flag does not latch.
bool walk_substate_catching      #Walk has stopped stepping and is preparing to go back to Stand. This flag does not latch.
bool walk_insufficient_step_data #Walk was unable to find the number of steps it was looking for and has triggered an attempt to go back to Stand. (Does not latch.)
bool walk_inconsistent_steps     #Walk detected the user tried to take 2 steps with the same foot, and attempt to go back into Stand.