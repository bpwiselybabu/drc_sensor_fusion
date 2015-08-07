#Message for representing the feedback for the Step behavior
Header header

#Data
float32 t_ste_rem          #Estimated step time remaining before new step data is needed. 
int32   current_step_index #Current step index
int32   next_step_index    #Next step index

AtlasStepData desired_step_saturated #Desired step data modified by any constraints (saturated)

#Status flags
bool step_substate_swaying  #Feet are in double support. This flag does not latch. Only one of STEP_SUBSTATE_SWAYING or STEP_SUBSTATE_STEPPING will be set at any given time.
bool step_substate_stepping #Actively stepping; one foot is in the air. This flag does not latch.