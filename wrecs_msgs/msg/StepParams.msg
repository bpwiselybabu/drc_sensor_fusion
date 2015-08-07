Header header

# Parameters for generating / sending steps
# planner type
int8	      plannerType
int8          FLATGROUNDTOPOINT = 0  # Linear interpolation
int8          FLATGROUNDASTAR = 1    # ROS ARA*/R*
int8          FLATGROUNDASTAR2 = 2
int8          FLATGROUNDTURNWALKTURN = 3
int8          FLATGROUNDSE2 = 4
int8          NUMBEROFSTEPS = 5
int8          FLATGROUND = 6         # use this one for flat ground!
int8          ASTAR = 7
int8          AVOID_OBSTACLES = 8
int8          TERRAIN = 9

int8          StepType
int8          BDI_STEP = 0
int8          CMU_STEP = 1
int8          BDI_WALK = 2

float32        stanceWidth
float32        stepSize
float32        stepWidth
float32        liftHeight      # default value when not using terrain planner
float32        stepHeight      # swing height default value
float32        stepDuration

float32        stepRotation
int32          stepsForward
int32          stepsLeft

bool           endSmall
bool           allowUpperBodyControl
bool           demoMode
bool           beAgressive
bool           firstFootLeft # only works with Linear Interp.
                             # start plan with left foot as first step
bool           allowTurns    # Only used for terrain planner
