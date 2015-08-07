Header header

# This message (and topic) is used to log data from the field_state executable
# Ideally rosbag would use UDP to go along with this

float64    timestamp
uint64     field_state_log_cnt

# what version of the log data is this (so old data can be used)
uint32     log_version


# what type of data is this
# LOG_TYPE_DATA is for logging algorithmic data
# LOG_TYPE_NULL is a null message sent at the beginning since rosbag record / playback
#    cycle tends to lose the first 150 or so messages
uint32     log_type
uint32     LOG_TYPE_ATLAS_DATA_TO_ROBOT = 1
uint32     LOG_TYPE_ATLAS_DATA_FROM_ROBOT  = 2
uint32     LOG_TYPE_ATLAS_CONTROL_CALL = 3
uint32     LOG_TYPE_DATA = 4
uint32     LOG_TYPE_NULL = 5

# return value of the call
int64      return_value

# sequence id of the call
int64      sequence_id

# List of function calls
# expand to cover the key atlas controls
uint32     CALL_TYPE_    = 1

# only used for logging data - could probably be in the data
uint32      number_of_values

byte[]    data
