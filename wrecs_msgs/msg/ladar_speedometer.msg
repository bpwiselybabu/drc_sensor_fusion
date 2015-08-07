Header header

# This message (and topic) is used to transmit the velocity as calculated from the
# /multisense/laser/scan topic.

# sequence of the laser_scan message that was used for the speed calculation
int64       laser_scan_sequence_id

# the calculated velocity
float64      calc_velocity
