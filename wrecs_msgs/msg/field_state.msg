Header header

# Robot position / orientation / acceleration
float32[3]  robot_position         # 0=x,1=y,2=z
float32[4]  orientation_quat       # 0=x,1=y,2=z,3=w
float32[3]  imu_acc                # filtered imu acc from body IMU

# current joint positions and forces
float32[30] atlas_state
float32[30] atlas_forces

float32[30] atlas_velocities       # joint velocities - not available on OCU
#float32[30] q_out
#float32[30] qd_out


# feedback from the wrist/ankle force/torque sensors
# 3 from each ankle, 6 from each wrist
# lfoot, rfoot, lhand, rhand (xyz torques, xyz forces)
float32[18] force_torques


int8 bdiMode                       # BDI mode (user, stand, freeze, etc.)
int8 control_mode                  # what the current control mode is
int8 gain_set                      # what the current gain set is


# feedback from the trapezoidal velocity controller - 1 indicates velocity loop is complete
int8 movement_complete

int32      bdiErr                  # error status from BDI
int8       unexpected_pump_stop
float32    air_sump_pressure
int16      current_pump_rpm
int8       battery_percent
