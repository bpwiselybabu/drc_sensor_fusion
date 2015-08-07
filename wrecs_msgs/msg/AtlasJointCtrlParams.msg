#Gains for position and force on the real-time loops.

float32 k_q_p     #Position Error Gain, in N*m/rad
float32 k_q_i     #Integral of Position Error Gain, in N*m/(rad*s)
float32 k_qd_p    #Derivative error gain, in N*m/(rad/s)
float32 k_f_p     #Proportional force feedback gain
float32 ff_qd     #Feedforward velocity gain.
float32 ff_qd_d   #Feedforward desired velocity gain. 
float32 ff_f_d    #Feedforward desired force gain.
float32 ff_const  #Constant force term.
