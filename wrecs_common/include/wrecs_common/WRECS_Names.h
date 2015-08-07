#ifndef WRECS_NAMES_H
#define WRECS_NAMES_H

#include <string>
namespace WRECS_NAMES
{
	/********* MSGS *********/

	static const std::string FIELD_STATE = "/field_state";
	static const std::string FIELD_COMMAND = "/field_command";

	/* Atlas */

	//Status
	static const std::string ATLAS_RUN_STATE_TOPIC = "/atlas/run_state";
	static const std::string ATLAS_BEHAVIOR_FEEDBACK_TOPIC = "/atlas/behavior/behavior_feedback";
	static const std::string ATLAS_BEHAVIOR_CURRENT_TOPIC = "/atlas/behavior/current_behavior";
	static const std::string ATLAS_MANIP_FEEDBACK_TOPIC = "/atlas/behavior/manip_feedback";
	static const std::string ATLAS_STAND_FEEDBACK_TOPIC = "/atlas/behavior/stand_feedback";
	static const std::string ATLAS_STEP_FEEDBACK_TOPIC = "/atlas/behavior/step_feedback";
	static const std::string ATLAS_WALK_FEEDBACK_TOPIC = "/atlas/behavior/walk_feedback";
	static const std::string ATLAS_SENSOR_FOOT_FORCE_LEFT_TOPIC = "/atlas/sensors/foot/force_left";
	static const std::string ATLAS_SENSOR_FOOT_FORCE_RIGHT_TOPIC = "/atlas/sensors/foot/force_right";
	static const std::string ATLAS_SENSOR_IMU_TOPIC = "/atlas/sensors/imu/imu";
	static const std::string ATLAS_SENSOR_FOOT_POSE_LEFT_TOPIC = "/atlas/sensors/pose/left_foot";
	static const std::string ATLAS_SENSOR_FOOT_POSE_RIGHT_TOPIC = "/atlas/sensors/pose/right_foot";
	static const std::string ATLAS_SENSOR_ROBOT_POSE_TOPIC = "/atlas/sensors/pose/robot_pose";
	static const std::string ATLAS_SENSOR_ROBOT_TWIST_TOPIC = "/atlas/sensors/pose/robot_twist";
	static const std::string ATLAS_SENSOR_PUMP_PRESSURE_TOPIC = "/atlas/sensors/pump/pump_pressure";
	static const std::string ATLAS_SENSOR_WRIST_FORCE_RIGHT_TOPIC = "/atlas/sensors/wrist/force_right";
	static const std::string ATLAS_SENSOR_WRIST_FORCE_RIGHT_FILTERED_TOPIC = "/atlas/sensors/wrist/force_right_filtered";
	static const std::string ATLAS_SENSOR_WRIST_FORCE_LEFT_TOPIC = "/atlas/sensors/wrist/force_left";
	static const std::string ATLAS_SENSOR_BATTERY_TOPIC = "/atlas/sensors/battery_status";
    static const std::string ATLAS_MULTISENSE_TOPIC = "atlas/multisense/status";
	//Control

	
	/* GUI MSGS */
	static const std::string GUI_ATLAS_BEHAVIOR_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string GUI_ATLAS_BEHAVIOR_STATUS_TOPIC = "/ocu/field_state";
	static const std::string GUI_PUMP_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string GUI_PUMP_STATUS_TOPIC = "/ocu/pump_status";
	static const std::string GUI_SENSOR_BATTERY_TOPIC = "/ocu/atlas/sensors/battery_status";
	static const std::string GUI_ERROR_STATUS_TOPIC = "/ocu/field_state";
	static const std::string GUI_STEERING_CONTROL_TOPIC = "/ocu/car_steering";
	static const std::string GUI_VELOCITY_CONTROL_TOPIC = "/ocu/car_gas_brake";
	static const std::string GUI_DRIVE_AHEAD_CONTROL_TOPIC = "/ocu/car_drive_ahead_time";
	static const std::string GUI_CAR_COMMAND_TOPIC = "/ocu/car_command";
	static const std::string GUI_CAR_WHEEL_LOC_TOPIC = "/ocu/car_steering_wheel_loc";
	static const std::string GUI_JOINT_SLIDERS_STATUS_TOPIC = "/ocu/joint_states";
	static const std::string GUI_JOINT_SLIDERS_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string GUI_TASK_SELECTION_TOPIC = "/ocu/task_selection";
    static const std::string GUI_HAND_POWER_TOPIC = "/ocu/atlas/hand_power_command";
	static const std::string GUI_HAND_NUDGE_TOPIC = "/atlas/hand_nudge_command";

	static const std::string GUI_START_PREP_TOPIC = "drc_gui/start_prep";
	static const std::string GUI_ATLAS_ERROR_STATUS_TOPIC = "drc_gui/atlas_error_status";
	static const std::string GUI_DRILL_SHAKE_TOPIC = "robot_motion/drill_shaking";
	static const std::string GUI_DRILL_TRAJECTORY_TOPIC = "/robot_motion/drill_trajectory";

	

	/* Pipeline MSGS */
	static const std::string OCU_GUI_COMM_STATUS_TOPIC = "/ocu/comm_status";
	static const std::string OCU_GUI_COMM_SIM_ON_TOPIC = "/ocu/comm_sim_on";
	static const std::string OCU_GUI_COMM_SIM_BPS_TOPIC = "/ocu/comm_sim_bps";
	static const std::string OCU_GUI_COMM_DROPOUT_TOPIC = "/ocu/comm_dropout";
	static const std::string OCU_GUI_COMM_LATENCY_TOPIC = "/ocu/comm_latency";
	static const std::string OCU_GUI_COMM_PROBABILITY_TOPIC = "/ocu/comm_probability";
	static const std::string OCU_GUI_COMM_BANDWIDTH_TOPIC = "/ocu/comm_bandwidth";
	
	static const std::string OCU_GUI_DRILL_SHAKE_TOPIC = "/ocu/robot_motion/drill_shaking";
	static const std::string OCU_GUI_DRILL_TRAJECTORY_TOPIC = "/ocu/robot_motion/drill_trajectory";
	static const std::string OCU_GUI_REQUEST_JPEG_TOPIC = "/ocu/atlas/request_jpeg";
	static const std::string OCU_ATLAS_SENSOR_WRIST_FORCE_RIGHT_TOPIC = "/ocu/atlas/sensors/wrist/force_right";
	static const std::string OCU_ATLAS_SENSOR_WRIST_FORCE_RIGHT_FILTERED_TOPIC = "/ocu/atlas/sensors/wrist/force_right_filtered";
	static const std::string OCU_GUI_PLUG_TOPIC = "/ocu/tasks/plug";
	static const std::string OCU_GUI_SHUFFLE_TOPIC = "/ocu/tasks/shuffle";
    static const std::string OCU_GUI_MULTISENSE_TOPIC = "/ocu/multisensecontrol";

	static const std::string PAUSE_TASK_TOPIC = "/tasks/pause_task";


		// Pipeline GUI
		//these are copied here because the constants above are in use --JoshGraff
		//The below are in use on the OCU_Ros2 
	static const std::string OCU_GUI_ATLAS_BEHAVIOR_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string OCU_GUI_ATLAS_BEHAVIOR_STATUS_TOPIC = "/ocu/field_state";
	static const std::string OCU_GUI_PUMP_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string OCU_GUI_PUMP_STATUS_TOPIC = "/ocu/field_state";
	static const std::string OCU_GUI_ERROR_STATUS_TOPIC = "/ocu/field_state";
	static const std::string OCU_GUI_JOINT_SLIDERS_STATUS_TOPIC = "/ocu/joint_states";
	static const std::string OCU_GUI_JOINT_SLIDERS_CONTROL_TOPIC = "/ocu/field_command";
	static const std::string OCU_GUI_STEERING_CONTROL_TOPIC = "/ocu/car_steering";
	static const std::string OCU_GUI_VELOCITY_CONTROL_TOPIC = "/ocu/car_gas_brake";
	static const std::string OCU_GUI_DRIVE_AHEAD_CONTROL_TOPIC = "/ocu/car_drive_ahead_time";
	static const std::string OCU_GUI_CAR_COMMAND_TOPIC = "/ocu/car_command";
	static const std::string OCU_GUI_SCRIBBLE_TOPIC = "/ocu/drc_gui/scribble";
	static const std::string OCU_GUI_OBJECT_TYPE_TOPIC = "/ocu/drc_gui/object_type";
	static const std::string OCU_DRILL_FSM_CONTROL_TOPIC = "/ocu/drill/fsm_control";
	static const std::string OCU_DRILL_FSM_FEEDBACK = "/ocu/drill/fsm_feedback";
	static const std::string OCU_GUI_PERCEPTION_FEEDBACK="/ocu/gui/perception_feedback";


	static const std::string GUI_PERCEPTION_FEEDBACK="/gui/perception_feedback";
	static const std::string DRILL_FSM_CONTROL_TOPIC="/drill/fsm_control";
	static const std::string DRILL_FSM_FEEDBACK = "/drill/fsm_feedback";



	//Posing MSGS
	static const std::string DRILL_CONTROL_POINT_STAMPED = "/JPG_POINT";

	//laser topics
	static const std::string MULTISENSE_LASER_SCAN_TOPIC = "/multisense/lidar_scan";
	static const std::string MULTISENSE_LASER_CLOUD_TOPIC = "/multisense/lidar_points2";
	static const std::string MULTISENSE_LASER_RAW_TOPIC = "/multisense/calibration/raw_lidar_data";

	//image top
	static const std::string MULTISENSE_LEFT_IMAGE_COLOR_TOPIC = "/multisense/left/image_rect_color";
	static const std::string MULTISENSE_LEFT_DISPARITY_TOPIC = "/multisense/left/disparity";
	static const std::string MULTISENSE_LEFT_DEPTH_TOPIC = "/multisense/left/depth";
	static const std::string MULTISENSE_STEREO_CLOUD_TOPIC = "/multisense/image_points2";
	static const std::string MULTISENSE_RAW_CAM_CONFIG_TOPIC = "/multisense/calibration/raw_cam_config";
	static const std::string MULTISENSE_STEREO_CLOUD_COLOR_TOPIC = "/multisense/image_points2";



	//imu topic
	static const std::string MULTISENSE_IMU_TOPIC = "/atlas/sensors/imu/imu";

	//multisense control topics
	static const std::string MULTISENSE_CONTROL_SERVICE = "/multisense/set_parameters";
	static const std::string MULTISENSE_LED_DUTY_CYCLE = "/multisense/led_duty_cycle";

	//multisense GAZEBO ONLY Topics
	static const std::string MULTISENSE_CONTROL_MOTOR_TOPIC = "/multisense/set_spindle_speed";
	static const std::string MULTISENSE_CONTROL_FPS_TOPIC = "/multisense/set_fps";

	//topics generated by us
	static const std::string ASSEMBLED_LASER_CLOUD_TOPIC = "/kin_odom_pub/laser_assembled";//"/wrecs/points2";

	//scribble topics
	static const std::string SCRIBBLE_TOPIC = "/drc_gui/scribble";
	static const std::string OBJECT_TYPE_TOPIC = "/drc_gui/object_type";

	//slam topic
	static const std::string KINEMATIC_POSE_TOPIC = "/kinematic_head_pose";
	static const std::string SLAM_POSE_TOPIC = "/slam_head_pose";

	//services

	static const std::string PLANE_DETECTION_SERVICE = "/perception/atlas_plane_point_service";
	static const std::string VISUAL_HAND_SERVO_SERVICE = "/perception/hand_servo_service";
	static const std::string OPERATIONAL_MODE_SERVICE = "/atlas/operational_mode_service";
	static const std::string WALK_ACTION_SERVER = "/walkToTargetAS";
	static const std::string DETECTION_SERVER = "/detect_debrisAS";


	/************************/

	/********* TF Frames *********/

	/* Multisense */
	static const std::string HOKUYO_LINK_TF = "/hokuyo_link";
	static const std::string HEAD_HOKUYO_FRAME_TF = "/head_hokuyo_frame";
	static const std::string LEFT_CAMERA_OPTICAL_FRAME_TF = "/left_camera_optical_frame";

	//Atlas Hand
	static const std::string R_HAND_TF = "r_hand";
	static const std::string R_PALM_TF = "r_palm";
	static const std::string L_HAND_TF = "l_hand";
	static const std::string L_PALM_TF = "l_palm";

	/* Atlas Torso*/
	static const std::string PELVIS_TF = "/pelvis";
	static const std::string IMU_LINK_TF = "/imu_link";

	/* Atlas Right Foot*/
	static const std::string R_UGLUT_TF = "/r_uglut";
	static const std::string R_LGLUT_TF = "/r_lglut";
	static const std::string R_ULEG_TF = "/r_uleg";
	static const std::string R_LLEG_TF = "/r_lleg";
	static const std::string R_TALUS_TF = "/r_talus";
	static const std::string R_FOOT_TF = "/r_foot";

	/* Atlas Left Foot*/
	static const std::string L_UGLUT_TF = "/l_uglut";
	static const std::string L_LGLUT_TF = "/l_lglut";
	static const std::string L_ULEG_TF = "/l_uleg";
	static const std::string L_LLEG_TF = "/l_lleg";
	static const std::string L_TALUS_TF = "/l_talus";
	static const std::string L_FOOT_TF = "/l_foot";


/*****************************/

}

#endif
