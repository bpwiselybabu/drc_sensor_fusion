#ifndef ATLAS_JOINT_LIMITS_HPP
#define ATLAS_JOINT_LIMITS_HPP


#include <string>

const int NUM_JOINTS = 28;

struct sldr_info {
	const std::string name;
	const std::string disp_name;

	float pos_min;
	float pos_max;
	const std::string l_hint;
	const std::string r_hint;
	const int joint_num;
	unsigned char which_CMD;
	bool categoryLine;
//	float frc_min;
//	float frc_max;
};


extern sldr_info sldr_array[];



#endif

