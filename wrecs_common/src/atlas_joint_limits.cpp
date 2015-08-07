#include <wrecs_common/AtlasLimits.h>
#include <wrecs_common/atlas_joint_limits.hpp>
//	 All/a lot of the necessary information for joints. All in one place... Isn't that nice?
sldr_info sldr_array[] = {

		//back
		{"back_bkz","back_lbz",LBZ_POS_MIN,LBZ_POS_MAX,"Twist Right","Twist Left",0,0,false},
		{"back_bky","back_mby",MBY_POS_MIN,MBY_POS_MAX,"Lean Back","Lean Forward",1,0,false},
		{"back_bkx","back_ubx",UBX_POS_MIN,UBX_POS_MAX,"Tilt Left","Tilt Right",2,0,true},
		//neck
		{"neck_ry","neck_ry",AY_POS_MIN,AY_POS_MAX,"Up","Down",3,0,true},
		//left leg
		{"l_leg_hpz","l_leg_hpz",L_UHZ_POS_MIN,L_UHZ_POS_MAX,"Rotate In","Rotate Out",4,0,false},
		{"l_leg_hpx","l_leg_hpx",L_MHX_POS_MIN,L_MHX_POS_MAX,"Lateral In","Lateral Out",5,0,false},
		{"l_leg_hpy","l_leg_hpy",L_LHY_POS_MIN,L_LHY_POS_MAX,"Forward","Back",6,0,false},
		{"l_leg_kny","l_leg_kny",L_KNY_POS_MIN,L_KNY_POS_MAX,"Straight","Flex",7,0,false},
		{"l_leg_aky","l_leg_aky",L_UAY_POS_MIN,L_UAY_POS_MAX,"Toe Up","Toe Down",8,0,false},
		{"l_leg_akx","l_leg_akx",L_LAX_POS_MIN,L_LAX_POS_MAX,"OutEdgeDown","EdgeUp",9,0,true},
		//Right leg
		{"r_leg_hpz","r_leg_hpz",R_UHZ_POS_MIN,R_UHZ_POS_MAX,"Rotate Out","Rotate In",10,0,false},
		{"r_leg_hpx","r_leg_hpx",R_MHX_POS_MIN,R_MHX_POS_MAX,"Lateral Out","Lateral In",11,0,false},
		{"r_leg_hpy","r_leg_hpy",R_LHY_POS_MIN,R_LHY_POS_MAX,"Forward","Back",12,0,false},
		{"r_leg_kny","r_leg_kny",R_KNY_POS_MIN,R_KNY_POS_MAX,"Straight","Flex",13,0,false},
		{"r_leg_aky","r_leg_aky",R_UAY_POS_MIN,R_UAY_POS_MAX,"Toe Up","Toe Down",14,0,false},
		{"r_leg_akx","r_leg_akx",R_LAX_POS_MIN,R_LAX_POS_MAX,"OutEdgeUp","EdgeDown",15,0,true},
		//left arm
		{"l_arm_shz","l_arm_shz",L_USY_POS_MIN,L_USY_POS_MAX,"Rot Front","Rot Back",16,0,false},
		{"l_arm_shx","l_arm_shx",L_SHX_POS_MIN,L_SHX_POS_MAX,"Down","Up",17,0,false},
		{"l_arm_ely","l_arm_ely",L_ELY_POS_MIN,L_ELY_POS_MAX,"Rot Out","Rot In",18,0,false},
		{"l_arm_elx","l_arm_elx",L_ELX_POS_MIN,L_ELX_POS_MAX,"Straight","Flex",19,0,false},
		{"l_arm_wry","l_arm_wry",L_UWY_POS_MIN,L_UWY_POS_MAX,"Rotate In","Rotate Out",20,0,false},
		{"l_arm_wrx","l_arm_wrx",L_MWX_POS_MIN,L_MWX_POS_MAX,"Straight","Flex",21,0,true},
		// right arm
		{"r_arm_shz","r_arm_shz",R_USY_POS_MIN,R_USY_POS_MAX,"Rot Back","Rot Front",22,0,false},
		{"r_arm_shx","r_arm_shx",R_SHX_POS_MIN,R_SHX_POS_MAX,"Up","Down",23,0,false},
		{"r_arm_ely","r_arm_ely",R_ELY_POS_MIN,R_ELY_POS_MAX,"Rot Out","Rot In",24,0,false},
		{"r_arm_elx","r_arm_elx",R_ELX_POS_MIN,R_ELX_POS_MAX,"Flex","Straight",25,0,false},
		{"r_arm_wry","r_arm_wry",R_UWY_POS_MIN,R_UWY_POS_MAX,"Rotate out","Rotate in",26,0,false},
		{"r_arm_wrx","r_arm_wrx",R_MWX_POS_MIN,R_MWX_POS_MAX,"Flex","Straight",27,0,false}

};
