#ifndef ATLASLIMITS_H
#define ATLASLIMITS_H

#include <string>
//#define NO_LIMITS

#ifndef NO_LIMITS
/* Back */
#define LBZ_POS_MIN -0.65
#define LBZ_POS_MAX 0.65

#define MBY_POS_MIN -0.2
#define MBY_POS_MAX 0.52

#define UBX_POS_MIN -0.52
#define UBX_POS_MAX 0.52

/* Neck */
#define AY_POS_MIN -0.59
#define AY_POS_MAX 1.13

/* Left Leg */
#define L_UHZ_POS_MIN -0.17
#define L_UHZ_POS_MAX 0.78

#define L_MHX_POS_MIN -0.52
#define L_MHX_POS_MAX 0.52

#define L_LHY_POS_MIN -1.57
#define L_LHY_POS_MAX 0.6

#define L_KNY_POS_MIN 0.0
#define L_KNY_POS_MAX 2.35

#define L_UAY_POS_MIN -0.9
#define L_UAY_POS_MAX 0.48

#define L_LAX_POS_MIN -0.43
#define L_LAX_POS_MAX 0.43

/* Right Leg */
#define R_UHZ_POS_MIN -0.78
#define R_UHZ_POS_MAX 0.17

#define R_MHX_POS_MIN -0.52
#define R_MHX_POS_MAX 0.52

#define R_LHY_POS_MIN -1.57
#define R_LHY_POS_MAX 0.6

#define R_KNY_POS_MIN 0.0
#define R_KNY_POS_MAX 2.35

#define R_UAY_POS_MIN -0.9
#define R_UAY_POS_MAX 0.48

#define R_LAX_POS_MIN -0.43
#define R_LAX_POS_MAX 0.43

/* Left Arm */
#define L_USY_POS_MIN -1.57
#define L_USY_POS_MAX 0.78

#define L_SHX_POS_MIN -1.57
#define L_SHX_POS_MAX 1.57

#define L_ELY_POS_MIN 0.0
#define L_ELY_POS_MAX 3.14

#define L_ELX_POS_MIN 0.0
#define L_ELX_POS_MAX 2.31

#define L_UWY_POS_MIN 0.0
#define L_UWY_POS_MAX 3.15

#define L_MWX_POS_MIN -1.18
#define L_MWX_POS_MAX 1.18

/* Right Arm */
#define R_USY_POS_MIN -0.78
#define R_USY_POS_MAX 1.57

#define R_SHX_POS_MIN -1.57
#define R_SHX_POS_MAX 1.57

#define R_ELY_POS_MIN 0.0
#define R_ELY_POS_MAX 3.14

#define R_ELX_POS_MIN -2.31
#define R_ELX_POS_MAX 0.0

#define R_UWY_POS_MIN 0.0
#define R_UWY_POS_MAX 3.15

#define R_MWX_POS_MIN -1.18
#define R_MWX_POS_MAX 1.18

#endif

#ifdef NO_LIMITS
/* Back */
#define LBZ_POS_MIN -2
#define LBZ_POS_MAX 2

#define MBY_POS_MIN -2
#define MBY_POS_MAX 2

#define UBX_POS_MIN -2
#define UBX_POS_MAX 2

/* Neck */
#define AY_POS_MIN -2
#define AY_POS_MAX 2

/* Left Leg */
#define L_UHZ_POS_MIN -2
#define L_UHZ_POS_MAX 2

#define L_MHX_POS_MIN -2
#define L_MHX_POS_MAX 2

#define L_LHY_POS_MIN -2
#define L_LHY_POS_MAX 2

#define L_KNY_POS_MIN -3
#define L_KNY_POS_MAX 3

#define L_UAY_POS_MIN -3
#define L_UAY_POS_MAX 3

#define L_LAX_POS_MIN -3
#define L_LAX_POS_MAX 3

/* Right Leg */
#define R_UHZ_POS_MIN -3
#define R_UHZ_POS_MAX 3

#define R_MHX_POS_MIN -3
#define R_MHX_POS_MAX 3

#define R_LHY_POS_MIN -3
#define R_LHY_POS_MAX 3

#define R_KNY_POS_MIN -3
#define R_KNY_POS_MAX 3

#define R_UAY_POS_MIN -3
#define R_UAY_POS_MAX 3

#define R_LAX_POS_MIN -3
#define R_LAX_POS_MAX 3

/* Left Arm */
#define L_USY_POS_MIN -3
#define L_USY_POS_MAX 3

#define L_SHX_POS_MIN -3
#define L_SHX_POS_MAX 3

#define L_ELY_POS_MIN -4
#define L_ELY_POS_MAX 4

#define L_ELX_POS_MIN -4
#define L_ELX_POS_MAX 4

#define L_UWY_POS_MIN -4
#define L_UWY_POS_MAX 4

#define L_MWX_POS_MIN -4
#define L_MWX_POS_MAX 4

/* Right Arm */
#define R_USY_POS_MIN -4
#define R_USY_POS_MAX 4

#define R_SHX_POS_MIN -4
#define R_SHX_POS_MAX 4

#define R_ELY_POS_MIN -4
#define R_ELY_POS_MAX 4

#define R_ELX_POS_MIN -4
#define R_ELX_POS_MAX 4

#define R_UWY_POS_MIN -4
#define R_UWY_POS_MAX 4

#define R_MWX_POS_MIN -4
#define R_MWX_POS_MAX 4

#endif

const static double g_pos_min[] = {
/* Back */
LBZ_POS_MIN, MBY_POS_MIN, UBX_POS_MIN,
/* Neck */
AY_POS_MIN,
/* Left Leg */
L_UHZ_POS_MIN, L_MHX_POS_MIN, L_LHY_POS_MIN, L_KNY_POS_MIN, L_UAY_POS_MIN,
		L_LAX_POS_MIN,
		/* Right Leg */
		R_UHZ_POS_MIN, R_MHX_POS_MIN, R_LHY_POS_MIN, R_KNY_POS_MIN,
		R_UAY_POS_MIN, R_LAX_POS_MIN,
		/* Left Arm */
		L_USY_POS_MIN, L_SHX_POS_MIN, L_ELY_POS_MIN, L_ELX_POS_MIN,
		L_UWY_POS_MIN, L_MWX_POS_MIN,
		/* Right Arm */
		R_USY_POS_MIN, R_SHX_POS_MIN, R_ELY_POS_MIN, R_ELX_POS_MIN,
		R_UWY_POS_MIN, R_MWX_POS_MIN };

const static double g_pos_max[] = {
/* Back */
LBZ_POS_MAX, MBY_POS_MAX, UBX_POS_MAX,
/* Neck */
AY_POS_MAX,
/* Left Leg */
L_UHZ_POS_MAX, L_MHX_POS_MAX, L_LHY_POS_MAX, L_KNY_POS_MAX, L_UAY_POS_MAX,
		L_LAX_POS_MAX,
		/* Right Leg */
		R_UHZ_POS_MAX, R_MHX_POS_MAX, R_LHY_POS_MAX, R_KNY_POS_MAX,
		R_UAY_POS_MAX, R_LAX_POS_MAX,
		/* Left Arm */
		L_USY_POS_MAX, L_SHX_POS_MAX, L_ELY_POS_MAX, L_ELX_POS_MAX,
		L_UWY_POS_MAX, L_MWX_POS_MAX,
		/* Right Arm */
		R_USY_POS_MAX, R_SHX_POS_MAX, R_ELY_POS_MAX, R_ELX_POS_MAX,
		R_UWY_POS_MAX, R_MWX_POS_MAX };

const static double g_frc_min[] = { -124.016, -206.843, -94.91, -5, -110, -180,
		-260, -400, -220, -90, -110, -180, -260, -400, -220, -90, -212, -170,
		-114, -114, -114, -60, -212, -170, -114, -114, -114, -60 };
const static double g_frc_max[] = { 124.016, 206.843, 94.91, 5, 110, 180, 260,
		400, 220, 90, 110, 180, 260, 400, 220, 90, 212, 170, 114, 114, 114, 60,
		212, 170, 114, 114, 114, 60 };


#endif

