// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008  Mikael Mayer
// Copyright  (C)  2008  Julia Jesse

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "wrecs_common/chainiksolverpos_nr_jl2.h"
#include <iostream> // for cout
#include <stdio.h>

namespace KDL
{
    ChainIkSolverPos_NR_JL2::ChainIkSolverPos_NR_JL2(const Chain& _chain, const JntArray& _q_min, const JntArray& _q_max, ChainFkSolverPos& _fksolver,ChainIkSolverVel& _iksolver,
                                             unsigned int _maxiter, double _eps):
        chain(_chain), q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints()), fksolver(_fksolver),iksolver(_iksolver),delta_q(_chain.getNrOfJoints()),
        maxiter(_maxiter),eps(_eps)
    {
    	q_min = _q_min;
    	q_max = _q_max;
    }


// The inverse kinematics velocity solver that is passed in to the constructor frequently diverges and gets stuck
// It would be a good place to look for the next round of improvements


// Mode 0 - Normal
// Mode 1 - Ignore rotation, score for position only
// Mode 2 - Ignore Z rotation
// Mode 3 - Use fewer search steps to speed up search, score for steering
// Mode 4 - Score considering minimal delta v
// Mode 5 - Use with hand type 5 (pipe out side of hand)

double ChainIkSolverPos_NR_JL2::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out, int mode)
 {
     JntArray q_rough = q_min;
     JntArray q_tmp = q_max;
     JntArray q_fine = q_max;
     // First do a search
     int i1, i2, i3, i4, i5, i6;
     int icnt = 8;
     double score;
     double best_score = 1000000000000.0;

     static JntArray saved_q_init(6);
     for (int i = 0; i < 6; i++) {
	 saved_q_init(i) = q_init(i);
     }


     if ((mode == 3) || (mode == 5)) {
         icnt = 6;
     }

     // For the first pass - step across the range
     // If icnt = 10, try at 5%, 15%, 25%, 35%, 45%, 55%, 65%, 75%, 85%, 95%
     // Searches could be much faster if the forward kinematics was integrated with this loop
     // right now the full solution is calculated for every point, but only one joint needs to be calculated in the inner loop
     for (i1 = 0; i1 < icnt; i1++) {
	 q_out(0) = q_min(0) + (q_max(0) - q_min(0)) * (((double)i1 + 0.5) / (double) icnt);
	 for (i2 = 0; i2 < icnt; i2++) {
	     q_out(1) = q_min(1) + (q_max(1) - q_min(1)) * (((double)i2 + 0.5) / (double) icnt);
	     for (i3 = 0; i3 < icnt; i3++) {
		 q_out(2) = q_min(2) + (q_max(2) - q_min(2)) * (((double)i3 + 0.5) / (double) icnt);
		 for (i4 = 0; i4 < icnt; i4++) {
		     q_out(3) = q_min(3) + (q_max(3) - q_min(3)) * (((double)i4 + 0.5) / (double) icnt);
		     for (i5 = 0; i5 < icnt; i5++) {
			 q_out(4) = q_min(4) + (q_max(4) - q_min(4)) * (((double)i5 + 0.5) / (double) icnt);
			 for (i6 = 0; i6 < icnt; i6++) {
			     q_out(5) = q_min(5) + (q_max(5) - q_min(5)) * (((double)i6 + 0.5) / (double) icnt);
			     fksolver.JntToCart(q_out,f);
			     score = myscore(p_in, f, mode);
			     //std::cout << score << " " << q_out(0) << " " << q_out(1)  << " ";
			     //std::cout << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << std::endl;

			     if (mode == 4) {
				 score+= myscore4(saved_q_init, q_init, q_out);
			     }

			     if (score < best_score) {
				 best_score = score;
				 q_rough = q_out;
			     }
			 }
		     }
		 }
	     }
	 }
     }
     //printf("Best search rough %6.3lf ", best_score);
     //printf("%6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf\n", q_rough(0),q_rough(1),q_rough(2),q_rough(3),q_rough(4),q_rough(5));


     // Second pass - fine tuning loop
     // If icnt2 = 10, try at -4%, -3%, -2%, -1%, 0%, 1%, 2%, 3%, 4%
     best_score = 1000000000000.0;
     int icnt2 = 8;
     int icntstart = -4;
     int icntend = 3;

     if ((mode == 3) || (mode == 5)) {
         icnt2 = 6;
	 icntstart = -3;
	 icntend = 2;
     }

     for (i1 = icntstart; i1 < icntend; i1++) {
	 q_out(0) = q_rough(0) + (q_max(0) - q_min(0)) * i1 * (1.0 / (icnt * icnt2));
	 for (i2 = icntstart; i2 < icntend; i2++) {
	     q_out(1) = q_rough(1) + (q_max(1) - q_min(1)) * i2 * (1.0 / (icnt * icnt2));
	     for (i3 = icntstart; i3 < icntend; i3++) {
		 q_out(2) = q_rough(2) + (q_max(2) - q_min(2)) * i3 * (1.0 / (icnt * icnt2));
		 for (i4 = icntstart; i4 < icntend; i4++) {
		     q_out(3) = q_rough(3) + (q_max(3) - q_min(3)) * i4 * (1.0 / (icnt * icnt2));
		     for (i5 = icntstart; i5 < icntend; i5++) {
			 q_out(4) = q_rough(4) + (q_max(4) - q_min(4)) * i5 * (1.0 / (icnt * icnt2));
			 for (i6 = icntstart; i6 < icntend; i6++) {
			     q_out(5) = q_rough(5) + (q_max(5) - q_min(5)) * i6 * (1.0 / (icnt * icnt2));
			     fksolver.JntToCart(q_out,f);
			     score = myscore(p_in, f, mode);
			     if (mode == 4) {
				 score+= myscore4(saved_q_init, q_init, q_out);
			     }
			     //std::cout << score << " " << q_out(0) << " " << q_out(1)  << " ";
			     //std::cout << q_out(2) << " " << q_out(3) << " " << q_out(4) << " " << q_out(5) << std::endl;
			     if (score < best_score) {
				 best_score = score;
				 q_fine = q_out;
			     }
			 }
		     }
		 }
	     }
	 }
     }
     //printf("Best search fine  %6.3lf ", best_score);
     //printf("%6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf\n", q_fine(0),q_fine(1),q_fine(2),q_fine(3),q_fine(4),q_fine(5));
     if ((mode == 3) || (mode == 4) || (mode == 5)) {
         q_out = q_fine;
	 return best_score;
     }

     // Try different initial states for solver
     // the current joint state, the zero state, full minimum, full maximum, 5 randoms, rough search, fine search
     unsigned int i, j, k;
     best_score = 1000000000000.0;
     for (j = 0; j < 11; j++) {
	 if (j == 0) { // Use initial angles - default
	     q_out = q_init;
	 }
	 else if (j == 1) { // Use zero angles
	     for (k = 0; k < q_min.rows(); k++) {
		 q_out(k) = 0.0;
	     }
	 }
	 else if (j == 2) {
	     q_out = q_min;
	 }
	 else if (j == 3) {
	     q_out = q_max;
	 }
	 else if (j == 9) {
	     q_out = q_rough;
	 }
	 else if (j == 10) {
	     q_out = q_fine;
	 }
         else { // Random initial positions
	     for (k = 0; k < q_min.rows(); k++) {
		 q_out(k) = q_min(k) + ((double)rand()) * (q_max(k) - q_min(k)) / RAND_MAX;
	     }
	 }


	 for(i=0;i<maxiter;i++) {
	     //std::cout << " here is i " << i << std::endl;
	     fksolver.JntToCart(q_out,f);
	     if (i == 0) {
		 score = myscore(p_in, f, mode);
		 //printf("Start Error       %6.3lf ", score);
		 //printf("%6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf\n", q_out(0),q_out(1),q_out(2),q_out(3),q_out(4),q_out(5));

		 if (score < best_score) {
		     best_score = score;
		     q_tmp = q_out;
		 }

	     }
	     delta_twist = diff(f,p_in);
	     if(Equal(delta_twist,Twist::Zero(),eps)) {
		 std::cout << "Found match " << i << std::endl;
	         return 0; // We have found one that is good enough
	     }
	     iksolver.CartToJnt(q_out,delta_twist,delta_q);
	     Add(q_out,delta_q,q_out);
	     
	     //printf("                         %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf\n", q_out(0),q_out(1),q_out(2),q_out(3),q_out(4),q_out(5));


	     for(unsigned int j=0; j<q_min.rows(); j++) {
	         if(q_out(j) < q_min(j))
		     q_out(j) = q_min(j);
	     }
	     
	     
	     for(unsigned int j=0; j<q_max.rows(); j++) {
	         if(q_out(j) > q_max(j))
		     q_out(j) = q_max(j);
	     }
	 }
	 fksolver.JntToCart(q_out,f);
	 score = myscore(p_in, f, mode);
	 //printf("End Error         %6.3lf ", score);
	 //printf("%6.3lf %6.3lf %6.3lf %6.3lf %6.3lf %6.3lf\n", q_out(0),q_out(1),q_out(2),q_out(3),q_out(4),q_out(5));

	 //for (k = 0; k < q_min.rows(); k++) {
	 //    std::cout << q_out(k) << " ";
	 // }
	 //std::cout <<std::endl;
	 
	 if (score < best_score) {
	     best_score = score;
	     q_tmp = q_out;
	 }
     }
     q_out = q_tmp; // return the closest match
     return best_score; // We have found one that is good enough
 }


double ChainIkSolverPos_NR_JL2::CartToJntWrist(const JntArray& q_init, const Frame& p_in, JntArray& q_out, int mode)
 {
     JntArray q_rough = q_min;
     int i1, i2;
     int icnt = 64;
     double score;
     double best_score = 1000000000000.0;



     // For the first pass - step across the range
     // If icnt = 10, try at 5%, 15%, 25%, 35%, 45%, 55%, 65%, 75%, 85%, 95%
     // Searches could be much faster if the forward kinematics was integrated with this loop
     // right now the full solution is calculated for every point, but only one joint needs to be calculated in the inner loop
     for (i1 = 0; i1 < icnt; i1++) {
	 q_out(0) = q_min(0) + (q_max(0) - q_min(0)) * (((double)i1 + 0.5) / (double) icnt);
	 for (i2 = 0; i2 < icnt; i2++) {
	     q_out(1) = q_min(1) + (q_max(1) - q_min(1)) * (((double)i2 + 0.5) / (double) icnt);
	     fksolver.JntToCart(q_out,f);
	     score = myscore(p_in, f, mode);
	     if (score < best_score) {
	         best_score = score;
	         q_rough = q_out;
	     }
	 }
     }

     q_out = q_rough;
     return best_score;
 }

    ChainIkSolverPos_NR_JL2::~ChainIkSolverPos_NR_JL2()
    {
    }

  // f1 is desired / goal pose
    double ChainIkSolverPos_NR_JL2::myscore(Frame f1, Frame f2, int mode)
    {
	if ((mode == 3) || (mode == 4) || (mode == 5)) {
	    Vector tmp_vector;
	    tmp_vector.x(0.0);
	    tmp_vector.y(0.1);
	    tmp_vector.z(0.0);
	    Vector des_tip = f1 * tmp_vector;
	    tmp_vector.x(0.0);
	    tmp_vector.y(0.1);
	    tmp_vector.z(0.0);
	    if (mode == 5) {
		tmp_vector.x(0.0);
		tmp_vector.y(0.0);
		tmp_vector.z(0.1);
	    }
	    Vector actual_tip = f2 * tmp_vector;
	    double score = fabs(f1(0,3) - f2(0,3)) + fabs(f1(1,3) - f2(1,3)) + fabs(f1(2,3) - f2(2,3)) +
		0.33 * (fabs(actual_tip.x() - des_tip.x()) + fabs(actual_tip.y() - des_tip.y()) + fabs(actual_tip.z() - des_tip.z()));
	    return score;
	}

	// Weighting of position higher than orientation array
        // Better would consider distance rather than just sum of x,y,z
	double rotWgt = 0.1; // Rotation weights
	double rotWgtz = 0.1; // Rotation weights
	if (mode == 1) {
	    rotWgt = 0.0;
	    rotWgtz = 0.0;
	}
	else if (mode == 2) {
	    rotWgt = 0.0;
	    rotWgtz = 0.1;
	}
	double score = fabs(f1(0,3) - f2(0,3)) + fabs(f1(1,3) - f2(1,3)) + fabs(f1(2,3) - f2(2,3)) +
	    rotWgt * fabs(f1(0,0) - f2(0,0)) + rotWgt * fabs(f1(1,0) - f2(1,0)) + rotWgt * fabs(f1(2,0) - f2(2,0)) +
	    rotWgt * fabs(f1(0,1) - f2(0,1)) + rotWgtz * fabs(f1(1,1) - f2(1,1)) + rotWgtz * fabs(f1(2,1) - f2(2,1)) +
	    rotWgt * fabs(f1(0,2) - f2(0,2)) + rotWgtz * fabs(f1(1,2) - f2(1,2)) + rotWgtz * fabs(f1(2,2) - f2(2,2));
	return score;
    }

    float ChainIkSolverPos_NR_JL2::myscore4(const JntArray& saved_q_init, const JntArray& q_init, const JntArray& q_out)
    {
	return 0;
	float score = 0.0;
	float deltav;
	for (int i = 4; i < 6; i++) {
	    deltav = (q_out(i) - q_init(i)) - (q_init(i) - saved_q_init(i));
	    score += fabs(deltav) * 0.05;
	}
	for (int i = 0; i < 4; i++) { // weight the bigger joints higher
	    deltav = (q_out(i) - q_init(i)) - (q_init(i) - saved_q_init(i));
	    score += fabs(deltav) * 0.1;
	}
	/*
	if (score > 0.02) {
	    score = 0.02;
	}
	*/
	return score;
    }

}

