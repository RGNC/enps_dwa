/*
    This file is part of ENPS_DWA.
    
    https://github.com/RGNC/enps_dwa
    ENPS_DWA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    RENPSM is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with RENPSM.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef _SIMULATOR_
#define _SIMULATOR_

#include <vector>
#include <cmath>
#include "../lightsfm/vector2d.hpp"
#include "../lightsfm/angle.hpp"
namespace simulator
{

#define PW(x) ((x)*(x))


// VARIABLES

// variables whose values are given by robot on-board sensors
struct InputVariables
{
	// robot pose 
	double r1;
	double r2;
	double theta;

	// robot velocity vector
	double v1;
	double v2;

	// period of execution
	double delta;

	// Obstacle positions
	std::vector<double> o1;
	std::vector<double> o2;

	// People positions
	std::vector<double> p1;
	std::vector<double> p2;

	// People velocities
	std::vector<double> pv1;
	std::vector<double> pv2;

};


// Variables whose values are user-defined
struct UserVariables
{
	// robot goal position
	double g1;
	double g2;

	// weights for user-defined fitness function
	double k1;
	double k2;

	// Set of pedefined velocites
	std::vector<double> linVels;
	std::vector<double> angVels;

};

// Rest of variables
struct SystemVariables
{

	// SFM parameters
	double v;
	double tau;
	double lambda;
	double A;
	double B;
	double n;
	double np;
	double a;
	double b;
	double K1;
	double K2;
	double K3;

	// enzymes for synchronization purposes
	double alpha;
	double alpha0;

	// optimal robot velocity vector
	double vp1;
	double vp2;

	// robot force to the goal position
	double F1;
	double F2;

	// set of distances from the robot to each static obstacle position
	std::vector<double> dp;

	// set of unitary vectors from the robot to each obstacle position
	std::vector<double> ep1;
	std::vector<double> ep2;

	// set of forces from each obstacle position to the robot
	std::vector<double> fp1;
	std::vector<double> fp2;
	
	// set of distances from the robot to each pedestrian position
	std::vector<double> d;	
	
	// set of interaction vectors from the robot to each pedestrian
	std::vector<double> d1;
	std::vector<double> d2;		

	// set of interaction direction from the robot to each pedestrian
	std::vector<double> t1;
	std::vector<double> t2;		

	// set of unitary vectors from the robot position to each pedestrian position
	std::vector<double> e1;
	std::vector<double> e2;	

	// set of social forces from each pedestrian to the robot
	std::vector<double> f1;
	std::vector<double> f2;	

	// set of interaction angles between the robot and pedestrians
	std::vector<double> gamma;

	// variables to accumulate forces
	double sum_fp1;
	double sum_f1;
	double sum_fp2;
	double sum_f2;

	// set of fitness values
	std::vector<double> f;

	// Variables to store the selected motion command
	double selectedLinVel;
	double selectedAngVel;

	unsigned selectedIndex;

};

inline
void initVariables(SystemVariables &v) {
	v.v = 1.2;
	v.tau = 0.5;
	v.lambda = 2.0;
	v.A = 1.0;
	v.B = 0.35;
	v.n = 2.0;
	v.np = 3.0;
	v.a = 1.0;
	v.b = 0.2;
	v.K1 = 2.0;
	v.K2 = 2.1;
	v.K3 = 10.0;
	v.alpha = 1;
	v.alpha0 = 0;
	v.F1 = 0;
	v.F2 = 0;
	v.vp1 = 0;
	v.vp2 = 0;
	v.dp.clear();
	v.ep1.clear();
	v.ep2.clear();
	v.fp1.clear();
	v.fp2.clear();
	v.d.clear();
	v.d1.clear();
	v.d2.clear();
	v.t1.clear();
	v.t2.clear();
	v.e1.clear();
	v.e2.clear();
	v.gamma.clear();
	v.f.clear();
	v.selectedLinVel = 0;
	v.selectedAngVel = 0;
}


inline
double fitness(double linearVelocity, double angularVelocity, double dt, double k1, double k2, 
	         double r1, double r2, double theta, double vp1, double vp2)
{
	// Theoretical pose of the robot (position1, yaw1) after executiong the circular trajectory described by (linVel,angVel) for dt time
	utils::Angle yaw1 = utils::Angle::fromRadian(theta);
	utils::Vector2d position1(r1,r2);
	double imd = linearVelocity * dt;
	utils::Vector2d inc(imd * std::cos(yaw1.toRadian() + angularVelocity*dt*0.5), imd * std::sin(yaw1.toRadian() + angularVelocity*dt*0.5));
	yaw1 += utils::Angle::fromRadian(angularVelocity * dt);	
	position1 += inc;
	
	// Theoretical pose of the robot (position2, yaw2) after following the optimal instant velocity vector (vp1,vp2) for dt time
	utils::Vector2d position2(r1,r2);
	utils::Vector2d v(vp1,vp2);
	position2 += v * dt;
	utils::Angle yaw2 = v.angle();


	utils::Vector2d diffPos = position1 - position2;

	double diffYaw = (yaw1 - yaw2).toRadian();
	if (diffYaw <0) {
		diffYaw = -diffYaw;
	}

	return k1 * diffPos.squaredNorm() + k2 * diffYaw;

}


inline
void run(const InputVariables& i, const UserVariables& u, SystemVariables& s)
{

	unsigned Q = i.p1.size();
	unsigned N = i.o1.size();
	unsigned M = u.linVels.size();


	// p1
	s.F1 = (s.v/ s.tau) * ( (u.g1 - i.r1) / std::sqrt(PW(u.g1 - i.r1) + PW(u.g2 - i.r2)) - i.v1)  ;

	// p2
	s.F2 = (s.v/ s.tau) * ( (u.g2 - i.r2) / std::sqrt(PW(u.g1 - i.r1) + PW(u.g2 - i.r2)) - i.v2)  ;	

	// p3
	s.d.resize(Q);
	for (unsigned k=0;k< Q; k++) {
		s.d[k] = std::sqrt( PW(i.p1[k]- i.r1)  + PW(i.p2[k]- i.r2));
	}
	
	// p4
	s.dp.resize(N);
	for (unsigned k=0;k< N; k++) {
		s.dp[k] = std::sqrt( PW(i.o1[k]- i.r1)  + PW(i.o2[k]- i.r2));
	}

	// p5
	s.e1.resize(Q);
	for (unsigned k=0;k< Q; k++) {
		s.e1[k] = (i.p1[k] - i.r1) / s.d[k];
	}

	// p6
	s.e2.resize(Q);
	for (unsigned k=0;k< Q; k++) {
		s.e2[k] = (i.p2[k] - i.r2) / s.d[k];
	}

	// p7
	s.ep1.resize(N);
	for (unsigned k=0;k< N; k++) {
		s.ep1[k] = -(i.o1[k] - i.r1) / s.dp[k];
	}

	// p8
	s.ep2.resize(N);
	for (unsigned k=0;k< N; k++) {
		s.ep2[k] = -(i.o2[k] - i.r2) / s.dp[k];
	}	

	// p9
	s.d1.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		s.d1[k] = s.lambda  * (i.v1 - i.pv1[k]) + s.e1[k];
	}
	
	// p10
	s.d2.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		s.d2[k] = s.lambda  * (i.v2 - i.pv2[k]) + s.e2[k];
	}
	
	// p11
	s.t1.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		s.t1[k] = s.d1[k] / std::sqrt(PW(s.d1[k]) + PW(s.d2[k])); 
	}

	// p12
	s.t2.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		s.t2[k] = s.d2[k] / std::sqrt(PW(s.d1[k]) + PW(s.d2[k])); 
	}
	
	// p13
	s.gamma.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		s.gamma[k] = std::atan2(s.e2[k],s.e1[k]) - std::atan2(s.t2[k] , s.t1[k]);
		while(s.gamma[k] <= -M_PI) 
			s.gamma[k] += 2*M_PI; 
		while(s.gamma[k] > M_PI) 
			s.gamma[k] -= 2*M_PI; 
	}
	
	// p14
	s.f1.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		double sgn=0;
		if (s.gamma[k]>0) {
			sgn = 1.0;
		} else if (s.gamma[k]<0) {
			sgn = -1.0;
		}
		double B = s.B * std::sqrt(PW(s.d1[k]) + PW(s.d2[k]));
		s.f1[k] = -s.A * std::exp(-s.d[k] / B) *
		(std::exp(-PW(s.np * B * s.gamma[k])) * s.t1[k] 
			- sgn *std::exp(-PW(s.n*B*s.gamma[k])) * s.t2[k]);
	}
	
	// p15
	s.f2.resize(Q);
	for (unsigned k=0;k<Q;k++) {
		double sgn=0;
		if (s.gamma[k]>0) {
			sgn = 1.0;
		} else if (s.gamma[k]<0) {
			sgn = -1.0;
		}
		double B = s.B * std::sqrt(PW(s.d1[k]) + PW(s.d2[k]));
		s.f2[k] = -s.A * std::exp(-s.d[k] / B) *
		(std::exp(-PW(s.np * B * s.gamma[k])) * s.t2[k] 
			+ sgn *std::exp(-PW(s.n*B*s.gamma[k])) * s.t1[k]);
	}	


	// p16
	s.fp1.resize(N);
	for (unsigned k=0;k<N;k++) {
		s.fp1[k] = s.a * std::exp(-s.dp[k]/s.b)*s.ep1[k];
	}

	// p17
	s.fp2.resize(N);
	for (unsigned k=0;k<N;k++) {
		s.fp2[k] = s.a * std::exp(-s.dp[k]/s.b)*s.ep2[k];
	}


	// p19
	s.sum_fp1 = 0;
	for (unsigned k=0;k<N;k++) {
		s.sum_fp1 +=  s.fp1[k];
	}
	
	// p20
	s.sum_f1 = 0;
	for (unsigned k=0;k<Q;k++) {
		s.sum_f1 += s.f1[k];
	}

	// p21
	s.sum_fp2 = 0;
	for (unsigned k=0;k<N;k++) {
		s.sum_fp2 +=  s.fp2[k];
	}
	
	// p22
	s.sum_f2 = 0;
	for (unsigned k=0;k<Q;k++) {
		s.sum_f2 += s.f2[k];
	}

	// p23
	s.vp1 = i.v1 + i.delta*(s.K1 * s.F1 + s.K2 * s.sum_f1 + s.K3 / (N==0?1.0 : (double)N) * s.sum_fp1 );

	// p23
	s.vp2 = i.v2 + i.delta*(s.K1 * s.F2 + s.K2 * s.sum_f2 + s.K3 / (N==0?1.0 : (double)N) * s.sum_fp2 );

	

	// p25 
	s.f.resize(M);
	for (unsigned k=0;k<M;k++) {
		s.f[k] = fitness(u.linVels[k],u.angVels[k],i.delta,u.k1,u.k2,i.r1,i.r2,i.theta,s.vp1,s.vp2);
	}

	// p26 p27
	s.selectedLinVel = u.linVels[0];
	s.selectedAngVel = u.angVels[0];
	s.selectedIndex = 0;
	double minFitness = s.f[0];
	for (unsigned k=1;k< M; k++) {
		if (s.f[k]<minFitness) {
			minFitness = s.f[k];
			s.selectedLinVel = u.linVels[k];
			s.selectedAngVel = u.angVels[k];
			s.selectedIndex = k;
		}
	}
}





}





#endif