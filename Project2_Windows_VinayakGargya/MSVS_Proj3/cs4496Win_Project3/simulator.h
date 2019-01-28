#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

struct ConstraintCircle {
	std::vector<int> particles;
	//C = x2 - r2
	double radius = 0.2f;
	ConstraintCircle()
	{
		particles.push_back(0);
	}
	//return a row with derivatives in x,y,z
	Eigen::VectorXd derivative(Particle *p);
	Eigen::VectorXd Jdot(Particle * p);
	double C(Particle * p);
	double Cdot(Particle * p);

};

struct ConstraintDistance {
	std::vector<int> particles;
	//C = x2 - r2
	double dist = 0.1f;
	ConstraintDistance()
	{
		particles.push_back(0);
	}
	//return a row with derivatives in x,y,z
	Eigen::VectorXd derivative(Particle *p1, Particle *p2);
	Eigen::VectorXd Jdot(Particle * p1, Particle * p2);
	double C(Particle * p1, Particle * p2);
	double Cdot(Particle * p1, Particle *p2);

};

//We need a position vector 3n
//A velocity vector 3n
//Force vector 3n
//Mass matrix 3n*3n
//Constraints vector m
//Constraint derivative matrix J = m*3n  
//J dot m*3n
//Constraint 1 - Equation of a circle
//Constraint 2 - Distance constraint
using namespace Eigen;
struct ConstrainedParticleSystem
{
	int n; //number of particles
	int m; //number of constraints

	std::vector<Particle> *particles;
	VectorXd velocity;
	VectorXd q;			//positions
	VectorXd f;			//forces
	MatrixXd W;			//mass inverse
	MatrixXd J;			//Jacobian
	MatrixXd Jdot;		//Jacobian gradient
	MatrixXd C;		//Jacobian gradient
	MatrixXd Cdot;		//Jacobian gradient
	VectorXd lambda;	//Lagrangians

	ConstraintCircle c;
	ConstraintDistance d;

	void setup();
	MatrixXd solve();
};


// class containing objects to be simulated
class Simulator {
public:
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();
private:
    double mTimeStep;       // time step
    std::vector<Particle> mParticles;
	ConstrainedParticleSystem cps;
};

#endif  // SIMULATOR_H
