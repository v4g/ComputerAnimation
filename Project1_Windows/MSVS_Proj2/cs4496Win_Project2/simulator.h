#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

// class containing objects to be simulated
class Simulator {
public:
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();

	float analyticalMethod(Particle *);

	float explicitEuler(Particle *);

	float midPointMethod(Particle *p);

	float derivative(Particle *p, float time);

	void changeInitialVelocity(float delta);

	float getInitVelocity() { return mInitialVelocity; }

private:
    double mTimeStep;       // time step
    double mElapsedTime;    // time pased since beginning of simulation
    std::vector<Particle> mParticles;
	const float A_GRAVITY = -0.5f;
	float mInitialVelocity;
	float init_vel;
};

#endif  // SIMULATOR_H
