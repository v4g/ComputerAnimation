#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(3);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    
    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    
    mTimeStep = 0.05;
    mElapsedTime = 0;
	mInitialVelocity = 0.0f;
	init_vel = 0.0f;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    
    for (int i = 0; i < 3; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
	mElapsedTime = 0;
}

void Simulator::simulate() {
    // TODO: Replace the following code
	if (mElapsedTime == 0.0f)
	{
		init_vel = mInitialVelocity;
	}
	mParticles[1].mPosition[1] += explicitEuler(&(mParticles[1]));
	mParticles[2].mPosition[1] += midPointMethod(&(mParticles[2]));
	mElapsedTime += mTimeStep;
	mParticles[0].mPosition[1] = 20.0 + analyticalMethod(&(mParticles[0]));

}

float Simulator::analyticalMethod(Particle *p)
{
	float deltaY = init_vel * mElapsedTime + (A_GRAVITY * mElapsedTime * mElapsedTime) / 2;
	return deltaY;
}

float Simulator::explicitEuler(Particle *p) {
	float deltaY = derivative(p,mElapsedTime);
	deltaY = deltaY * mTimeStep;
	return deltaY ;
}

float Simulator::derivative(Particle *p, float time)
{
	float deriv = init_vel + (A_GRAVITY * time);
	return deriv;
}

float Simulator::midPointMethod(Particle *p) {
	float deltaAtX = mTimeStep * derivative(p,mElapsedTime)/2;
	if (deltaAtX == 0.0f)
		return 0.0f;
	float newTime = mElapsedTime + (mTimeStep / 2);
	float deltaY = mTimeStep * derivative(p,newTime);
	return deltaY;
}

void Simulator::changeInitialVelocity(float delta)
{
	mInitialVelocity += delta;
}







