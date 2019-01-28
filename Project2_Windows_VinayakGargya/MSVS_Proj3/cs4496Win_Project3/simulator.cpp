#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(2);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    mTimeStep = 0.0003;
	cps.particles = &mParticles;
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
    mParticles[0].mPosition[1] = 0.0;
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    for (int i = 0; i < getNumParticles(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    
}

void Simulator::simulate() {
    // TODO:
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }
	cps.setup();
	MatrixXd Qdash = cps.solve();
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i].mAccumulatedForce[0] += Qdash(i * 3 + 0);
		mParticles[i].mAccumulatedForce[1] += Qdash(i * 3 + 1);
		mParticles[i].mAccumulatedForce[2] += Qdash(i * 3 + 2);
	}

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep;
        mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;
		
	}
    
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}

//Setup all the necessary vectors
void ConstrainedParticleSystem::setup()
{
	n = particles->size();
	m = 2;// no of constraints
	velocity.resize(3 * n);
	q.resize(3 * n);
	W.resize(3 * n, 3 * n);
	J.resize(m, 3 * n);
	Jdot.resize(m, 3 * n);
	f.resize(3 * n);
	C.resize(m, 1);
	Cdot.resize(m, 1);
	velocity.setZero();
	q.setZero();
	W.setZero();
	J.setZero();
	Jdot.setZero();
	f.setZero();
	C.setZero();
	Cdot.setZero();

	for (int i = 0; i < n; i++)
	{
		//set velocities
		velocity[3*i] = (*particles)[i].mVelocity[0];
		velocity[3*i + 1] = (*particles)[i].mVelocity[1];
		velocity[3*i + 2] = (*particles)[i].mVelocity[2];
	
		//set positions
		q[3 * i] = (*particles)[i].mPosition[0];
		q[3 * i + 1] = (*particles)[i].mPosition[1];
		q[3 * i + 2] = (*particles)[i].mPosition[2];

		f.block(3 * i, 0, 3, 1) = (*particles)[i].mAccumulatedForce;

		//set mass
		W(3 * i + 2, 3 * i + 2) = 1 / ((*particles)[i].mMass);
		W(3 * i + 1, 3 * i + 1) = 1 / ((*particles)[i].mMass);
		W(3 * i, 3 * i) = 1 / ((*particles)[i].mMass);

	}
	//Update Jacobian
	VectorXd deriv1 = c.derivative(&(particles->at(0)));
	J.block(0, 0, 1, 3) = deriv1.transpose();
	VectorXd deriv1p1 = d.derivative(&(particles->at(0)), &(particles->at(1)));
	J.block(1, 0, 1, 3) = deriv1p1.transpose();
	VectorXd deriv1p2 = d.derivative(&(particles->at(1)), &(particles->at(0)));
	J.block(1, 3, 1, 3) = deriv1p2.transpose();

	//Update Jacobian time derivative
	VectorXd deriv2 = c.Jdot(&(particles->at(0)));
	Jdot.block(0, 0, 1, 3) = deriv2.transpose();
	VectorXd deriv2p1 = d.Jdot(&(particles->at(0)), &(particles->at(1)));
	Jdot.block(1, 0, 1, 3) = deriv2p1.transpose();
	VectorXd deriv2p2 = d.Jdot(&(particles->at(1)), &(particles->at(0)));
	Jdot.block(1, 3, 1, 3) = deriv2p2.transpose();

	//Update C
	C(0) = c.C(&(particles->at(0)));
	C(1) = d.C(&(particles->at(0)), &(particles->at(1)));
	
	//Update Cdot
	Cdot(0) = c.Cdot(&(particles->at(0)));
	Cdot(1) = d.Cdot(&(particles->at(0)),&(particles->at(0)));
}

MatrixXd ConstrainedParticleSystem::solve()
{
	float ks = -100.0f, kd = -100.0f;
	MatrixXd Jt = J.transpose();
	MatrixXd temp = (W * Jt);
	MatrixXd invertThis = J * temp;
	MatrixXd JWJ = invertThis.inverse();
	MatrixXd JdotQdot = -1 * Jdot * velocity;
	MatrixXd JWQ = -1 * J * W*f;
	MatrixXd correction = ks*C + kd*Cdot;
	MatrixXd inBw = JWQ + JdotQdot;
	VectorXd lambda = JWJ * inBw;
	MatrixXd Qdash = J.transpose() * lambda;
	return Qdash;
}

//Let's calculate the constraints and their derivatives here	
Eigen::VectorXd ConstraintCircle::derivative(Particle *p)
{
	//double denom = 1 / p->mPosition.norm();
	Eigen::VectorXd deriv = p->mPosition;
	return deriv;
}

Eigen::VectorXd ConstraintCircle::Jdot(Particle * p)
{
	Eigen::VectorXd deriv = p->mVelocity;
	return deriv;
}

double ConstraintCircle::C(Particle * p)
{
	double deriv = p->mPosition.dot(p->mPosition) - radius * radius;
	return (0.5) * deriv;
}
double ConstraintCircle::Cdot(Particle * p)
{
	double deriv = p->mPosition.dot(p->mVelocity);
	return deriv;
}

//Let's calculate the constraints and their derivatives here	
Eigen::VectorXd ConstraintDistance::derivative(Particle *p1, Particle *p2)
{
	//double denom = 1 / p->mPosition.norm();
	Eigen::VectorXd deriv = p1->mPosition - p2->mPosition;
	return deriv;
}

Eigen::VectorXd ConstraintDistance::Jdot(Particle *p1, Particle *p2)
{
	Eigen::VectorXd deriv = p1->mVelocity - p2->mVelocity;
	return deriv;
}

double ConstraintDistance::C(Particle *p1, Particle *p2)
{
	VectorXd diff = (p1->mPosition - p2->mPosition);
	double deriv =  diff.dot(diff) - (dist * dist);
	return (0.5) * deriv;
}
double ConstraintDistance::Cdot(Particle *p1, Particle *p2)
{
	VectorXd diffDist = (p1->mPosition - p2->mPosition);
	VectorXd diffVel = (p1->mVelocity - p2->mVelocity);

	double deriv = diffDist.dot(diffVel);
	return deriv;
}




