#ifndef PARTICLE_H
#define PARTICLE_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

// class for spline
class Particle {
public:
    Particle() {
        // Create a default particle
        mMass = 1.0;
        mPosition.setZero();
        mVelocity.setZero();
        mAccumulatedForce.setZero();
        mColor << 0.9, 0.2, 0.2, 1.0; // Red
		mInitialVelocity.setZero();
    }
    virtual ~Particle() {}
    
    void draw();
    
    double mMass;
    Eigen::Vector3d mPosition;
    Eigen::Vector3d mVelocity;
    Eigen::Vector3d mAccumulatedForce;
	Eigen::Vector3d mInitialVelocity;
    Eigen::Vector4d mColor;
};

#endif  // PARTICLE_H
