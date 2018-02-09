# MATLAB package for kinematic and dynamic analysis and simulation for a planar 3DOF one-legged hopping robot

## Background
See [this PDF](/robotV2/planar3DOF.pdf) for context, derivations, and a numerical forward kinematics algorithm. The algorithm is implemented using the functions described below.

## Scripts:

### [`exploreWorkspace.m`](/robotV2/MATLAB/exploreWorkspace.m)
Script for exploring the robot's workspace and finding foot poses that satisfy joint limits

#### Dependencies:
* [`subchainIK.m`](#subchainikm)
* [`checkJointLimits.m`](#checkjointlimitsm)
* [`jointLimits.mat`](#jointlimitsmat)
* [`plotRobot.m`](#plotrobotm)

### [`testFK.m`](/robotV2/MATLAB/testFK.m)
Script for testing the NRFK algorithm

#### Dependencies:
* [`NRFK.m`](#nrfkm)
* [`constraintVector.m`](#constraintvectorm)
* [`constraintJacobian.m`](#constraintjacobianm)
* [`subchainFK.m`](#subchainfkm)

### [`testJa.m`](/robotV2/MATLAB/testJa.m)
Script for calculating and using the actuator Jacobian for motor sizing

#### Dependencies:
* [`NRqu.m`](#nrqum)
* [`constraintVector.m`](#constraintvectorm)
* [`constraintJacobian.m`](#constraintjacobianm)

## M-functions:
### [`NRFK.m`](/robotV2/MATLAB/NRFK.m)
Computes the foot pose, using the actuated joint positions and the constraint Jacobian. Uses a Newton-Raphson root finder to solve for the unactuated joint positions, then solves the forward kinematics problem along one of the three open sub-chains.

#### Dependencies:
* [`constraintVector.m`](#constraintvectorm)
* [`constraintJacobian.m`](#constraintjacobianm)
* [`subchainFK.m`](#subchainfkm)

### [`NRqu.m`](/robotV2/MATLAB/NRqu.m)
Uses a Newton-Raphson root finder to solve for unactuated joint positions.

#### Dependencies:
* [`constraintVector.m`](#constraintvectorm)
* [`constraintJacobian.m`](#constraintjacobianm)

### [`constraintVector.m`](/robotV2/MATLAB/constraintVector.m)
Calculates a 6x1 vector of the loop-closure expressions.

#### Dependencies:
* [`subchainFK.m`](#subchainfkm)

### [`constraintJacobian.m`](/robotV2/MATLAB/constraintJacobian.m)
Calculates the constraint Jacobian, i.e., the matrix of partial derivatives of the foot pose with respect to the unactuated joint positions.

#### Dependencies:
none

### [`subchainFK.m`](/robotV2/MATLAB/subchainFK.m)
Computes forward kinematics along each sub-chain (theta-, phi-, and psi-chains).

#### Dependencies:
none

### [`subchainIK.m`](/robotV2/MATLAB/subchainIK.m)
Computes inverse kinematics of each sub-chain (theta-, phi-, and psi-chains).

#### Dependencies:
none
