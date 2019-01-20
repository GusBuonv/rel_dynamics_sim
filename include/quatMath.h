#ifndef QUATMATH_H
#define QUATMATH_H

#include <math.h>
#include <Eigen/Dense>

Eigen::Matrix3d quat2rot(const Eigen::VectorXd& q);
Eigen::VectorXd quatRot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
Eigen::MatrixXd skew(const Eigen::VectorXd& q);
Eigen::Matrix3d crossProductEquivalent(const Eigen::Vector3d& a);
Eigen::Vector4d inverse(const Eigen::Vector4d& q);
#endif