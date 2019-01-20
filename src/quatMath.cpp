#include "quatMath.h"

Eigen::Matrix3d quat2rot(const Eigen::VectorXd& q){
  Eigen::Matrix3d T = Eigen::MatrixXd::Identity(3,3)*pow(q(3),2);
  T -= (q.head(3).transpose()*q.head(3))*Eigen::MatrixXd::Identity(3,3);
  T += 2*q.head(3)*q.head(3).transpose();
  T -= 2*q(3)*crossProductEquivalent(q.head(3));
  return T;
}
Eigen::VectorXd quatRot(const Eigen::VectorXd& q, const Eigen::VectorXd& dq){
  Eigen::VectorXd qOut = (Eigen::MatrixXd::Identity(4,4)*dq(3)+skew(dq))*q;
  return qOut;
}
Eigen::MatrixXd skew(const Eigen::VectorXd& q){
  Eigen::MatrixXd mat(4,4);
  mat << 0    ,q(2) ,-q(1),q(0),
         -q(2),0    ,q(0) ,q(1),
         q(1) ,-q(0),0    ,q(2),
         -q(0),-q(1),-q(2),0   ;
  return mat;
}
Eigen::Matrix3d crossProductEquivalent(const Eigen::Vector3d& a){
  Eigen::Matrix3d ax;
  ax << 0,-a(2),a(1),
        a(2),0,-a(0),
        -a(1),a(0),0;
  return ax;
}
Eigen::Vector4d inverse(const Eigen::Vector4d& q){
  Eigen::Vector4d qinv;
  qinv << -q(0),-q(1),-q(2),q(3);
  return qinv;
}
