#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>


struct State{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond quat;
    Eigen::Vector3d omega;
};


struct StateDot{
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond quat_Dot;
    Eigen::Vector3d omega_Dot;
};

//quaterion dervative helper function. tells us how the quaternions changes based
//projectile angular velocity
inline Eigen::Quaterniond quat_derivative(const Eigen::Quaterniond& q, const Eigen::Vector3d& omega){
    Eigen::Quaterniond omega_q(0.0, omega.x(), omega.y(), omega.z());
    Eigen::Quaterniond product = q * omega_q;
    return Eigen::Quaterniond(product.w() * 0.5,
                              product.x() * 0.5,
                              product.y() * 0.5,
                              product.z() * 0.5);
}


//helper function to convert quaternion to euler angles (roll, pitch, yaw)
inline Eigen::Vector3d quat_to_euler(const Eigen::Quaterniond& q){
    Eigen::Vector3d angles = q.toRotationMatrix().canonicalEulerAngles(2, 1, 0);
    return angles;
}