#pragma once

#include <Eigen/Dense>





struct FlightRecord{
    //time
    double time;
    //range
    double range;
    //crossrange
    double crossrange;
    //altitude
    double altitude;
    //velocity
    double velocity;
    //mach
    double mach;
    //spin rpm
    double spin_rpm;
    //pitch
    double pitch;
    //yaw
    double yaw;
    //roll
    double roll;
    //alpha
    double alpha;
    //beta
    double beta;
    //pos
    Eigen::Vector3d pos;
};