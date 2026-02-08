#pragma once

#include "state.hpp"
#include "projectile.hpp"
#include "flight_record.hpp"

#include <vector>
#include <Eigen/Dense>


struct LaunchConfig {
    double muzzle_velocity; //m/s
    double twist_rate; //twist rate in inches per turn
    double elevation; //launch angle above hotizontal in radians
    double azimuth; //compass direction of launch in radians (0 = North)
    double latitude; //need this for coriolis
    double dt; //time step for integration
    double max_range; //range where we stop the simulation, in meters


    Eigen::Vector3d wind;  //wind velocity in the Earth fram in m/s
};

class Solver6DOF {
    public:

    Projectile proj;
    std::vector<FlightRecord> flight_records;
    void run();
    LaunchConfig config;
    

    private:
    State state;
    double time;
    StateDot derivatives(const State& s, double t);
    void init();
    State apply(const State& s, const StateDot& sd, double dt);
    State rk4_step(const State& s, double t, double dt);
    
};






