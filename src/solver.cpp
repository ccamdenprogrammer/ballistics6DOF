#include "solver.hpp"
#include "constants.hpp"
#include "atmosphere.hpp"

#include <cmath>




void Solver6DOF::init(){


    //position init (1.5m muzzle height above ground)
    state.pos = Eigen::Vector3d(0.0, 0.0, 1.5);

    //velocity init
    state.vel = Eigen::Vector3d(
        config.muzzle_velocity * std::cos(config.elevation) * std::cos(config.azimuth),
        config.muzzle_velocity * std::cos(config.elevation) * std::sin(config.azimuth),
        config.muzzle_velocity * std::sin(config.elevation)
    );


    //quaternion init
    state.quat = Eigen::AngleAxisd(config.azimuth, Eigen::Vector3d::UnitZ()) 
               * Eigen::AngleAxisd(-config.elevation, Eigen::Vector3d::UnitY());

    //angular velocity init
    double spin = config.muzzle_velocity / (config.twist_rate * 0.0254) * 2*PI;
    state.omega = Eigen::Vector3d(spin, 0.0, 0.0);

    //time init
    time = 0.0;
}


StateDot Solver6DOF::derivatives(const State& s, double t){
    StateDot s_dot;

    //position derivative is just velocity
    s_dot.vel = s.vel;

    //quaternion derivative (I already have this)
    s_dot.quat_Dot = quat_derivative(s.quat, s.omega);

    //angular velocity derivative
    s_dot.omega_Dot = Eigen::Vector3d(0.0, 0.0, 0.0); 


    //DRAG----------------------------------------------

    //atmospheric properties at current altitude
    Atmosphere atmo = standard_atmo(s.pos.z(), 0.0);

    //airpseed in earth fram
    Eigen::Vector3d v_air_earth = s.vel - config.wind;

    //airpseed in body frame
    Eigen::Vector3d v_air_body = s.quat.conjugate() * v_air_earth;

    //total airspeed
    double V = v_air_body.norm(); //total speed
    double mach = V / atmo.speed_of_sound; //mach number
    double Q = 0.5 * atmo.density * V * V; //dynamic pressure
    double S = ref_area(proj); //reference area


    if (V < 1.0) {
        s_dot.acc = Eigen::Vector3d(0.0, 0.0, -g); //just gravity if we're not moving
        return s_dot;
    }

    double alpha = std::atan2(v_air_body.z(), v_air_body.x()); //angle of attack
    double beta = std::asin(v_air_body.y() / V); //sideslip angle

    //aero moments for torque
    double d = proj.diameter;
    double p_bar = s.omega.x() * d / (2.0 * V); //p_bar is how fast the projtile is spinning relative to air
    double q_bar = d / (2.0 * V); //scaling for dampling moments

    //drag lookup and drag computation
    double cd = lookup_cd(proj.aero, mach);
    //drag must always oppose airspeed direction (sign follows v_air_body.x)
    double sign_vx = (v_air_body.x() >= 0.0) ? 1.0 : -1.0;
    Eigen::Vector3d F_aero_body(-sign_vx * Q * S * cd, 0.0, 0.0);

    //Normal Fore
    F_aero_body.y() += -Q * S * proj.aero.C_Na * beta;
    F_aero_body.z() += -Q * S * proj.aero.C_Na * alpha;


    //Magnus Effect
    F_aero_body.y() += Q * S * proj.aero.C_Ypa * p_bar * alpha;
    F_aero_body.z() += -Q * S * proj.aero.C_Ypa * p_bar * beta;

    //rotate back to earth and get acceleration
    Eigen::Vector3d F_aero_earth = s.quat * F_aero_body;

    //coriollis effect
    Eigen::Vector3d omega_earth(
        EARTH_OMEGA * std::cos(config.latitude),
        0.0,
        EARTH_OMEGA * std::sin(config.latitude)
    );
    

    Eigen::Vector3d a_coriolis = -2.0 * omega_earth.cross(s.vel);

    //update gravity to invlude drag
    s_dot.acc = F_aero_earth / proj.mass + Eigen::Vector3d(0.0, 0.0, -g) + a_coriolis;


    //mopment computations
    double Mx = Q * S * d * d * proj.aero.C_lp * p_bar;
    double My = Q * S * d * proj.aero.C_Ma * alpha
              + Q * S * d * d * proj.aero.C_Mq * s.omega.y() * q_bar;

    double Mz = -Q * S * d * proj.aero.C_Ma * beta             
               + Q * S * d * d * proj.aero.C_Mq * s.omega.z() * q_bar;  

    //Magnus moments
    My += Q * S * d * proj.aero.C_Mpa * p_bar * (-beta);
    Mz += Q * S * d * proj.aero.C_Mpa * p_bar * alpha;

    
    s_dot.omega_Dot.x() = Mx / proj.Ix;
    s_dot.omega_Dot.y() = (My - (proj.Ix - proj.Iy) * s.omega.z() * s.omega.x()) / proj.Iy;
    s_dot.omega_Dot.z() = (Mz - (proj.Iy - proj.Ix) * s.omega.x() * s.omega.y()) / proj.Iy;

    return s_dot;
}







//RK4 INTEGRATION-------------------------------------------------------------------------
State Solver6DOF::apply(const State& s, const StateDot& sd, double dt){
    State result;
    result.pos  = s.pos + sd.vel * dt;
    result.vel  = s.vel + sd.acc * dt;
    result.quat = Eigen::Quaterniond(
        s.quat.w() + sd.quat_Dot.w() * dt,
        s.quat.x() + sd.quat_Dot.x() * dt,
        s.quat.y() + sd.quat_Dot.y() * dt,
        s.quat.z() + sd.quat_Dot.z() * dt
    ).normalized();
    result.omega = s.omega + sd.omega_Dot * dt;
    return result;
}

State Solver6DOF::rk4_step(const State& s, double t, double dt){
    StateDot k1 = derivatives(s, t);
    StateDot k2 = derivatives(apply(s, k1, dt/2), t + dt/2);
    StateDot k3 = derivatives(apply(s, k2, dt/2), t + dt/2);
    StateDot k4 = derivatives(apply(s, k3, dt), t + dt);

    // Blend: (k1 + 2*k2 + 2*k3 + k4) / 6
    StateDot avg;
    avg.vel      = (k1.vel + 2*k2.vel + 2*k3.vel + k4.vel) / 6.0;
    avg.acc      = (k1.acc + 2*k2.acc + 2*k3.acc + k4.acc) / 6.0;
    avg.omega_Dot = (k1.omega_Dot + 2*k2.omega_Dot + 2*k3.omega_Dot + k4.omega_Dot) / 6.0;
    avg.quat_Dot = Eigen::Quaterniond(
        (k1.quat_Dot.w() + 2*k2.quat_Dot.w() + 2*k3.quat_Dot.w() + k4.quat_Dot.w()) / 6.0,
        (k1.quat_Dot.x() + 2*k2.quat_Dot.x() + 2*k3.quat_Dot.x() + k4.quat_Dot.x()) / 6.0,
        (k1.quat_Dot.y() + 2*k2.quat_Dot.y() + 2*k3.quat_Dot.y() + k4.quat_Dot.y()) / 6.0,
        (k1.quat_Dot.z() + 2*k2.quat_Dot.z() + 2*k3.quat_Dot.z() + k4.quat_Dot.z()) / 6.0
    );

    return apply(s, avg, dt);
}


void Solver6DOF::run(){
    init();

    double next_record_range = 0.0;
    double record_interval = 9.144; // 10 yards in meters

    while(true){


        //TERMINATION CONDITIONS CHECKING-------------------------------------------------
        // Advance one step
        state = rk4_step(state, time, config.dt);
        time += config.dt;

        // Check termination conditions
        // 1. Ground impact (z < 0 and x > 10)
        if(state.pos.z() < 0.0 && state.pos.x() > 10.0){
            break;
        }
        // 2. Max range exceeded
        if(state.pos.x() > config.max_range){
            break;
        }
        // 3. Downrange velocity < 1 m/s
        if(state.vel.x() < 1.0){
            break;
        }
        // 4. Time > 120 seconds
        if(time > 120.0){
            break;
        }


        //RECORD FLIGHT DATA--------------------------------------------------------------
        if(state.pos.x() >= next_record_range){
        // Build a FlightRecord and push it into flight_records
        FlightRecord rec;
        rec.time = time;
        rec.range = state.pos.x();
        rec.crossrange = state.pos.y();
        rec.altitude = state.pos.z();
        rec.velocity = state.vel.norm();
    
        Atmosphere atmo = standard_atmo(state.pos.z(), 0.0);
        rec.mach = state.vel.norm() / atmo.speed_of_sound;
        rec.spin_rpm = state.omega.x() * 60.0 / (2.0 * PI);
    
        Eigen::Vector3d euler = quat_to_euler(state.quat);
        rec.yaw = euler.x();
        rec.pitch = euler.y();
        rec.roll = euler.z();
    
        // Compute alpha and beta for the record
        Eigen::Vector3d v_air = state.vel - config.wind;
        Eigen::Vector3d v_body = state.quat.conjugate() * v_air;
        double V = v_body.norm();
        rec.alpha = std::atan2(v_body.z(), v_body.x());
        rec.beta = (V > 1.0) ? std::asin(v_body.y() / V) : 0.0;
    
        rec.pos = state.pos;
    
        flight_records.push_back(rec);
        next_record_range += record_interval;
        }

    }




}
