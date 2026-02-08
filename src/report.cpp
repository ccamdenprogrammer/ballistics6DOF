#include "report.hpp"
#include "constants.hpp"
#include "atmosphere.hpp"

#include <fstream>
#include <iomanip>
#include <cmath>


void write_report(const Solver6DOF& solver, const std::string& filename){
    std::ofstream out(filename);

    const auto& cfg = solver.config;
    const auto& proj = solver.proj;
    const auto& recs = solver.flight_records;

    if(recs.empty()) return;

    //unit conversions
    double mass_gr = proj.mass / 0.00006479891;
    double mass_g  = proj.mass * 1000.0;
    double cal_in  = proj.diameter / 0.0254;
    double cal_mm  = proj.diameter * 1000.0;
    double len_mm  = proj.length * 1000.0;
    double mv_fps  = cfg.muzzle_velocity * 3.28084;
    double spin_rpm = cfg.muzzle_velocity / (cfg.twist_rate * 0.0254) * 60.0;
    double elev_deg = cfg.elevation / DEG_TO_RAD;
    double az_deg   = cfg.azimuth / DEG_TO_RAD;
    double lat_deg  = cfg.latitude / DEG_TO_RAD;

    //muzzle atmosphere
    Atmosphere muzzle_atmo = standard_atmo(0.0, 0.0);
    double muzzle_mach = cfg.muzzle_velocity / muzzle_atmo.speed_of_sound;

    //muzzle energy
    double energy_j   = 0.5 * proj.mass * cfg.muzzle_velocity * cfg.muzzle_velocity;
    double energy_ftlb = energy_j * 0.737562;

    //terminal values from last record
    const FlightRecord& last = recs.back();
    double term_energy_j    = 0.5 * proj.mass * last.velocity * last.velocity;
    double term_energy_ftlb = term_energy_j * 0.737562;

    //find apogee
    double apogee = 0.0;
    double apogee_range = 0.0;
    for(const auto& r : recs){
        if(r.altitude > apogee){
            apogee = r.altitude;
            apogee_range = r.range;
        }
    }

    //drop and drift in multiple units
    double drop_m   = last.altitude;
    double drop_ft  = drop_m * 3.28084;
    double drop_in  = drop_m * 39.3701;
    double drift_m  = last.crossrange;
    double drift_in = drift_m * 39.3701;

    //angular drop/drift (only if range > 0)
    double drop_moa  = 0.0;
    double drop_mrad = 0.0;
    double drift_moa  = 0.0;
    double drift_mrad = 0.0;
    if(last.range > 1.0){
        drop_moa  = std::atan2(std::abs(drop_m), last.range) * (180.0 / PI) * 60.0;
        drop_mrad = (std::abs(drop_m) / last.range) * 1000.0;
        drift_moa  = std::atan2(std::abs(drift_m), last.range) * (180.0 / PI) * 60.0;
        drift_mrad = (std::abs(drift_m) / last.range) * 1000.0;
    }


    //write the report
    out << std::fixed;
    out << "=========================================================\n";
    out << "          BALLISTIC 6-DOF FLIGHT REPORT\n";
    out << "=========================================================\n\n";

    //projectile section
    out << "--- Projectile ---\n";
    out << std::setprecision(1);
    out << "  Mass:        " << mass_gr << " gr  (" << mass_g << " g)\n";
    out << std::setprecision(3);
    out << "  Caliber:     " << cal_in << " in  (" << std::setprecision(2) << cal_mm << " mm)\n";
    out << "  Length:      " << len_mm << " mm\n";
    out << std::setprecision(6);
    out << "  Ix (axial):  " << std::scientific << proj.Ix << " kg*m^2\n";
    out << "  Iy (trans):  " << proj.Iy << " kg*m^2\n";
    out << std::fixed;
    out << "\n";

    //launch section
    out << "--- Launch ---\n";
    out << std::setprecision(1);
    out << "  Muzzle Vel:  " << cfg.muzzle_velocity << " m/s  (" << mv_fps << " fps)\n";
    out << "  Twist Rate:  1:" << cfg.twist_rate << " in\n";
    out << std::setprecision(0);
    out << "  Spin Rate:   " << spin_rpm << " RPM\n";
    out << std::setprecision(2);
    out << "  Elevation:   " << elev_deg << " deg\n";
    out << "  Azimuth:     " << az_deg << " deg\n";
    out << std::setprecision(1);
    out << "  Muzzle E:    " << energy_j << " J  (" << energy_ftlb << " ft-lbs)\n";
    out << "\n";

    //environment section
    out << "--- Environment ---\n";
    out << std::setprecision(2);
    out << "  Latitude:    " << lat_deg << " deg\n";
    out << std::setprecision(3);
    out << "  Air Density: " << muzzle_atmo.density << " kg/m^3\n";
    out << std::setprecision(1);
    out << "  Speed/Sound: " << muzzle_atmo.speed_of_sound << " m/s\n";
    out << std::setprecision(3);
    out << "  Muzzle Mach: " << muzzle_mach << "\n";
    out << std::setprecision(1);
    out << "  Wind:        (" << cfg.wind.x() << ", " << cfg.wind.y() << ", " << cfg.wind.z() << ") m/s\n";
    out << "\n";

    //flight summary
    out << "--- Flight Summary ---\n";
    out << std::setprecision(4);
    out << "  Time of Flight: " << last.time << " s\n";
    out << std::setprecision(1);
    out << "  Final Range:    " << last.range << " m  (" << last.range * 1.09361 << " yd)\n";
    out << std::setprecision(2);
    out << "  Apogee:         " << apogee << " m  (at " << std::setprecision(1) << apogee_range << " m)\n";
    out << std::setprecision(1);
    out << "  Terminal Vel:   " << last.velocity << " m/s  (" << last.velocity * 3.28084 << " fps)\n";
    out << std::setprecision(3);
    out << "  Terminal Mach:  " << last.mach << "\n";
    out << std::setprecision(1);
    out << "  Terminal E:     " << term_energy_j << " J  (" << term_energy_ftlb << " ft-lbs)\n";
    out << "\n";

    //drop and drift
    out << "--- Drop & Drift ---\n";
    out << std::setprecision(2);
    out << "  Drop:   " << drop_m << " m  /  " << drop_ft << " ft  /  " << drop_in << " in\n";
    out << std::setprecision(2);
    out << "          " << drop_moa << " MOA  /  " << drop_mrad << " MRAD\n";
    out << std::setprecision(4);
    out << "  Drift:  " << drift_m << " m  /  " << drift_in << " in\n";
    out << std::setprecision(2);
    out << "          " << drift_moa << " MOA  /  " << drift_mrad << " MRAD\n";
    out << "\n";

    out << "=========================================================\n\n";


    //range table
    out << "--- Range Table ---\n\n";
    out << std::right;
    out << std::setw(8)  << "Rng(yd)"
        << std::setw(8)  << "ToF(s)"
        << std::setw(9)  << "Vel(fps)"
        << std::setw(7)  << "Mach"
        << std::setw(10) << "Drop(in)"
        << std::setw(10) << "Drop(MOA)"
        << std::setw(11) << "Drop(MRAD)"
        << std::setw(10) << "Drft(in)"
        << std::setw(10) << "Drft(MOA)"
        << std::setw(11) << "Drft(MRAD)"
        << std::setw(10) << "Energy"
        << "\n";

    out << std::string(114, '-') << "\n";

    for(const auto& r : recs){
        double range_yd  = r.range * 1.09361;
        double vel_fps   = r.velocity * 3.28084;
        double drop_in   = r.altitude * 39.3701;
        double drift_in  = r.crossrange * 39.3701;
        double energy    = 0.5 * proj.mass * r.velocity * r.velocity * 0.737562;

        double d_moa  = 0.0;
        double d_mrad = 0.0;
        double w_moa  = 0.0;
        double w_mrad = 0.0;

        if(r.range > 1.0){
            d_moa  = (r.altitude / r.range) * 3438.0;
            d_mrad = (r.altitude / r.range) * 1000.0;
            w_moa  = (r.crossrange / r.range) * 3438.0;
            w_mrad = (r.crossrange / r.range) * 1000.0;
        }

        out << std::setprecision(1) << std::setw(8)  << range_yd
            << std::setprecision(4) << std::setw(8)  << r.time
            << std::setprecision(1) << std::setw(9)  << vel_fps
            << std::setprecision(3) << std::setw(7)  << r.mach
            << std::setprecision(1) << std::setw(10) << drop_in
            << std::setprecision(2) << std::setw(10) << d_moa
            << std::setprecision(2) << std::setw(11) << d_mrad
            << std::setprecision(1) << std::setw(10) << drift_in
            << std::setprecision(2) << std::setw(10) << w_moa
            << std::setprecision(2) << std::setw(11) << w_mrad
            << std::setprecision(1) << std::setw(10) << energy
            << "\n";
    }

    out << "\n=========================================================\n";
    out.close();
}


void write_csv(const Solver6DOF& solver, const std::string& filename){
    std::ofstream out(filename);

    const auto& cfg = solver.config;
    const auto& proj = solver.proj;
    const auto& recs = solver.flight_records;

    if(recs.empty()) return;

    const FlightRecord& last = recs.back();

    //find apogee
    double apogee = 0.0;
    for(const auto& r : recs){
        if(r.altitude > apogee) apogee = r.altitude;
    }

    //comment header with summary stats for the plotting script
    out << std::fixed;
    out << "# BALLISTIC 6-DOF FLIGHT DATA\n";
    out << std::setprecision(4);
    out << "# tof=" << last.time;
    out << std::setprecision(3);
    out << " apogee=" << apogee;
    out << " impact_vel=" << last.velocity;
    out << " impact_mach=" << last.mach;
    out << "\n";

    double max_mach = 0.0;
    for(const auto& r : recs){
        if(r.mach > max_mach) max_mach = r.mach;
    }

    out << "# max_mach=" << max_mach;
    out << std::setprecision(4);
    out << " final_drift_m=" << last.crossrange;
    out << " final_drop_m=" << last.altitude;
    out << "\n";

    out << std::setprecision(1);
    out << "# muzzle_vel=" << cfg.muzzle_velocity;
    out << " elev_deg=" << cfg.elevation / DEG_TO_RAD;
    out << " twist=" << cfg.twist_rate;
    out << " lat=" << cfg.latitude / DEG_TO_RAD;
    out << "\n";

    out << "# wind_x=" << cfg.wind.x();
    out << " wind_y=" << cfg.wind.y();
    out << " wind_z=" << cfg.wind.z();
    out << "\n";

    out << std::setprecision(6);
    out << "# cal_m=" << proj.diameter;
    out << " len_m=" << proj.length;
    out << " mass_kg=" << proj.mass;
    out << "\n";

    //CSV header row
    out << "time,range,crossrange,altitude,velocity,mach,spin_rpm,pitch,yaw,roll,alpha,beta\n";

    //data rows
    out << std::setprecision(6);
    for(const auto& r : recs){
        out << r.time << ","
            << r.range << ","
            << r.crossrange << ","
            << r.altitude << ","
            << r.velocity << ","
            << r.mach << ","
            << r.spin_rpm << ","
            << r.pitch << ","
            << r.yaw << ","
            << r.roll << ","
            << r.alpha << ","
            << r.beta << "\n";
    }

    out.close();
}
