#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <cstdlib>
#include <filesystem>
#include "solver.hpp"
#include "report.hpp"
#include "constants.hpp"

namespace fs = std::filesystem;


//parse a config file into a key-value map
//format: key = value   (# for comments, blank lines ignored)
std::map<std::string, double> parse_config(const std::string& filename){
    std::map<std::string, double> cfg;
    std::ifstream in(filename);
    if(!in.is_open()){
        std::cerr << "ERROR: Cannot open config file: " << filename << "\n";
        std::exit(1);
    }

    std::string line;
    int line_num = 0;
    while(std::getline(in, line)){
        line_num++;

        //strip comments
        auto pos = line.find('#');
        if(pos != std::string::npos) line = line.substr(0, pos);

        //skip blank lines
        if(line.find_first_not_of(" \t\r\n") == std::string::npos) continue;

        //find '='
        auto eq = line.find('=');
        if(eq == std::string::npos){
            std::cerr << "WARNING: Skipping line " << line_num << " (no '=' found): " << line << "\n";
            continue;
        }

        //extract key and value
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        //trim whitespace
        auto trim = [](std::string& s){
            size_t start = s.find_first_not_of(" \t\r\n");
            size_t end   = s.find_last_not_of(" \t\r\n");
            s = (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
        };
        trim(key);
        trim(val);

        if(key.empty() || val.empty()) continue;

        try {
            cfg[key] = std::stod(val);
        } catch(...) {
            std::cerr << "WARNING: Skipping line " << line_num << " (bad number): " << val << "\n";
        }
    }

    return cfg;
}


//get a value from the config map, or return a default
double get(const std::map<std::string, double>& cfg, const std::string& key, double default_val){
    auto it = cfg.find(key);
    if(it != cfg.end()) return it->second;
    return default_val;
}


int main(int argc, char* argv[]) {

    if(argc < 2){
        std::cerr << "Usage: ballistic6dof <config_file>\n";
        std::cerr << "  See configs/default.cfg for an example.\n";
        return 1;
    }

    std::string config_path = argv[1];
    auto cfg = parse_config(config_path);

    //output directory = same folder as the config file
    fs::path cfg_dir = fs::path(config_path).parent_path();
    if(cfg_dir.empty()) cfg_dir = ".";

    std::string report_path = (cfg_dir / "ballistic_report.txt").string();
    std::string csv_path    = (cfg_dir / "ballistic_trajectory.csv").string();


    std::cout << "=========================================================\n";
    std::cout << "          BALLISTIC 6-DOF FLIGHT SIMULATOR\n";
    std::cout << "=========================================================\n";
    std::cout << "  Config: " << config_path << "\n\n";


    //--- Projectile parameters ---
    double mass_gr   = get(cfg, "mass_gr",       150.0);
    double cal_in    = get(cfg, "caliber_in",    0.308);
    double length_in = get(cfg, "length_in",     1.12);

    Projectile proj;
    proj.aero     = default_aero();
    proj.mass     = mass_gr * 0.00006479891;   //grains to kg
    proj.diameter = cal_in * 0.0254;            //inches to meters
    proj.length   = length_in * 0.0254;         //inches to meters
    proj.Ix = 0.0;
    proj.Iy = 0.0;
    compute_moi(proj);


    //Launch parameters
    double mv_fps   = get(cfg, "muzzle_velocity_fps", 2750.0);
    double twist    = get(cfg, "twist_rate",           12.0);
    double elev_deg = get(cfg, "elevation_deg",        0.5);
    double az_deg   = get(cfg, "azimuth_deg",          0.0);


    //Environment parameters
    double lat_deg = get(cfg, "latitude_deg",    45.0);
    double wind_x  = get(cfg, "headwind",        0.0);
    double wind_y  = get(cfg, "crosswind",       0.0);
    double wind_z  = get(cfg, "vertical_wind",   0.0);


    //Simulation parameters
    double max_range = get(cfg, "max_range_m",   1000.0);
    double dt        = get(cfg, "timestep_us",   50.0);


    //Print loaded config
    std::cout << "--- Projectile ---\n";
    std::cout << "  Mass:      " << mass_gr   << " gr\n";
    std::cout << "  Caliber:   " << cal_in    << " in\n";
    std::cout << "  Length:    " << length_in << " in\n\n";

    std::cout << "--- Launch ---\n";
    std::cout << "  Muzzle Vel: " << mv_fps   << " fps\n";
    std::cout << "  Twist Rate: " << twist    << " in/turn\n";
    std::cout << "  Elevation:  " << elev_deg << " deg\n";
    std::cout << "  Azimuth:    " << az_deg   << " deg\n\n";

    std::cout << "--- Environment ---\n";
    std::cout << "  Latitude:   " << lat_deg << " deg\n";
    std::cout << "  Headwind:   " << wind_x  << " m/s\n";
    std::cout << "  Crosswind:  " << wind_y  << " m/s\n";
    std::cout << "  Vert wind:  " << wind_z  << " m/s\n\n";

    std::cout << "--- Simulation ---\n";
    std::cout << "  Max range:  " << max_range << " m\n";
    std::cout << "  Timestep:   " << dt << " us\n\n";


    //Build solver
    Solver6DOF solver;
    solver.proj = proj;

    solver.config.muzzle_velocity = mv_fps / 3.28084;     //fps to m/s
    solver.config.twist_rate      = twist;
    solver.config.elevation       = elev_deg * DEG_TO_RAD;
    solver.config.azimuth         = az_deg * DEG_TO_RAD;
    solver.config.latitude        = lat_deg * DEG_TO_RAD;
    solver.config.wind            = Eigen::Vector3d(wind_x, wind_y, wind_z);
    solver.config.dt              = dt * 1e-6;             //microseconds to seconds
    solver.config.max_range       = max_range;


    //Run
    std::cout << "Running simulation..." << std::endl;
    solver.run();
    std::cout << "Done! Recorded " << solver.flight_records.size() << " data points.\n\n";


    //Write output files
    write_report(solver, report_path);
    write_csv(solver, csv_path);

    std::cout << "Report written to: " << report_path << "\n";
    std::cout << "CSV written to:    " << csv_path << "\n\n";


    //Launch plotting script
    std::string python      = "C:\\Users\\camde\\AppData\\Local\\Python\\bin\\python.exe";
    std::string plot_script = "../plot/plot_trajectory.py";
    std::string anim_script = "../plot/animate_3d.py";

    //Windows cmd.exe needs the whole command wrapped in outer quotes
    //when inner paths are also quoted:  cmd /c "  "exe" "arg"  "
    auto sys_cmd = [](const std::string& exe, const std::string& script, const std::string& arg){
        std::string cmd = "\"\"" + exe + "\" \"" + script + "\" \"" + arg + "\"\"";
        return std::system(cmd.c_str());
    };

    std::cout << "Launching plots...\n";
    sys_cmd(python, plot_script, csv_path);

    //--- Launch 3D animation ---
    std::cout << "Launching 3D animation...\n";
    sys_cmd(python, anim_script, csv_path);

    return 0;
}
