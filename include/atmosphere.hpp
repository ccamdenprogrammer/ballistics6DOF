#pragma once



struct Atmosphere{
    double temperature;
    double pressure;
    double speed_of_sound;
    double density;
};


Atmosphere standard_atmo(double altitude, double temp_offset);