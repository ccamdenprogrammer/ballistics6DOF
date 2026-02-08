#include "atmosphere.hpp"
#include "constants.hpp"

#include <iostream>
#include <cmath>

Atmosphere standard_atmo(double altitude, double temp_offset){

    //troposphere 0 - 11km
    if(altitude <= 11000.0){
        //temp at altitude T
        double T0 = 288.15 + temp_offset;
        double T = T0 - 0.0065 * altitude;

        //pressure from barometric formula
        double P = 101325.0 * pow(T / T0, 5.2559);

        //density from temp and pressure using ideal gas law
        double d = P / (287.058 * T);

        //speed of sound
        double SOS = sqrt(1.4 * 287.058 * T);

        return Atmosphere{T, P, SOS, d};

    //tropopause 11 - 20km
    } else if(altitude <= 20000.0){

        //temp at altitude T
        double T0 = 288.15 + temp_offset;
        double T = T0 - 0.0065 * 11000.0;
         //p_11 is just the pressure at 11km
        double P_11 = 101325.0 * pow(T / T0, 5.2559);

        double P = P_11 * exp(-g * (altitude-11000.0) / (287.058 * T));

        //density from temp and pressure using ideal gas law
        double d = P / (287.058 * T);

        //speed of sound
        double SOS = sqrt(1.4 * 287.058 * T);

        return Atmosphere{T, P, SOS, d};
    } else if(altitude <= 32000.0){

        //temp at altitude T
        double T0 = 288.15 + temp_offset;
        double T_20 = T0 - 0.0065 * 11000.0;

        double T = T_20 + 0.001 * (altitude - 20000.0);

        //pressure from barometric formula
        double P_11 = 101325.0 * pow(T_20 / T0, 5.2559);
        double P_20 = P_11 * exp(-g * 9000.0 / (287.058 * T_20));
        double P = P_20 * pow(T / T_20, -34.1632);

        //density from temp and pressure using ideal gas law
        double d = P / (287.058 * T);

        //speed of sound
        double SOS = sqrt(1.4 * 287.058 * T);

        return Atmosphere{T, P, SOS, d};
    } else{
        std::cout << "Altitude out of range for standard atmosphere model" << std::endl;
        return Atmosphere{};
    }




    
}
    