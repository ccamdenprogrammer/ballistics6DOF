#include "aero.hpp"


double lookup_cd(const AeroCoeffs& aero, double mach){

    if(mach <= aero.mach_table[0]) return aero.cd_table[0];
    if(mach >= aero.mach_table.back()) return aero.cd_table.back();


    for(int i = 1; i < aero.mach_table.size(); i++){

        
        
        if(mach <= aero.mach_table[i]){
            double m1 = aero.mach_table[i-1];
            double m2 = aero.mach_table[i];
            double cd1 = aero.cd_table[i-1];
            double cd2 = aero.cd_table[i];

            //linear interpolation
            return cd1 + (cd2 - cd1) * (mach - m1) / (m2 - m1);
        }
    }

    return aero.cd_table.back();
}


AeroCoeffs default_aero(){
    AeroCoeffs aero;

    aero.mach_table = {
        0.00, 0.40, 0.60, 0.70, 0.80, 0.85, 0.90, 0.92,
        0.95, 0.97, 1.00, 1.02, 1.05, 1.10, 1.15, 1.20,
        1.30, 1.50, 1.75, 2.00, 2.25, 2.50, 3.00, 3.50
    };

    aero.cd_table = {
        0.130, 0.130, 0.131, 0.135, 0.145, 0.158, 0.185, 0.210,
        0.280, 0.330, 0.395, 0.410, 0.418, 0.410, 0.395, 0.380,
        0.355, 0.320, 0.290, 0.268, 0.250, 0.236, 0.215, 0.200
    };

    //stability derivatives - will fill in next step
    aero.C_Na = 2.50;
    aero.C_Ma = 2.20;
    aero.C_Mq = -8.00;
    aero.C_lp = -0.012;
    aero.C_Mpa = -0.30;
    aero.C_Ypa = 0.00;

    return aero;
}
    
