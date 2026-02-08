#pragma once
#include <vector>

struct AeroCoeffs{

    std::vector<double> mach_table;
    std::vector<double> cd_table;

    double C_Na;
    double C_Ma;
    double C_Mq;
    double C_lp;
    double C_Mpa;
    double C_Ypa;
};

double lookup_cd(const AeroCoeffs& aero, double mach);
AeroCoeffs default_aero();