#pragma once

#include "aero.hpp"

struct Projectile{

    double mass; //kg
    double diameter; //caliber in meters
    double length; //bullet length in meters
    double Ix; //axial moment of inertia
    double Iy; //transverse moment of inertia

    AeroCoeffs aero;
};


double ref_area(const Projectile& proj);
void compute_moi(Projectile& proj);
Projectile default_projectile();