#include "projectile.hpp"
#include "constants.hpp"


double ref_area(const Projectile& proj){

    //this is the cross sectional area of the projectile (just a circle)
    double S = PI * ((proj.diameter/2.0) * (proj.diameter/2.0));
    return S;
}


void compute_moi(Projectile& proj){

    //compute the moments of inertia for the projectile
    //assuming it's a solid cylinder

    double r = proj.diameter / 2.0; //radius
    double L = proj.length; //length
    double m = proj.mass; //mass

    //moment of inertia about the longitudinal axis (Ix)
    if(proj.Ix == 0.0) proj.Ix = m * (r * r) / 2.0;
    

    //moment of inertia about the transverse axis (Iy)
    if(proj.Iy == 0.0) proj.Iy = m * (3 * (r * r) + (L * L)) / 12.0;
    
    return;
}



//standard .308 winchester projectile
Projectile default_projectile(){
    Projectile proj; 

    //apply default aerodynamic coefficients
    proj.aero = default_aero();

    proj.mass = 0.009719836; //kg (150 grain)
    proj.diameter = 0.00782; //meters (0.308 caliber)
    proj.length = 0.02845; //meters (1.1 inches)
    proj.Ix = 0.0; //will compute 
    proj.Iy = 0.0; //will compute 

    compute_moi(proj);

    return proj;

}
    