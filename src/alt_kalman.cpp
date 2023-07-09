#include "alt_kalman.hpp"
//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>

Alt_kalman::Alt_kalman(){};

void Alt_kalman::update(double z_sens, double accel)
{
        //update kalman gain
        k1 = p12/(p22+R);
        k2 = p22/(p22+R);

        //estimate state
        velocity_ = velocity + k1*(z_sens - altitude);
        altitude_ = altitude + k2*(z_sens - altitude);
        velocity = velocity_;
        altitude = altitude_;
        
        //Estimated state (output)
        Velocity = velocity;
        Altitude = altitude;
        //printf("%11.3f %11.4f %11.4f %11.4f %11.4f ", t+step, velocity, altitude, z_sens, true_v);

        //estimate P
        p11_ = p11 - k1*p21;
        p12_ = p12 - k1*p22;
        p21_ = p21 - k2*p21;
        p22_ = p22 - k2*p22;
        p11 = p11_;
        p12 = p12_;
        p21 = p21_;
        p22 = p22_;
        //printf("%11.4f %11.4f %11.4f %11.4f\n", p11, p12, p21, p22);

        //predict state
        velocity_ = velocity + accel*step;
        altitude_ = altitude + velocity*step;
        velocity = velocity_;
        altitude = altitude_; 

        //predict P
        p11_ = p11 + step*step*q11;
        p12_ = step*p11 + p12;
        p21_ = step*p11 + p21;
        p22_ = step*step*p11 + step*p21 + step*p12 + p22 + step*step*q22;
        p11 = p11_;
        p12 = p12_;
        p21 = p21_;
        p22 = p22_;
}




