#include "alt_kalman.hpp"
//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>

Alt_kalman::Alt_kalman(){};

void Alt_kalman::update(float z_sens, float accel, float h)
{
        step = h;

        //x:estimate x_:predict

        //predict state
        velocity_ = velocity + accel*step;
        altitude_ = altitude + velocity*step;
        //velocity = velocity_;
        //altitude = altitude_; 

        //predict P
        p11_ = p11 + step*step*q;
        p12_ = step*p11 + p12;
        p21_ = step*p11 + p21;
        p22_ = step*step*p11 + step*p21 + step*p12 + p22;


        //update kalman gain
        float s = p22_ + R;
        k1 = p12_ / s;
        k2 = p22_ / s;

        //inovation
        float e = z_sens - altitude_; 

        //estimate state
        velocity = velocity_ + k1 * e;
        altitude = altitude_ + k2 * e;
        
        //Estimated state output
        Velocity = velocity;
        Altitude = altitude;
        //printf("%11.3f %11.4f %11.4f %11.4f %11.4f ", t+step, velocity, altitude, z_sens, true_v);

        //estimate P
        p11 = p11_ - k1 * p21_;
        p12 = p12_ - k1 * p22_;
        p21 = p21_ * (1 - k2);
        p22 = p22_ * (1 - k2);
        //printf("%11.4f %11.4f %11.4f %11.4f\n", p11, p12, p21, p22);
}




