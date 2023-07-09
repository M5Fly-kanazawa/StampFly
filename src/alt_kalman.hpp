#ifndef ALT_KALMAN_HPP
#define ALT_KALMAN_HPP

class Alt_kalman
{
    //accel
    double accel=0.0;

    //state
    double velocity=0.0, altitude=0.0;
    double velocity_, altitude_;
    double true_v;

    //Sensor
    double z_sens;

    //Kalman Gain
    double k1,k2;

    //F
    double f11=1.0, f12=0.0, f21=step, f22=1.0;

    //H
    double h1=0.0, h2=1.0;

    
    //P
    double p11=10.0, p12=10.0, p21=10.0, p22=10.0;
    double p11_, p12_, p21_, p22_;

    //Q
    double q11=1.0, q12=0.0, q21=0.0, q22 =1.0;

    //R
    double R = 0.1;

    public:
    //setep
    double step=0.05;
    //state
    double Velocity=0.0, Altitude=0.0;
    Alt_kalman();
    void update(double z_sens, double accel);

};

#endif