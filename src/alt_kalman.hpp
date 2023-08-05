#ifndef ALT_KALMAN_HPP
#define ALT_KALMAN_HPP

class Alt_kalman
{
    //accel
    float accel=0.0;

    //state
    float velocity=0.0, altitude=0.0;
    float velocity_, altitude_;
    float true_v;

    //Sensor
    float z_sens;

    //Kalman Gain
    float k1,k2;

    //F
    float f11=1.0, f12=0.0, f21=step, f22=1.0;

    //H
    double h1=0.0, h2=1.0;

    //P
    double p11=1.0, p12=1.0, p21=1.0, p22=1.0;
    double p11_, p12_, p21_, p22_;

    //Q
    double q1=(1.0e-3)*(1.0e-3), q2=(1.0e-3)*(1.0e-3);

    //R
    double R = 0.02*0.02;

    public:
    //setep
    double step=1/30;
    //state
    double Velocity=0.0, Altitude=0.0;
    Alt_kalman();
    void update(double z_sens, double accel);

};

#endif