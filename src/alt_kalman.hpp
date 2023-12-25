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
    float h1=0.0, h2=1.0;

    //P
    float p11=100.0, p12=100.0, p21=100.0, p22=100.0;
    float p11_, p12_, p21_, p22_;

    //Q
    float q1=(0.04)*(0.04), q2=(0.001)*(0.001);//q1=0.04*0.04 q2=0.001*0.001

    //R
    float R = 0.05*0.05;

    public:
    //setep
    float step=1.0/30.0;
    //state
    float Velocity=0.0, Altitude=0.0;
    Alt_kalman();
    void update(float z_sens, float accel);

};

#endif