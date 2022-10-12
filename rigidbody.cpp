#include "rigidbody.hpp"
#include <stdio.h>
#include <math.h>

rigidbody::rigidbody(   double u, double v, double w,
                        double p, double q, double r,
                        double x, double y, double z,
                        double phi, double theta, double psi)
{
    Mass = 0.71;
    Ixx = 6.1e-3;
    Ixy = 0.0;
    Ixz = 0.0;
    Iyx = 0.0;
    Iyy = 6.53e-3;
    Iyz = 0.0;
    Izx = 0.0;
    Izy = 0.0;
    Izz = 1.16e-2;

    //Velocity
    Ub = u;
    Vb = v;
    Wb = w;
    Pb = p;
    Qb = q;
    Rb = r;

    //Angle
    Phi = phi;
    Theta = theta;
    Psi = psi;

    //DCM
    double e11 =  cos(Theta)*cos(Psi);
    double e12 =  cos(Theta)*sin(Psi);
    double e13 = -sin(Theta);
    double e21 =  sin(Phi)*sin(Theta)*cos(Psi) - cos(Phi)*sin(Psi);
    double e22 =  sin(Phi)*sin(Theta)*sin(Psi) + cos(Phi)*cos(Psi);
    double e23 =  sin(Phi)*cos(Theta);
    double e31 =  cos(Phi)*sin(Theta)*cos(Psi) + sin(Phi)*sin(Psi);
    double e32 =  cos(Phi)*sin(Theta)*sin(Psi) - sin(Phi)*cos(Psi);
    double e33 =  cos(Psi)*cos(Theta);

    //DCM to Quaternion
    Q0 = 0.5*sqrt(1 + e11 + e22 + e33);
    Q1 = 0.5*sqrt(1 + e11 - e22 - e33);
    Q2 = 0.5*sqrt(1 - e11 + e22 - e33);
    Q3 = 0.5*sqrt(1 - e11 - e22 + e33);

    if (Q0 > 0.0)
    {
        Q1 = (e23 - e32)/4/Q0;
        Q2 = (e13 - e31)/4/Q0;
        Q3 = (e12 - e21)/4/Q0;
    }
    else if (Q1 > 0.0)
    {
        Q0 = (e23 + e32)/4/Q1;
        Q2 = (e12 + e21)/4/Q1;
        Q3 = (e13 + e31)/4/Q1;
    }
    else if (Q2>0.0)
    {
        Q0 = (e31 - e13)/4/Q2;
        Q1 = (e12 + e21)/4/Q2;
        Q3 = (e23 + e32)/4/Q3;
    }
    else if (Q3>0.0)
    {
        Q0 = (e12 - e21)/4/Q3;
        Q1 = (e13 + e31)/4/Q3;
        Q2 = (e23 + e32)/4/Q3;
    }
    double mag = sqrt(Q0*Q0 + Q1*Q1 + Q2*Q2 + Q3*Q3);
    Q0 = Q0/mag;
    Q1 = Q1/mag;
    Q2 = Q2/mag;
    Q3 = Q3/mag;


    //Position
    Xe = x;
    Ye = y;
    Ze = z;
}

//Rigidbody Equation of motion
void rigidbody::dxdy(double* output, double* state, double* force, double* moment)
{
    //state
    double u = state[0];
    double v = state[1];
    double w = state[2];
    double p = state[3];
    double q = state[4];
    double r = state[5];
    double q0 = state[6];
    double q1 = state[7];
    double q2 = state[8];
    double q3 = state[9];
    double x =state[10];
    double y =state[11];
    double z =state[12];
    
    //force & moment
    double fx = force[0];
    double fy = force[1];
    double fz = force[2];
    double mx = moment[0];
    double my = moment[1];
    double mz = moment[2];

    //DCM
    double e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    double e12 = 2*(q1*q2 + q0*q3);
    double e13 = 2*(q1*q3 - q0*q2);
    double e21 = 2*(q1*q2 - q0*q3);
    double e22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    double e23 = 2*(q2*q3 + q0*q1);
    double e31 = 2*(q1*q3 + q0*q2);
    double e32 = 2*(q2*q3 - q0*q1);
    double e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    double u_dot = -q*w + r*v + fx/Mass;
    double v_dot = -r*u + p*w + fy/Mass;
    double w_dot = -p*v + q*u + fz/Mass;

    double a11 = Ixx;
    double a12 = Ixy;
    double a13 = Ixz;
    double a21 = Iyx;
    double a22 = Iyy;
    double a23 = Iyz;
    double a31 = Izx;
    double a32 = Izy;
    double a33 = Izz;
    double b1 = -Ixz*p*q - (Izz - Iyy)*q*r + Iyx*r*p - Izy*q*q + Iyz*r*r + mx;
    double b2 =  Izy*p*q - Ixy*q*r - (Ixx - Izz)*r*p + Izx*p*p - Ixz*r*r + my;
    double b3 = -(Iyy - Ixx)*p*q + Ixz*q*r - Iyz*r*p - Iyx*p*p + Ixy*q*q + mz;

    double det = (a11*a22*a33 - a11*a23*a32 - a12*a21*a33 + a12*a23*a31 + a13*a21*a32 - a13*a22*a31);
    double p_dot = (b1*( a22*a33 - a23*a32) + b2*(-a12*a33 + a13*a32) + b3*( a12*a23 - a13*a22))/det;
    double q_dot = (b1*(-a21*a33 + a23*a31) + b2*( a11*a33 - a13*a31) + b3*(-a11*a23 + a13*a21))/det;
    double r_dot = (b1*( a21*a32 - a22*a31) + b2*(-a11*a32 + a12*a31) + b3*( a11*a22 - a12*a21))/det;

    double q0_dot = 0.5*(-p*q1 - q*q2 + r*q3);
    double q1_dot = 0.5*( p*q0 + r*q2 - q*q3);
    double q2_dot = 0.5*( q*q0 - r*q1 + p*q3);
    double q3_dot = 0.5*( r*q0 + q*q1 - r*q2);

    double x_dot = e11*u + e12*v + e13*w;
    double y_dot = e21*u + e22*v + e23*w;
    double z_dot = e31*u + e32*v + e33*w;
    
    output[0] = u_dot;
    output[1] = v_dot;
    output[2] = w_dot;
    output[3] = p_dot;
    output[4] = q_dot;
    output[5] = r_dot;
    output[6] = q0_dot;
    output[7] = q1_dot;
    output[8] = q2_dot;
    output[9] = q3_dot;
    output[10] = x_dot;
    output[11] = y_dot;
    output[12] = z_dot;

}

//State update (rk4)
void rigidbody::update(double* force, double* moment, double h)
{
    double k1[STATE_NUM], k2[STATE_NUM], k3[STATE_NUM], k4[STATE_NUM];
    double state[STATE_NUM];
    double state2[STATE_NUM];

    //State
    state[0] = Ub;
    state[1] = Vb;
    state[2] = Wb;
    state[3] = Pb;
    state[4] = Qb;
    state[5] = Rb;
    state[6] = Q0;
    state[7] = Q1;
    state[8] = Q2;
    state[9] = Q3;
    state[10] = Xe;
    state[11] = Ye;
    state[12] = Ze;

    dxdy(k1, state, force, moment);
    for (int i=0; i<STATE_NUM; i++) state2[i]= state[i] + 0.5*h*k1[i];
    dxdy(k2, state2, force, moment);
    for (int i=0; i<STATE_NUM; i++) state2[i]= state[i] + 0.5*h*k2[i];
    dxdy(k3, state2, force, moment);
    for (int i=0; i<STATE_NUM; i++) state2[i]= state[i] + h*k3[i];
    dxdy(k4, state2, force, moment);

    for (int i=0; i<STATE_NUM; i++) state[i]= state[i] + h*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i])/6;

    Ub = state[0];
    Vb = state[1];
    Wb = state[2];
    Pb = state[3];
    Qb = state[4];
    Rb = state[5];
    Q0 = state[6];
    Q1 = state[7];
    Q2 = state[8];
    Q3 = state[9];
    Xe = state[10];
    Ye = state[11];
    Ze = state[12];

    double mag = sqrt(Q0*Q0 + Q1*Q1 + Q2*Q2 + Q3*Q3);
    Q0 = Q0/mag;
    Q1 = Q1/mag;
    Q2 = Q2/mag;
    Q3 = Q3/mag;

    //DCM
    double dcm[9];
    double quat[4]={Q0, Q1, Q2, Q3};
    quat2dcm(dcm, quat);

    Phi = atan2(dcm[5], dcm[8]);
    Theta = atan2(-dcm[2], sqrt(dcm[5]*dcm[5] + dcm[8]*dcm[8]));
    Psi = atan2(dcm[1], dcm[0]);

}


void rigidbody::print(double t)
{
    printf( "%9.6f "
            "%9.6f %9.6f %9.6f "
            "%9.6f %9.6f %9.6f "
            "%9.6f %9.6f %9.6f %9.6f "
            "%9.6f %9.6f %9.6f "
            "%9.6f %9.6f %9.6f\n",
        t, 
        Ub, Vb, Wb,
        Pb, Qb, Rb,
        Q0, Q1, Q2, Q3,
        Phi, Theta, Psi,
        Xe, Ye, Ze);
}

void quat2dcm(double* dcm, double* quat)
{
    double q0 = quat[0];
    double q1 = quat[1];
    double q2 = quat[2];
    double q3 = quat[3];

    //DCM
    dcm[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    dcm[1] = 2*(q1*q2 + q0*q3);
    dcm[2] = 2*(q1*q3 - q0*q2);
    dcm[3] = 2*(q1*q2 - q0*q3);
    dcm[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    dcm[5] = 2*(q2*q3 + q0*q1);
    dcm[6] = 2*(q1*q3 + q0*q2);
    dcm[7] = 2*(q2*q3 - q0*q1);
    dcm[8] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}
