#ifndef MOTOR_HPP
#define MOTOR_HPP

class motor
{
    private:
        double J_mot;
        double L_mot;
        double R_mot;
        double D_mot;
        double K_mot;
        double Fric_mot;
        void dxdt(double* output, double* state, double dist, double u);
    public:
        motor();
        void update(void);

};

#endif