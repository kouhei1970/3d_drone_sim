#ifndef MOTOR_HPP
#define MOTOR_HPP
#define MOTOR_STATE_NUM 1

class motor
{
    private:
        double J_mot;
        double J_load;
        double L_mot;
        double R_mot;
        double D_mot;
        double K_mot;
        double Fric_mot;
        
        void dxdt(double* output, double state, double dist, double u);
    public:
        motor();
        double Omega_mot;
        void update(double u, double dist, double h);
        void set_load_inartia(double j_load);
        double torque(double e);
        void print(double t);

};

#endif