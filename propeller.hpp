#ifndef PROPELLER_HPP
#define PROPELLER_HPP

class propeller
{
    private:
        double J_prop;
        double Ct;
        double Cq;
        
    public:
        propeller();
        double torque(double omega);
        double thrust(double omega);

};


#endif