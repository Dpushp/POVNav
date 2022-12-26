#ifndef _pid_IBVS_H
#define _pid_IBVS_H

#include <iostream>
#include <cmath>

class PID_IBVS
{
private:
    double _Kp;
	double _Kd;
	double _Ki;
	double _alpha;
	double _pre_error;
	double _integral;
public:
    PID_IBVS();
    PID_IBVS(double _alpha, double Kp, double Ki, double Kd);
    ~PID_IBVS();

    // Setters
    void set_alpha(double _alpha);
    void set_Kp(double Kp);
    void set_Ki(double Ki);
    void set_Kd(double Kd);

    // Utils
    double getControl(double error, double dt);

};

#endif // _pid_IBVS_H