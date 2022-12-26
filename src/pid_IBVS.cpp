#include <pid_IBVS.h>

// Implementation of above defined class definition
PID_IBVS::PID_IBVS()
{

}

PID_IBVS::PID_IBVS(double _alpha, double Kp, double Ki, double Kd)
{
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;
	this->_alpha = _alpha;
	this->_pre_error = 0;
	this->_integral = 0;
}

PID_IBVS::~PID_IBVS()
{
}

/*! Setters */
void PID_IBVS::set_alpha(double _alpha){
    this->_alpha = _alpha;
}
void PID_IBVS::set_Kp(double Kp){
    this->_Kp = Kp;
}
void PID_IBVS::set_Ki(double Ki){
    this->_Ki = Ki;
}
void PID_IBVS::set_Kd(double Kd){
    this->_Kd = Kd;
}

double PID_IBVS::getControl(double error, double _dt){
    double _intrgral_MAX = 2.0;

    /*! Discard the first value. */
    // Calculate rate of change of error with low pass filter 
    double e_dot = _alpha*(_pre_error/_dt) + (1 - _alpha)*((error - _pre_error)/_dt);

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;
    // Limit integral value
    if(Iout > _intrgral_MAX)
        Iout = _intrgral_MAX;

    // Derivative term
    double Dout = _Kd * e_dot;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Save error to previous error
    _pre_error = error;

    // std::cout << "dt: " << _dt << std::endl;
    return output;
}
