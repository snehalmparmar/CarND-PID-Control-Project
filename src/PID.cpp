#include "PID.h"
#include "twiddler.h"

using namespace std;

PID::PID() { this->is_initialized = false; }

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	//need to tune the pid here... use twiddle 
	//double[] params = {Kp, Ki, Kd}; 
	this->p_error = this->d_error = this->i_error = 0.0;
	this->int_cte=0;
	this->is_initialized=true;
	this->twiddle_tolerance = 0.002;
	this->twiddler.Init(twiddle_tolerance);
	for(int i=0; i<3; i++){
		switch(i) {
			case 0: params[i] = Kp;
					break;
			case 1: params[i] = Ki;
					break;
			case 2: params[i] = Kd;
					break;
		}
	}

	p_error=0;
	i_error=0;
	d_error=0;
	is_initialized=true;
}

void PID::UpdateError(double cte) {

	this->twiddler.twiddle(this,cte);
	d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
	return (-this->params[0]*p_error) + (-this->params[1]*i_error) + (-this-> params[2]*d_error);
}

double PID::GetSteering(){
	return TotalError();
}