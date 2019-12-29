#ifndef TWIDDLER_H_
#define TWIDDLER_H_ 

#include "PID.h"
class PID;  //just a forward declaration for later use
class Twiddler{
public: 
	//extern const double TWIDDLE_INCREASE_FACTOR; 
	//extern const double TWIDDLE_DECREASE_FACTOR; 

	double best_err;
	double dp[3];
	int settle_iterations; //number of iterations allowed to settle (before calculating error)
	int twiddle_iterations; //number of total iteration for calculating error
	double tolerance;//parameter: tolerance of the twiddler
	double err;
	int settle_completion;
	int twiddle_completion;
	bool first_pass;

	enum twiddle_phase 
	{	p_tuning, 
		i_tuning, 
		d_tuning
	} current_tuning_phase; 

	bool tried_subtraction;
	bool updating_phase;
	int iteration;

	Twiddler();

	virtual ~Twiddler();
  
	void Init(double tol);

	bool refreshError(double err);

	void twiddle(PID* pid, double cte);
};

#endif /* TWIDDLER_H_ */