#include <numeric>
#include "PID.h"
#include "twiddler.h"
using namespace std;

#define TWIDDLE_INCREASE_FACTOR 1.1
#define TWIDDLE_DECREASE_FACTOR 0.9

Twiddler::Twiddler() {}

Twiddler::~Twiddler() {}

void Twiddler::Init(double tol){
	this->best_err = 9999999.99;
	for (int i=0; i<3; i++){
		this->dp[i]=0.01;
	}

	//reduce i delta to something very very low.
	dp[1]=0.00001;


	settle_iterations=100; 
	settle_completion=100;
	twiddle_iterations=700;
	twiddle_completion=100;
	tolerance = tol;
	first_pass=true;

	this->current_tuning_phase=p_tuning;
	tried_subtraction = false;
	updating_phase = true;
	err=0;
	iteration = settle_iterations;
}


void Twiddler::twiddle(PID* pid, double cte){
	if(this->iteration <= settle_iterations){
		if((int)100*iteration/settle_iterations % 25 == 0){
			if(settle_completion != (int)100*iteration/settle_iterations){
				cout << "settle phase : "<< (int)100*iteration/settle_iterations<<(char)37<< endl;
			}
			settle_completion= (int)100*iteration/settle_iterations;
		}
		this->iteration++;
		return; //do nothing

	} else if(this->iteration <= (settle_iterations+twiddle_iterations)){

		if((int)100*(iteration-settle_iterations)/twiddle_iterations % 25 == 0){
			if(twiddle_completion != (int)100*(iteration-settle_iterations)/twiddle_iterations){
				cout << "error calculation phase : "<< 100*(iteration-settle_iterations)/twiddle_iterations <<(char)37<< endl;
			}
			twiddle_completion = (int)100*(iteration-settle_iterations)/twiddle_iterations;
		}
		/* calculate error */
		err += cte*cte;
		this->iteration++;
		return; //stop execution of function here, move to next iteration
	}

	double sum = 0;
	for (int i=0; i<3; i++){
		sum+=this->dp[i];
	}
	if (sum>this->tolerance){  //having a while loop is not right... it needs to be an if ... rethink this part
	//implement here

		if (updating_phase){

			if(first_pass){
				best_err = err; 
				cout<<"initialized error to: "<<err<<endl;
				first_pass=false;
			}

			cout << "total iterations: "<< iteration << endl;
			cout << "increasing ";
			switch(current_tuning_phase){
					case p_tuning:
						cout<<"Kp";
						break;
					case i_tuning:
						cout<<"Ki";
						break;
					case d_tuning:
						cout<<"Kd";
						break;
				}
			cout<<" by "<<dp[current_tuning_phase]<<endl;
			pid->params[current_tuning_phase] +=  dp[current_tuning_phase];
			cout<<"Current parameters:\n";
			cout<<"Kp="<<pid->params[0]<<", "<<"Ki="<<pid->params[1]<<", "<<"Kd="<<pid->params[2]<<endl;

			this->iteration=0; //reset iterations, allowing for recalculation of error
			this->err=0;
			updating_phase=false; 
			return;
		} else {

			if ( err < best_err ) {
				cout << "Error ("<<err<<") is lower than best error ("<<best_err<<")"<<endl;
				cout << "let's try increasing dp by "<<100*(TWIDDLE_INCREASE_FACTOR-1)<<(char)37<<" for ";
				switch(current_tuning_phase){
					case p_tuning:
						cout<<"Kp"<<endl;
						break;
					case i_tuning:
						cout<<"Ki"<<endl;
						break;
					case d_tuning:
						cout<<"Kd"<<endl;
						break;
				}
				cout<<"increased from: "<<dp[current_tuning_phase]<<" to ";

				best_err = err; 
				dp[current_tuning_phase]*= TWIDDLE_INCREASE_FACTOR;
				cout<<dp[current_tuning_phase]<<endl;

				/* then switch to next parameter */
				current_tuning_phase = static_cast<twiddle_phase>((current_tuning_phase + 1) % 3);
				tried_subtraction=false; //reset the tried_subtraction flag state
				updating_phase=true; //next iteration will increase the parameter value by dp
				//this->iteration=0; //reset iterations so the error will be evaluated
				return;

			} else if (!tried_subtraction){
				cout << "Error ("<<err<<") is not lower than best error ("<<best_err<<")"<<endl;
				cout << "let's try subtracting "<<dp[current_tuning_phase]<<" from ";
				switch(current_tuning_phase){
					case p_tuning:
						cout<<"Kp"<<endl;
						break;
					case i_tuning:
						cout<<"Ki"<<endl;
						break;
					case d_tuning:
						cout<<"Kd"<<endl;
						break;
				}

				pid->params[current_tuning_phase] -=  2*dp[current_tuning_phase];  //try subtracting dp
				cout<<"Current parameters:\n";
				cout<<"Kp="<<pid->params[0]<<", "<<"Ki="<<pid->params[1]<<", "<<"Kd="<<pid->params[2]<<endl;
				
				cout<<endl;
				this->iteration=0; //reset iterations so the error will be evaluated
				this->err=0;
				tried_subtraction = true; //next failure to decrease error will make parameter increments (dp) decrease
				return;

			} else {
				cout << "Error ("<<err<<") is not lower than best error ("<<best_err<<")"<<endl;
				cout << "let's try lowering dp by "<<100*(1-TWIDDLE_DECREASE_FACTOR)<<(char)37<<" for ";
				switch(current_tuning_phase){
					case p_tuning:
						cout<<"Kp"<<endl;
						break;
					case i_tuning:
						cout<<"Ki"<<endl;
						break;
					case d_tuning:
						cout<<"Kd"<<endl;
						break;
				}
				cout<<"lowered from: "<<dp[current_tuning_phase]<<" to ";

				pid->params[current_tuning_phase] += dp[current_tuning_phase];
				dp[current_tuning_phase] *= TWIDDLE_DECREASE_FACTOR; //decrease dp
				cout<<dp[current_tuning_phase]<<endl;

				/* then switch to next parameter */
				current_tuning_phase = static_cast<twiddle_phase>((current_tuning_phase + 1) % 3);

				updating_phase=true; //we go back to the begining (adding dp to the parameter)
				//this->iteration=0; //reset iterations so the error will be evaluated
				return;
			}
		}

	} else {
		cout<<"Twiddler fully twiddled. Sum of dp: " <<sum<< endl;
	}
}
