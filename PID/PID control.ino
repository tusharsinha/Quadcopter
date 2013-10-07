

int PWM_motXp = 0; PWM_motXn = 0; PWM_motYp = 0; PWM_motYn = 0;
int pin_motXp = 1; pin_motXn = 2; pin_motYp = 3; pin_motYn = 4; //change when assembled

class PID
{
  public:
	float PIDsum(float* position, float* setpoint);  
	float error;
	float integral;
	float derivative;
	float propotional;
	float prev_error;
	float PID_sum;
	float kp;
	float ki;
	float kd;
	float integral_max;
	float PID_min;
	float PID_max;
	long unsigned int t;
	long unsigned int prev_t;
	
}

float PID::PIDsum(float* position, float* setpoint){
	error = setpoint - position;
	propotional = kp*error;
	t=millis();
	derivative = kd*(error-prev_error)/(t - prev_t);
	integral = integral + ki*error*(t - prev_t);
	integral = min(integral,integral_max);
	float sum = propotional + integral + derivative;
	PID_sum = max(min(sum,PID_max),PID_min);
	prev_error = error;
	prev_t = t;
	return PID_sum;
}


void setup(){
}


void loop (){
}

void controlquad(){

}