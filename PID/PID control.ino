
int RPM_motXp = 0;
int RPM_motXn = 0;
int RPM_motYp = 0;
int RPM_motYn = 0;
int PWM_motXp = 0;
int PWM_motXn = 0;
int PWM_motYp = 0;
int PWM_motYn = 0;
int pin_motXp = 1;
int pin_motXn = 2;
int pin_motYp = 3;
int pin_motYn = 4; //change when assembled

class PID
{
  public:
    PID (float _kp,float _ki,float _kd,float _integral_max,float _PID_max,float _PID_min);
	void PIDsum(float* input, float* setpoint);  
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
	
};

PID::PID (float _kp,float _ki,float _kd,float _integral_max,float _PID_max,float _PID_min){
	kp = _kp;
	ki = _ki;
	kd = _kd;
	integral_max = _integral_max;
	PID_max = _PID_max;
	PID_min = _PID_min;
}

void PID::PIDsum(float* input, float* setpoint){
	error = setpoint - input;
	propotional = kp*error;
	t=millis();
	derivative = kd*(error-prev_error)/(t - prev_t);
	integral = integral + ki*error*(t - prev_t);
	integral = min(integral,integral_max);
	float sum = propotional + integral + derivative;
	PID_sum = max(min(sum,PID_max),PID_min);
	prev_error = error;
	prev_t = t;
}

//defining 6 PID classes
int _kp, _ki, _kd, _integral_max, _PID_max, _PID_min;

PID pitch (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);//set values
PID roll (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);
PID pitch_rate (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);
PID roll_rate (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);
PID yaw_rate (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);
PID thrust (_kp, _ki, _kd, _integral_max, _PID_max, _PID_min);

//defining Inertial parameters
float IXX = 10.0;
float IYY = 10.0;
float IZZ = 10.0;

float drag_coeff = 1.0;
float thrust_coeff = 1.0;
float length = 1.0;

float pitch_setpoint = 0;
float roll_setpoint = 0;
float yaw_setpoint = 0;
float thrust_setpoint = 0;

float pitch_filter = 0;
float roll_filter = 0;
float gyro_pitch =0;
float gyro_roll = 0;
float gyro_yaw = 0;
float thrust_input = 0;

void setup(){
}


void loop (){
}

void controlquad(){
	/*
	getangles();
	getsetpoints();
	*/
	pitch.PIDsum(&pitch_filter,&pitch_setpoint);
	roll.PIDsum(&roll_filter,&roll_setpoint);
	pitch_rate.PIDsum(&gyro_pitch,&pitch.PID_sum);
	roll_rate.PIDsum(&gyro_roll,&roll.PID_sum);
	yaw_rate.PIDsum(&gyro_yaw,&yaw_setpoint);
	thrust.PIDsum(&thrust_input,&thrust_setpoint);
	RPM_motXp = sqrt(thrust.PID_sum + yaw_rate.PID_sum + pitch_rate.PID_sum);	
	RPM_motXn = sqrt(thrust.PID_sum + yaw_rate.PID_sum - pitch_rate.PID_sum);
	RPM_motYp = sqrt(thrust.PID_sum - yaw_rate.PID_sum + roll_rate.PID_sum);
	RPM_motYn = sqrt(thrust.PID_sum - yaw_rate.PID_sum - roll_rate.PID_sum);
	/*
	lookup table to relate each RPM to PWM
	or
	PWM=RPM/1240;
	*/
	analogWrite(pin_motXp,PWM_motXp);
	analogWrite(pin_motXn,PWM_motXn);
	analogWrite(pin_motYp,PWM_motYp);
	analogWrite(pin_motYn,PWM_motYn);
}