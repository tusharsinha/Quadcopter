#include math.h

float accel_final[3];
float gyro_final[3];
float magn_final[3];
float zenith_accel[3];
float north_magn[3];
float zenith_gyro[3];
float zenith_gyro_prev[3];
float north_gyro[3];
float north_gyro_prev[3];
float north_avg[3];
float zenith[3];
float north[3];
float west[3];
unsigned long int time_gyroPoll;
unsigned long int time_prev_gyroPoll;
float dt;  //int?
float roll_angle; // final op
float pitch_angle; // final
float thrust; // final
const float compFilt_weight; // set weight
const float gravity = 9.8;

void accel_to_zenith(float a[3])
{
	zenith_accel[0] = a[0];
	zenith_accel[1] = a[1];
	zenith_accel[2] = sqrt(gravity*gravity - (zenith_accel[0]*zenith_accel[0] + zenith_accel[1]*zenith_accel[1]);
}

void magn_to_north(float m[3])
{
	north_magn[0] = m[0];
	north_magn[1] = m[1];
	north_magn[2] = m[2];
}

void gyro_direction(float g[3])
{
	time_gyroPoll = millis();
	dt = time_gyroPoll - time_prev_gyroPoll;
	time_prev_gyroPoll = time_gyroPoll;
	
	zenith_gyro_prev[0] = zenith_gyro[0]; 
	zenith_gyro_prev[1] = zenith_gyro[1];
	zenith_gyro_prev[2] = zenith_gyro[2];
	zenith_gyro[0] = zenith_gyro_prev[0] + dt*(g[1]*zenith_gyro_prev[2] - g[2]*zenith_gyro_prev[1]);
	zenith_gyro[1] = zenith_gyro_prev[1] + dt*(g[2]*zenith_gyro_prev[0] - g[0]*zenith_gyro_prev[2]);
	zenith_gyro[2] = zenith_gyro_prev[2] + dt*(g[0]*zenith_gyro_prev[1] - g[1]*zenith_gyro_prev[0]);
	
	north_gyro_prev[0] = north_gyro[0]; 
	north_gyro_prev[1] = north_gyro[1];
	north_gyro_prev[2] = north_gyro[2];
	north_gyro[0] = north_gyro_prev[0] + dt*(g[1]*north_gyro_prev[2] - g[2]*north_gyro_prev[1]);
	north_gyro[1] = north_gyro_prev[1] + dt*(g[2]*north_gyro_prev[0] - g[0]*north_gyro_prev[2]);
	north_gyro[2] = north_gyro_prev[2] + dt*(g[0]*north_gyro_prev[1] - g[1]*north_gyro_prev[0]);
}

void comp_filter()
{ 
	zenith[0] = compFilt_weight*zenith_accel[0] + (1 - compFilt_weight)*zenith_gyro[0];
	zenith[1] = compFilt_weight*zenith_accel[1] + (1 - compFilt_weight)*zenith_gyro[1];
	zenith[2] = compFilt_weight*zenith_accel[2] + (1 - compFilt_weight)*zenith_gyro[2];
	
	north_avg[0] = compFilt_weight*north_accel[0] + (1 - compFilt_weight)*north_gyro[0];
    north_avg[1] = compFilt_weight*north_accel[1] + (1 - compFilt_weight)*north_gyro[1];
    north_avg[2] = compFilt_weight*north_accel[2] + (1 - compFilt_weight)*north_gyro[2];
}

void get_West()
{
	west[0] = zenith[1]*north_avg[2] - zenith[2]*north_avg[1];
	west[1] = zenith[2]*north_avg[0] - zenith[0]*north_avg[2];
	west[2] = zenith[0]*north_avg[1] - zenith[1]*north_avg[0];
}

void normalize(v[3])
{
	float v_magnitude = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] = v[0]/v_magnitude;
	v[1] = v[1]/v_magnitude;
	v[2] = v[2]/v_magnitude;
}

void correct_north()
{
	north[0] = west[1]*zenith[2] - zenith[1]*west[2];
	north[1] = west[2]*zenith[0] - zenith[2]*west[0];
	north[2] = west[0]*zenith[1] - zenith[0]*west[1];
}

void get_Pitch()
{
	pitch_angle = atan(north[2]/zenith[2]);
}


void get_Roll()
{
	roll_angle = atan(west[2]/zentih[2]);
}
void get_Thrust()
{
		thrust = a[2] - zenith_accel[2];
}

void main(float a[3], float g[3], float m[3]) //input parameters accel data, gyro data, magno data
{
	accel_to_zenith(a);
	magn_to_north(m);
	gyro_direction(g);
	comp_filter();
	get_West();
	correct_north();
	normalize(zenith);
	normalize(west);
	normalize(north);
	get_Pitch // final op stored in global variables
	get_Roll(); // final op global variable
	get_Thrust(); //final op global var
}
