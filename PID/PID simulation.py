import time
import threading
import socket
import sys  #for exit
import struct
import numpy as np
import binascii
import struct
import datetime
import matplotlib as mpl 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import csv

#packet structure:
#0 - time
#1,2,3-ax,ay,az (ft/sec2)
#4,5,6-p,q,r (all in rad/sec)
#7,8,9-u,v,w (fps)
#10,11,12-phi,theta,psi (all in rad)
#13,14,15-lat,lon,alt (alt-in mtrs)
#16,17,18,19-P,vc,vg,rho
#20 heading-true-rad

try:
    udpobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpobj.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcpobj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcpobj.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error:
    print ('Failed to create socket')
    sys.exit()
try:
	tcpobj.connect( ('127.0.0.1', 9894) )
	tcpobj.send("resume\n")
except:
	print ('tcp object error')
	sys.exit()
udpobj.bind( ('', 9891) )

_time=0
prev_time=0
sum = 0

pitch_error = pitch_integral = pitch_derivative = pitch_proportional = pitch_prev_error = pitch_PIDsum =0
roll_error = roll_integral = roll_derivative = roll_proportional = roll_prev_error = roll_PIDsum = 0
pitch_rate_error = pitch_rate_integral = pitch_rate_derivative = pitch_rate_proportional = pitch_rate_prev_error = pitch_rate_PIDsum = 0
roll_rate_error = roll_rate_integral = roll_rate_derivative = roll_rate_proportional = roll_rate_prev_error = roll_rate_PIDsum = 0
yaw_rate_error = yaw_rate_integral = yaw_rate_derivative = yaw_rate_proportional = yaw_rate_prev_error = yaw_rate_PIDsum = 0
thrust_error = thrust_integral = thrust_derivative = thrust_proportional = thrust_prev_error = thrust_PIDsum = 0

pitch_setpoint=0
roll_setpoint=0
yaw_rate_setpoint=0
thrust_setpoint=0    #altitude

while (1):
	rcv, addr = udpobj.recvfrom(1024)
	data=rcv.split(",")
	_time=float(data[0])
	rollv=float(data[4])
	pitchv=float(data[5])
	yawv=float(data[6])
	roll=float(data[10])*(180/np.pi)
	pitch=float(data[11])*(180/np.pi)
	yaw=float(data[13])*(180/np.pi)
	alt=float(data[15])
	vc=float(data[17])
	hdg=float(data[20])*(180/np.pi)
	
	dt = _time - prev_time
	
	#pitch
	pitch_error = pitch_setpoint - pitch
	pitch_proportional = 'kp'*pitch_error
	pitch_derivative = 'kd'*(pitch_error-pitch_prev_error)/(dt)
	pitch_integral = pitch_integral + 'ki'*pitch_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = pitch_proportional + pitch_integral + pitch_derivative
	if(sum<'PID_min'):
		sum='PID_min'
	elif(sum>'PID_max'):
		sum='PID_max'
	pitch_PIDsum=sum
	pitch_prev_error = pitch_error
	
	#roll
	roll_error = roll_setpoint - roll
	roll_proportional = 'kp'*roll_error
	roll_derivative = 'kd'*(roll_error-roll_prev_error)/(dt)
	roll_integral = roll_integral + 'ki'*roll_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = roll_proportional + roll_integral + roll_derivative
	if(sum<PID_min):
		sum=PID_min
	elif(sum>PID_max):
		sum=PID_max
	roll_PIDsum=sum
	roll_prev_error = roll_error
	
	#pitch_rate
	pitch_rate_error = pitch_PIDsum - pitchv
	pitch_rate_proportional = 'kp'*pitch_rate_error
	pitch_rate_derivative = 'kd'*(pitch_rate_error-pitch_rate_prev_error)/(dt)
	pitch_rate_integral = pitch_rate_integral + 'ki'*pitch_rate_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = pitch_rate_proportional + pitch_rate_integral + pitch_rate_derivative
	if(sum<PID_min):
		sum=PID_min
	elif(sum>PID_max):
		sum=PID_max
	pitch_rate_PIDsum=sum
	pitch_rate_prev_error = pitch_rate_error
	
	#roll_rate
	roll_rate_error = roll_PIDsum - rollv
	roll_rate_proportional = 'kp'*roll_rate_error
	roll_rate_derivative = 'kd'*(roll_rate_error-roll_rate_prev_error)/(dt)
	roll_rate_integral = roll_rate_integral + 'ki'*roll_rate_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = roll_rate_proportional + roll_rate_integral + roll_rate_derivative
	if(sum<PID_min):
		sum=PID_min
	elif(sum>PID_max):
		sum=PID_max
	roll_rate_PIDsum=sum
	roll_rate_prev_error = roll_rate_error
	
	#yaw_rate
	yaw_rate_error = yaw_rate_setpoint - yawv
	yaw_rate_proportional = 'kp'*yaw_rate_error
	yaw_rate_derivative = 'kd'*(yaw_rate_error-yaw_rate_prev_error)/(dt)
	yaw_rate_integral = yaw_rate_integral + 'ki'*yaw_rate_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = yaw_rate_proportional + yaw_rate_integral + yaw_rate_derivative
	if(sum<PID_min):
		sum=PID_min
	elif(sum>PID_max):
		sum=PID_max
	yaw_rate_PIDsum=sum
	yaw_rate_prev_error = yaw_rate_error

	#thrust
	thrust_error = thrust_setpoint - alt
	thrust_proportional = 'kp'*thrust_error
	thrust_derivative = 'kd'*(thrust_error-thrust_prev_error)/(dt)
	thrust_integral = thrust_integral + 'ki'*thrust_error*(dt)
	if(integral>'integral_max'):
		integral='integral_max'

	sum = thrust_proportional + thrust_integral + thrust_derivative
	if(sum<PID_min):
		sum=PID_min
	elif(sum>PID_max):
		sum=PID_max
	thrust_PIDsum=sum
	thrust_prev_error = thrust_error


  #the front and rear motors spin clockwise, and the left and right motors spin counter-clockwise.
  #order: F->B->L->R
	m1_ctrl = (sqrt(thrust.PID_sum + yaw_rate.PID_sum + pitch_rate.PID_sum))/1280	#F
	m2_ctrl = (sqrt(thrust.PID_sum + yaw_rate.PID_sum - pitch_rate.PID_sum))/1280	#B
	m3_ctrl = (sqrt(thrust.PID_sum - yaw_rate.PID_sum + roll_rate.PID_sum))/1280	#L
	m4_ctrl = (sqrt(thrust.PID_sum - yaw_rate.PID_sum - roll_rate.PID_sum))/1280	#R
	prev_t = t;
	print (alt,vc,hdg_err,thr,phi)
	#if sim_time>15 and sim_time<20:
	#	tcpobj.send("set atmosphere/psiw-rad %f\n" % 1.5708)
	#	tcpobj.send("set atmosphere/wind-mag-fps %f\n" % 10)

	#if sim_time>35 and sim_time<40:
	#	tcpobj.send("set atmosphere/psiw-rad %f\n" % 0)
	#	tcpobj.send("set atmosphere/wind-mag-fps %f\n" % 15)
	
	tcpobj.send("set fcs/throttle-cmd-norm[0] %f\n" % m1_ctrl)
	tcpobj.send("set fcs/throttle-cmd-norm[1] %f\n" % m2_ctrl)
	tcpobj.send("set fcs/throttle-cmd-norm[2] %f\n" % m3_ctrl)
	tcpobj.send("set fcs/throttle-cmd-norm[3] %f\n" % m4_ctrl)
tcpobj.send("hold\n")
udpobj.close()
tcpobj.close()
