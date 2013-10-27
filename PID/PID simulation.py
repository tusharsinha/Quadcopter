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
    print 'Failed to create socket'
    sys.exit()
try:
	tcpobj.connect( ('127.0.0.1', 9894) )
	tcpobj.send("resume\n")
except:
	print 'tcp object error'
	sys.exit()
udpobj.bind( ('', 9891) )

sim_time=0
while (1):
	rcv, addr = udpobj.recvfrom(1024)
	data=rcv.split(",")
	sim_time=float(data[0])
	roll=float(data[10])*(180/np.pi)
	pitch=float(data[11])*(180/np.pi)
	yaw=float(data[13])*(180/np.pi)
	alt=float(data[15])
	vc=float(data[17])
	hdg=float(data[20])*(180/np.pi)
	hdg_sp = 45.0
	roll_sp = 0.0
	pitch_sp = 0.0
	alt_sp = 1200
	
	thr = -0.015*(alt-1200)

	if (abs(thr) > 0.98):
		thr = (thr/abs(thr))*0.95

	if (thr<0):
		thr = 0.1

	m1_P=m2_P=m3_P=m4_P=0
	m1_R=m2_R=m3_R=m4_R=0
	m1_Y=m2_Y=m3_Y=m4_Y=0

	hdg_err=hdg-hdg_sp
	#if(hdg_err<0):
	#	m1_Y=0.01
	#	m3_Y=0.01
	#if(hdg_err>0):
	#	m2_Y=0.01
	#	m4_Y=0.01
	roll_err = phi - roll_sp
	m3_R=0.01*(roll_err)
	if(m3_R<0):
		m3_R=0
	elif(m3_R>1):
		m3_R=1
	#m1_R=1-m1_R

	pitch_err = theta - pitch_sp
	m4_P=0.01*(pitch_err)
	if(m4_P<0):
		m4_P=0
	elif(m4_P>1):
		m4_P=1
	#m2_P=1-m2_P

	yaw_err = psi - hdg_sp
	

  #the front and rear motors spin clockwise, and the left and right motors spin counter-clockwise.
  #order: F->B->L->R
	m1_ctrl = thr+m1_P+m1_R+m1_Y	#F
	m2_ctrl = thr+m2_P+m2_R+m2_Y	#B
	m3_ctrl = thr+m3_P+m3_R+m3_Y	#L
	m4_ctrl = thr+m4_P+m4_R+m4_Y	#R
	
	print alt,vc,hdg_err,thr,phi
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
