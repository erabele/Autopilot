#!/usr/bin/env python

from __future__ import print_function
from pymavlink import mavutil
import multiprocessing, time
import numpy as np
import air

# define indices of xh for easier access.
x, y, z, vt, alpha, beta, phi, theta, psi, p, q, r = range(12)

# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d = range(10,16)

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1,7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7,13)
throttle, aileron, elevator, rudder, none, flaps = range(7,13)

# define indices of cmd for easier access.
psi_c, h_c = range(2)

def estimator_loop(y,xh,servo):
    # get sensors for read_sensor function call.
    adc,imu,baro,ubl = air.initialize_sensors()
    time.sleep(3)
    count=0
    #Sensor installation details
    Rba=np.array([[0,-1,0], [-1,0,0], [0,0,1]]) #acc frame to body (eg, imu to body)
    #Environmental parameters
    declination=+3.233*np.pi/180 #rad, mag declination is +3deg14' (eg, east) in Stillwater 
    pres_sl=1010		#millibars, sea level pressure for the day. Update me! 1mb is 100Pa
    rhoSL=1.225			#kg/m^2, sea level standard density
    g=9.8065 #m/s^2, gravity
    while True:
        initialEstTime = time.time()
        new_gps = air.read_sensor(y,adc,imu,baro,ubl) # updates values in y
	#=====ESTIMATOR CODE STARTS HERE==================================
        #PREDICT: Predict step here (use old xh to find new xh)--------------------------------------
	#eg, xhatkminus = F*xhatkminusoneplus + G*ukminusone
	
	#CORRECT: Measurement correction step here (depends on which sensors available)--------------
	#First compute sensor-specific estimates for imu/mag sensors
	#Raw sensor data needs to be rotated to stability axes
	acc=Rba.dot( np.array([y[ax],y[ay],y[az]]) )
	mag=Rba.dot( np.array([y[mag_x],y[mag_y],y[mag_z]]) )
	#---add a line here for gyro rotation

	#Accel based pitch/roll estimater (assumes no accel other than gravity)
	phi_a =np.arctan2(acc[1],np.sqrt(acc[0]**2+acc[2]**2))
	theta_a=np.arctan2(-acc[0],np.sqrt(acc[1]**2+acc[2]**2))

	#Gyro based pitch/roll estimator
	#--add a routine here integrating the gyros for phi_g, theta_g

	#Magnetic heading (psi_m)
	Rhb=np.array([[np.cos(theta_a), np.sin(theta_a)*np.sin(phi_a), np.sin(theta_a)*np.cos(phi_a)], [0, np.cos(phi_a), -np.sin(phi_a)], [-np.sin(theta_a), np.cos(theta_a)*np.sin(phi_a), np.cos(theta_a)*np.cos(phi_a)]]) #rotation from body to horizontal plane
	magh=Rhb.dot(mag) #mag vector in horizontal plane components
	psi_m = np.arctan2(magh[1], magh[0]) + declination

	#Pressure altitude
	h_b= -(y[pres_baro]-pres_sl)*100 /(rhoSL*g)  #*100  from mb to Pa

	#Handle GPS and then fuse all measurements
        if (new_gps):
            pass # use gps to find x_g, y_g, z_g, Vt_g, alpha, beta
	    #---then fuse with xhminus to find xhplus
        else:
            pass # all non-GPS estimates already found, just fuse with xhminus to find xhplus

        #OUTPUT: write estimated values to the xh array--------------------------------------
	xh[z] = -h_b
	xh[phi]=phi_a
	xh[theta]=theta_a
        xh[psi]=psi_m

	count=count+1
	if (count % 8000)==0:
	    print("Phi=%3.0f, Theta=%3.0f, Psi=%3.0f" %  (phi_a*180/np.pi, theta_a*180/np.pi, psi_m*180/np.pi))
	#======ESTIMATOR CODE STOPS HERE===================================
	#if (0.0125- (time.time()-initialEstTime) < 0): print( 1/(time.time()-initialEstTime) )
        time.sleep(max(0.0125-(time.time()-initialEstTime),0) )

def controller_loop(xh,servo,cmd):

    while True:
    	initial_time=time.time()
        # print("total milliseconds between controller_loop iterations: {}".format(initial_time-last_time))
        if (servo[mode_flag] == 1):
	    #======CONTROLLER CODE STARTS HERE===============================================
            # rewrite servo_out values to servo array based on their previous values and xh, cmd
            # if (servo[servo_1]<1.5): servo[servo_1] = 1.55
            # else: servo[servo_1] = 1.45
            # time.sleep(1)
            #Controller should assign values in range 1.25 to 1.75 to outputs;
            #WARNING, servo damage likely if values outside this range are assigned
            #Example: This is a manual passthrough function
            servo[throttle]=servo[rcin_0]
            servo[aileron]=servo[rcin_1]
            servo[elevator]=servo[rcin_2]
            servo[rudder]=servo[rcin_3]
            servo[servo_4]=servo[servo_4] #no servo; channel used for manual/auto switch
            servo[flaps]=servo[rcin_5]
	    #=======CONTROLLER CODE STOPS HERE ======================================
        time.sleep(max(0.0125-(time.time()-initial_time),0) )


if __name__ == "__main__":

    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255)

    # initialize arrays for sharing sensor data.
    y = multiprocessing.Array('d', np.zeros(26)) # imu, baro, gps, adc
    xh = multiprocessing.Array('d', np.zeros(12)) # position, orientation, rates
    servo = multiprocessing.Array('d', np.zeros(13)) # mode_flag, rcin, servo_out
    cmd = multiprocessing.Array('d', np.zeros(2)) # psi_c, h_c

    # start processes for interpreting sensor data and setting servo pwm.
    estimator_process = multiprocessing.Process(target=estimator_loop, args=(y,xh,servo))
    estimator_process.daemon = True
    estimator_process.start()
    controller_process = multiprocessing.Process(target=controller_loop, args=(xh,servo,cmd))
    controller_process.daemon = True
    controller_process.start()
    servo_process = multiprocessing.Process(target=air.servo_loop, args=(servo,))
    servo_process.daemon = True
    servo_process.start()
    time.sleep(3)
    # start process for telemetry after other processes have initialized.
    telemetry_process = multiprocessing.Process(target=air.telemetry_loop, args=(y,xh,servo,master))
    telemetry_process.daemon = True
    telemetry_process.start()

    print("\nsending heartbeats to {} at 1hz.".format('/dev/ttyAMA0'))
    # loop for sending heartbeats and receiving messages from gcs.
    while True:
        # send heartbeat message periodically
        master.mav.heartbeat_send(1, 0, 0, 0, 4, 0)
        # still haven't figured out how to get mode to show up in mission planner.
        # print('heartbeat sent.')
        time.sleep(0.5)
	#=====WAYPOINT TRACKER STARTS HERE======================
	#Simple waypoint tracker
        #
	#=====WAYPOINT TRACKER STOPS HERE=======================

    # handle incoming commands over telemetry
        # try:
        #     msg = master.recv_match().to_dict()
        #     if (not (msg['mavpackettype'] == 'RADIO' or msg['mavpackettype'] == 'RADIO_STATUS' or msg['mavpackettype'] == 'HEARTBEAT')):
        #         print(msg)
        #         if (msg['mavpackettype'] == 'COMMAND_LONG'):
        #             master.mav.command_ack_send(msg['command'],4)
        #             print("acknowledge sent.")
        # except:
        #     pass
