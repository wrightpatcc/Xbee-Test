# Xbee-Test


####################################################
##Commander's Challenge Program for Drone Automation
##Starts Drone
##		Flies to multiple waypoints
##		Returns to starting location, lands/shutdown - Press "L" to land, "P" to acquire another target
#####################################

#####################################
## Imports 
from __future__ import division
import math
import datetime

import dronekit_sitl
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import serial
import RPi.GPIO as gpio


#####################################
## Global Variables and initial values
global x, y, z, key, vel, accel, correction_x, correction_y, correction_z, trkx, trky
global distance, elat, elon, ealt, submode, distance, edist, between, safe
key = 76
trkx = 7
trky = 7
correction_x = 0
correction_y = 0
correction_z = 0
accel = .9
s_lat = .00001
s_lon = .00001
b_lat = .00001
b_lon = .00001
f_lat = .00001
f_lon = .00001
safe = 4
dist1 = 1
between1 = 1
meh = 3
loft = 1
TIMEOUT = 3
find = None 

#need to uncomment GPIO

#GPIO Setup
LEDPin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LEDPin, GPIO.OUT)
pwm = GPIO.PWM(LEDPin, 50)

##########################################
##########Boots Simulator#################

#print "Start simulator (SITL)"
#sitl = dronekit_sitl.start_default()
#connection_string = sitl.connection_string()
#print("Connecting to vehicle on: %s" % (connection_string,))
#vehicle = connect(connection_string, wait_ready=True)

##########################################

#############################################
##########Start up code for real test########
##Connect to XBee
#ser = serial.Serial('com7', 9600, timeout = 0.5)
ser = serial.Serial('/dev/ttyUSB1', 9600, timeout = 0.5)
#Connected via RPi
vehicle = connect("/dev/ttyUSB0", wait_ready=True, baud = 57600) 
ser.write("Connecting...\n")

#############################################
##########Connect through Cord###############
#vehicle = connect('com5', wait_ready=True, baud = 57600) 
time.sleep(2)
#print "Autopilot Firmware version: %s" % vehicle.version
ser.write("Autopilot Firmware version: %s\n" % vehicle.version)
#############################################

## Functions 

#Arms and rises to given Altitude
def arm_and_takeoff(aTargetAltitude):
	"""
	Arms vehicle and fly to aTargetAltitude.
	"""
	while True:
		if not x == None and not y == None and not z == None:
		####################################################				
			print "Arming motors"
			ser.write("Starting motors\n")
			# Copter should arm in GUIDED mode
			vehicle.mode    = VehicleMode("GUIDED")
			vehicle.armed   = True
			# Confirm vehicle armed before attempting to take off
			while not vehicle.armed:
				print "Waiting for arming..."
				ser.write("Waiting for arming...\n")
				time.sleep(1)

			print "Taking off!"
			ser.write("Taking off!\n")
			vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

			# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
			#  after Vehicle.simple_takeoff will execute immediately).
			while True:
				print " Altitude: ", vehicle.location.global_relative_frame.alt
				ser.write("Altitude: %s\n" % vehicle.location.global_relative_frame.alt)
				#Break and return from function just below target altitude.
				if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
					print "Reached target altitude"
					ser.write("Reached target altitude\n")
					break
				time.sleep(1)
				
			break
		else:
			ser.write("False Coords Received\n")
			time.sleep(2)
			
			[Name, x, y, z] = rec_full_data("WP")

			ser.write("Lat:%s, Lon:%s, Alt:%s\n" % (x, y,z))
			[Name, elat, elon, ealt] = rec_full_data("EnemyWP")
			
			ser.write("ELat:%s, ELon:%s, EAlt:%s\n" % (elat, elon, ealt))
			

#Needed only for conditionyaw action
def send_global_velocity(velocity_x,velocity_y,velocity_z):
	msg = vehicle.message_factory.set_position_target_global_int_encode(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 0b0000111111000111,0,0,0,velocity_x,velocity_y,velocity_z,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

#Turns drone on z access
def conditionyaw(heading, relative = False):
	is_relative = 0
	# ^ states that heading is not relative to flight path
	send_global_velocity(0,0,0)
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,heading,0,1,is_relative, 0,0,0)
	vehicle.send_mavlink(msg)

#Flies drone to a given set of coords and alt
def gotoGPS(location):
	global x
	global y	
	global z
	global key
	global correction_x
	global correction_y
	global correction_z
	vehicle.simple_goto(location, airspeed=None, groundspeed=None)

#Distance to waypoint
def distance():
	#global x, y, z
	dist1 = 100
	dist1 = math.sqrt(((x*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*y)-((100000*vehicle.location.global_relative_frame.lon)))**2+(z-(vehicle.location.global_relative_frame.alt))**2)
	return dist1
	
#Distance to enemy
def between():
	global elat, elon, ealt, between
	between1 = 100
	between1 = math.sqrt(((elat*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*elon)-((100000*vehicle.location.global_relative_frame.lon)))**2+(ealt-(vehicle.location.global_relative_frame.alt))**2)
	return between1
	
	#Distance from home location	
def homedist():
	global homedist
	home1 = 100
	home1 = math.sqrt(((home_lat*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*home_lon)-((100000*vehicle.location.global_relative_frame.lon)))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2)
	return home1
	
#Sends vehicle home
def gotoHome(location):  ###Note: Groundspeed must be set via missionplanner
	print "I'm coming home!"
	ser.write("I'm coming home now\n")
	time.sleep(2)
	vehicle.simple_goto(location)
	while True:
		
		print " Location: Lat:%s, Lon:%s, Alt: %s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		print " Distance to Waypoint: %s" % (math.sqrt(((home_lat*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*home_lon)-((100000*vehicle.location.global_relative_frame.lon)))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2))
		ser.write("Location: Lat:%s, Lon:%s, Alt: %s\n" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt))
		ser.write("Distance to Waypoint: %s\n" % (math.sqrt(((home_lat*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*home_lon)-((100000*vehicle.location.global_relative_frame.lon)))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2)))
		
		#if vehicle.location.global_relative_frame.alt>= z*0.95 and vehicle.location.global_relative_frame.lat>= x*0.80 and vehicle.location.global_relative_frame.lon>= y*0.80:
		if math.sqrt(((100000*home_lat)-(100000*vehicle.location.global_relative_frame.lat))**2+((100000*home_lon)-(100000*vehicle.location.global_relative_frame.lon))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2)<=1:
		#if math.sqrt((x-(d*(-1)))**2+(y-(e*(-1)))**2+(z-(f*-(1))**2))<=2:
			
			print "The Drone is Back."
			ser.write("Drone is Back\n")
			
			break
		time.sleep(1)

#Returns vehicle to ground		
def RTL(landingAlt):
    print "Landing"
	ser.write("Landing\n")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt>=landingAlt:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
    	ser.write("Altitude: %s\n" % vehicle.location.global_relative_frame.alt)
		time.sleep(1)		
		
#Attempt to control drone directly from visual imputs
def tracking(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

#Setting for drone to fly based on heading 

#Glide is the base code for shifting the drone left and right...further comments can be found where "keys" "A" and "D" are utilized 
def strafe():
	global s_lat, s_lon, b_lat, b_lon, f_lat, f_lon
	#@ heading = 0, drone goes East
	s_lat =  math.sin(math.radians(vehicle.heading))
	s_lon =  math.cos(math.radians(vehicle.heading))
		
	f_lat = math.cos(math.radians(vehicle.heading))
	f_lon = math.sin(math.radians(vehicle.heading))
	
	b_lat = -math.cos(math.radians(vehicle.heading))
	b_lon = -math.sin(math.radians(vehicle.heading))
	

			
#How to send data thru XBee
def send_full_data(Name, arg1, arg2, arg3):
	while True:
		ser.write("%s\n" % Name)
		print Name
		time.sleep(.5)
		incoming = ser.readline().strip()
		if incoming == "Go":
			print "writing arg1"
			time.sleep(.5)
			ser.write("%s\n" % arg1)
			time.sleep(.5)
			becoming = ser.readline().strip()
			print becoming
			if becoming == "Received arg1":
				print "writing arg2"
				time.sleep(.5)
				ser.write("%s\n" % arg2)
				time.sleep(.5)
				#print "I got to here"
				becoming = ser.readline().strip()
				print becoming
				if becoming == "Received arg2":
					print "writing arg3"
					time.sleep(.5)
					ser.write("%s\n" % arg3)
					#print "I got to here"
					time.sleep(.5)
					becoming = ser.readline().strip()
					print becoming
					if becoming == "Received arg3":
						break
					print "woohoo"
					break
 						
#How to receive data thru XBee			
def rec_full_data(Name):
	while True:
		ser.write("%s\n" % Name)
		time.sleep(1)
		if ser.readline().strip() == Name:
			time.sleep(.5)
			ser.write("Go\n")
			print "ready for arg1"
			time.sleep(1)
			incoming = ser.readline().strip()
			while True:
				try:
					incoming = ser.readline().strip()
					print "incoming is: %s" % incoming
					arg1 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg1\n")
					print "Arg1 worked!"
					break
				except ValueError, e:
					print "error", e
					time.sleep(1)
					
	
			#ready for arg2		
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			while True:
				try:
					incoming = ser.readline().strip()
					print "incoming is: %s" % incoming
					arg2 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg2\n")
					print "Arg2 worked!"
					break
				except ValueError, e:
					print "error", e
					time.sleep(1)
					

			#ready for arg3
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			while True:
				try:
					incoming = ser.readline().strip()
					print "incoming is: %s" % incoming
					arg3 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg3\n")
					print "Arg3 worked!"
					break
				except ValueError, e:
					print "error", e
					time.sleep(1)
					
			return Name, arg1, arg2, arg3
 
#How to receive chars thru XBee
def rec_key(Name):
	while True:
		ser.write("%s\n" % Name)
		time.sleep(1)
		if ser.readline().strip() == Name:
			time.sleep(.5)
			print Name
			ser.write("Go\n")
			print "ready for arg1"
			time.sleep(.5)
			incoming = ser.readline().strip()
			print "incoming is: %s" % incoming
			while True:
				if incoming == Name:
					print "still failing"
					bc = ser.readline().strip()
					if bc != Name:
						arg1 = bc
						print "got it"
						ser.write("Received arg1\n")
						break
				else:
					arg1 = incoming
					print "got it"
					ser.write("Received arg1\n")
					break
			return Name, arg1
 		
#How to send chars thru XBee
def send_key(Name, arg1):
	while True:
		ser.write("%s\n" % Name)
		time.sleep(.5)
		incoming = ser.readline().strip()
		if incoming == "Go":
			print "writing arg1"
			time.sleep(.5)
			while True:
				ser.write("%s\n" % arg1)
				print "wrote"
				time.sleep(.5)
				becoming = ser.readline().strip()
				print "becoming is: %s" % becoming
				if becoming == "Received arg1":
					return
	
	
def traveling():
	while True:
		global x, y, z, key, vel, accel, correction_x, correction_y, correction_z, trkx, trky
		global elat, elon, ealt, submode, edist, between, homedist
		count = 1
		home1 = homedist()
		if home1 >= 1000:
			ser.write("Vehicle is out of bounds\n")
			time.sleep(1)
			ser.write("Vehicle is RTB\n")
			submode = "landing"
			break
		a = float(x - vehicle.location.global_relative_frame.lat)
		b = float(y - vehicle.location.global_relative_frame.lon)		
		c = float(z - vehicle.location.global_relative_frame.alt) + 2
		int(a)
		int(b)
		int(c)
		#newLoc = LocationGlobal (vehicle.location.global_frame.lat + a, vehicle.location.global_frame.lon + b, vehicle.location.global_frame.alt + c)
		#gotoGPS(newLoc)		
		
		#print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		#print " Enroute to Lat:%s, Lon:%s, Alt:%s" % (x,y,z)
		ser.write("Current Location: Lat:%s, Lon:%s, Alt:%s\n" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt))
		ser.write("Enroute to Lat:%s, Lon:%s, Alt:%s\n" % (x,y,z))
		
		dist1 = distance()
		between1 = between()
		#print " Distance to Waypoint: %s" % dist1
		#print "Distance to Enemy: %s" % between1
		ser.write("Distance to Waypoint: %s\n" % dist1)
		ser.write("Distance to Enemy: %s\n" % between1)
			
		time.sleep(2)
		La = float(vehicle.location.global_relative_frame.lat)
		la = float(vehicle.location.global_relative_frame.lon)
		Lb = float(elat)
		lb = float(elon)
		int(La)
		int(la)
		int(Lb)
		int(lb)
		U = math.cos(math.radians(Lb))*math.sin(math.radians(lb-la))
		T = math.cos(math.radians(La))*math.sin(math.radians(Lb))-math.sin(math.radians(La))*math.cos(math.radians(Lb))*math.cos(math.radians(lb-la))
		Bearing = math.atan2(U,T)
		if Bearing < 0:
			Hdg = math.degrees(Bearing)+360
		else:
			Hdg = math.degrees(Bearing)
		
		"""
		conditionyaw(Hdg)
		while True:
			print " Heading: ", vehicle.heading
			ser.write("Heading: %s\n" % vehicle.heading)
			if Hdg - 5 <= vehicle.heading and Hdg + 5 >= vehicle.heading:
				print "Lock-on"
				ser.write("Lock-on\n")
				break
		"""
				
		#check if in the correct position
		##############################################
		#need to add the function to determine
		
		if abs(between1) < 100 and abs(dist1) < 2: # Update the numbers  is this necessary? abs(ealt - vehicle.location.global_relative_frame.alt) <= 1 
			submode = "intercept"
			ser.write("Switching to intercept\n")
			break
		else: 
			ser.write("Need new set of coords\n")
			[Name, x, y, z] = rec_full_data("WP")
			[Name, elat, elon, ealt] = rec_full_data("EnemyWP")
		
		submode = "intercept"
		ser.write("Switching to intercept\n")
		break
		
		
def intercept():
	while True:
		global x, y, z, key, vel, accel, correction_x, correction_y, correction_z, trkx, trky
		global edist, elat, elon, ealt, submode, yaw, turn, between, climb, red, meh, loft, homedist
		climb = vehicle.location.global_frame.alt
		meh = 3
		home1 = homedist()
		if home1 >= 1000:
			ser.write("Vehicle is out of bounds\n")
			time.sleep(1)
			ser.write("Vehicle is RTB\n")
			submode = "landing"
			break
		
		ser.write("Key Request\n")
		
		
		while True:
			[Name, key] = rec_key("key")
			break
		
		print "climb: %s" % climb
		#print home_lat
		#print home_lon
		#print home_alt
		#key=msvcrt.getch()
		print "key is %s" % key
		
		 
		yaw = vehicle.heading
		between1 = between()
		d1 = between1
		start = time.time()
		time.sleep(1)
		
		between2 = between()
		d2 = between2
		end = time.time()
		tgt = d2-d1
		vel = (tgt/(end-start))*.9
		
		#print tgt
		#print trkx
		#print trky
		#print vel
		#tracking(vel*trkx, vel*trky,0,5)
		
		#not sure if we want drone to automatically go towards other enemy or make that a manual decision
		"""
		if abs(ealt - vehicle.location.global_relative_frame.alt) > 1 and abs(between1) > 30:
			submode = "goto"
			#ser.write("Acquired new target\n")
			break
		"""
		
		
		#if between > 5:
			#turn = 15
			#time = 1
		#elif between<5:
			#turn = 5
			#time = .5
		if key == "a": 
			ser.write("(A) works\n")
			#strafe()
			#newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat + (s_lat*.00005), vehicle.location.global_relative_frame.lon - (s_lon*.00005), vehicle.location.global_frame.alt)
			#gotoGPS(newLoc)
			
			
		elif key == "d":
			ser.write("(D) works\n")
			#strafe()
			#newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat - (s_lat*.00005), vehicle.location.global_relative_frame.lon + (s_lon*.00005), vehicle.location.global_frame.alt)
			#gotoGPS(newLoc)
			
		elif key == "w":
			ser.write("(W) works\n")
			#strafe()
			#newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat + (f_lat*.00005), vehicle.location.global_relative_frame.lon + (f_lon*.00005), vehicle.location.global_frame.alt)
			#gotoGPS(newLoc)
			
			
		elif key == "s":
			ser.write("(S) works\n")
			#strafe()
			#newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat + (b_lat*.00005), vehicle.location.global_relative_frame.lon + (b_lon*.00005), vehicle.location.global_frame.alt)
			#gotoGPS(newLoc)
			
		#makes the drone fly down 1 m
		elif key == "z":
			ser.write("(Z) works\n")
			#loft = climb - meh
			#lift = lift - 3
			#print "going down"
			#ser.write("Descending\n")
			#newLoc = LocationGlobal (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon , loft)
			#gotoGPS(newLoc)
			#while True:
				#if vehicle.location.global_frame.alt <= loft + 1: #Can remove the hard number if mission planner waypoint radius is changable
					#print "check"
					#break
			
			
			
		#makes the drone fly up 1 m
		elif key == "x":
			ser.write("(X) works\n")
			#loft = climb + meh
			#print "going up"
			#ser.write("Ascending\n")
			#newLoc = LocationGlobal (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, loft)
			#gotoGPS(newLoc)
			#while True:
				#if vehicle.location.global_frame.alt >= loft - 1: #Can remove the hard number if mission planner waypoint radius is changable
					#print "check"
					#break
			
		
		#p leaves function
		elif key == "p":
			ser.write("(P) works\n")
			#Used to track another target
			#Get WP and enemy's WP
			submode = "goto"
			break
		"""
		elif key == "t":
			print "Is this a Real World Test? (Y/N)"
			#safe=msvcrt.getch()
			if safe == "y":
				print "set new tgt. (N,S,E,W)"
				#red=msvcrt.getch()
				print "red is %s" % red
				if red == "n":
					elat = 39.793893
					elon = -84.17082
					ealt = 10
					
				elif red == "s":
					elat = 39.793723
					elon = -84.171005
					ealt = 10
					
				elif red == "e":
					elat = 39.793708
					elon = -84.170764
					ealt = 10
					
				elif red == "w":	
					elat = 39.793908
					elon = -84.17106
					ealt = 10
					
				elif red == "h":
					elat = 39.793805
					elon = -84.170903
					elat = 10
			
			else:
				print "set new tgt. (N,S,E,W)"
				red=msvcrt.getch()
				print "red is %s" % red
				if red == "n":
					elat = -35.362339
					elon = 149.165204
					ealt = 10
					
				elif red == "s":
					elat = -35.364806
					elon = 149.165244
					ealt = 10
					
				elif red == "e":
					elat = -35.36311
					elon = 149.166904
					ealt = 10
					
				elif red == "w":	
					elat = -35.36315
					elon = 149.163761
					ealt = 10
					
				elif red == "h":	
					elat = -35.363261
					elon = 149.1652299
					ealt = 10
		"""
		elif key == "t":
			ser.write("(T) works\n")
			"""
			strafe()
			loft = climb + meh
			print "Engaging!"
			ser.write("Engaging!\n")
			newLoc = LocationGlobal (vehicle.location.global_frame.lat + (b_lat*.00003), vehicle.location.global_frame.lon + (b_lon*.00003), loft)
			gotoGPS(newLoc)
			while True:
				if vehicle.location.global_frame.alt >= loft - 1: #Can remove the hard number if mission planner waypoint radius is changable
					break
			
			
		elif key == "k":
			repeat = 0
			#pwm.stop()
			pwm.start(10)
			time.sleep(10)
			pwm.stop
			while True:
				loft = climb + meh
				strafe()
				newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat + (s_lat*.00005), vehicle.location.global_relative_frame.lon - (s_lon*.00005), loft)
				gotoGPS(newLoc)
				time.sleep(1.2)
				strafe()
				newLoc = LocationGlobal (vehicle.location.global_relative_frame.lat - (s_lat*.00005), vehicle.location.global_relative_frame.lon + (s_lon*.00005), loft)
				gotoGPS(newLoc)
				time.sleep(1.2)
				repeat = repeat + 1
				if repeat == 2:
					break
			"""	
				
		#n releases another net
		elif key == "n":
			#pwm.stop()
			pwm.start(5)
			time.sleep(6)
			pwm.stop
			submode = "goto"
			
			
		elif key == "/":
			submode = "landing"
			break
		#tracking(vel*trkx, vel*trky, 0, 1)
		#print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		ser.write("Current Location: Lat:%s, Lon:%s, Alt:%s\n" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt))
		La = float(vehicle.location.global_relative_frame.lat)
		la = float(vehicle.location.global_relative_frame.lon)
		Lb = float(elat)
		lb = float(elon)
		int(La)
		int(la)
		int(Lb)
		int(lb)
		U = math.cos(math.radians(Lb))*math.sin(math.radians(lb-la))
		T = math.cos(math.radians(La))*math.sin(math.radians(Lb))-math.sin(math.radians(La))*math.cos(math.radians(Lb))*math.cos(math.radians(lb-la))
		Bearing = math.atan2(U,T)
		if Bearing < 0:
			Hdg = math.degrees(Bearing)+360
		else:
			Hdg = math.degrees(Bearing)
		#print Bearing
		"""
		conditionyaw(Hdg)
		while True:
			
			print " Heading: ", vehicle.heading
			ser.write("Heading: %s\n" % vehicle.heading)
			if Hdg - 5 <= vehicle.heading and Hdg + 5 >= vehicle.heading:
				print "Lock-on"
				break
		"""		
		print "Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		print vehicle.heading
		ser.write("Current Location: Lat:%s, Lon:%s, Alt:%s\n" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt))
		
		print "tgt is located at %s" % Hdg
		ser.write("Tgt heading is %s\n" % Hdg)
		ser.write("Vehicle heading is %s\n" % vehicle.heading)
print " Current-Location: Lat:%s, Lon:%s, Alt: %s" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
ser.write("Current Location: Lat:%s, Lon:%s, Alt:%s\n" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt))


##########################

#START OF MAIN CODE

##########################
#Get WP and enemy's WP

ser.write("Battery: %s\n") % vehicle.battery.level
"""
[Name, x, y, z] = rec_full_data("WP")

ser.write("Lat:%s, Lon:%s, Alt:%s\n" % (x, y,z))
[Name, elat, elon, ealt] = rec_full_data("EnemyWP")

ser.write("ELat:%s, ELon:%s, EAlt:%s\n" % (elat, elon, ealt))
"""

#print "Type Latitude"
#x=float(raw_input())
x=vehicle.location.global_frame.lat
#int(x)
#print "Type Longitude"
#y=float(raw_input())
y=vehicle.location.global_frame.lon
#int(y)
#print "Type Altitude"
#z=float(raw_input())
z=.1
#int(z)

elat=vehicle.location.global_frame.lat+.00002
elon=vehicle.location.global_frame.lon+.00002
elat=vehicle.location.global_frame.alt
 
##############################
##Sets home point coordinates
home_lat = float(vehicle.location.global_frame.lat)
int(home_lat)
home_lon = float(vehicle.location.global_frame.lon)
int(home_lon)
home_alt = .1

##############################
## Transit code
#pwm.start(1)
##############################################################################################
print "Arming"
ser.write("Arming\n")

##arms drone and lifts drone
arm_and_takeoff(.1)
time.sleep(.5)	

#program start
submode = "goto"
while True:
	if not vehicle.mode == VehicleMode("GUIDED"):
		print vehicle.mode
		print "manual"
		ser.write("manual\n")
		ser.write("Mode: %s\n" % vehicle.mode)
	if vehicle.mode == VehicleMode("GUIDED"):
		submode = "goto"
		if submode == "goto":
			print "submode is goto"
			print "traveling"
			ser.write("traveling\n")
			traveling()
		if submode == "intercept":
			print "submode is intercept"
			print "intercepting"
			ser.write("intercepting\n")
			intercept()
		if submode == "landing":
			print "submode is landing"
			print "landing mode"
			ser.write("landing mode\n")
			break
			

##############################
##Send drone home at the altitude set in the "Home Point" section
a = float(home_lat - vehicle.location.global_relative_frame.lat)
b = float(home_lon - vehicle.location.global_relative_frame.lon)		
c = float(home_alt - vehicle.location.global_relative_frame.alt)
d = vehicle.location.global_relative_frame.lat
e = vehicle.location.global_relative_frame.lon
f = vehicle.location.global_relative_frame.alt
int(a)
int(b)
int(c)

##############################

Home = LocationGlobal (vehicle.location.global_frame.lat+a, vehicle.location.global_frame.lon+b, vehicle.location.global_frame.alt+c)
#gotoHome(Home)	
	
##Slow decent to set alt, example: RTL("##") where "##" is the landing altitude, currently set >0 to account for landing gear
RTL(.10)	
	
print "Landing Alt Reached"	
ser.write("Landing Alt Reached\n")

##Countdown warning before drone powers off props
count = 5
while count>0:
	print count
	ser.write("%s" % count)
	count = count-1
	time.sleep(1.5)
	

time.sleep(3)
print "Done"
ser.write("Done\n")
time.sleep(2)
#GPIO.cleanup()
	
vehicle.close()	
	
ser.close()	
##Ending Sim command
#sitl.stop()
