# Autonomy
#Raw file for Engagement system flight. Note: View in "raw" tab setting for an easier view           ^

####################################################
##Commander's Challenge Program for Drone Automation
##Starts Drone
##		Flies to multiple waypoints
##		Returns to starting location, lands/shutdown - Press "L" or "P" when done flying
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
#find = None 

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

#vehicle = connect("/dev/ttyUSB0", wait_ready=True, 57600) #Connected via RPi

#vehicle = connect('com3', wait_ready=True, baud =57600) 
#time.sleep(2)
#print "Autopilot Firmware version: %s" % vehicle.version

#############################################

## Functions 

#Arms and rises to given Altitude
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    #print "Basic pre-arm checks"
        #Don't try to arm until autopilot is ready
    #while not vehicle.is_armable:
       #print " Waiting for vehicle to initialise..."
       #time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
	# Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

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
	vehicle.simple_goto(location)

#Sends vehicle home
def gotoHome(location):  ###Note: Groundspeed must be set via missionplanner
	print "I'm coming home!"
	vehicle.simple_goto(location)
	while True:
		print " Location: Lat:%s, Lon:%s, Alt: %s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		print " Distance to Waypoint: %s" % (math.sqrt(((100000*home_lat)-(100000*vehicle.location.global_relative_frame.lat))**2+((100000*home_lon)-(100000*vehicle.location.global_relative_frame.lon))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2))
		#if vehicle.location.global_relative_frame.alt>= z*0.95 and vehicle.location.global_relative_frame.lat>= x*0.80 and vehicle.location.global_relative_frame.lon>= y*0.80:
		if math.sqrt(((100000*home_lat)-(100000*vehicle.location.global_relative_frame.lat))**2+((100000*home_lon)-(100000*vehicle.location.global_relative_frame.lon))**2+(home_alt-(vehicle.location.global_relative_frame.alt))**2)<=1:
		#if math.sqrt((x-(d*(-1)))**2+(y-(e*(-1)))**2+(z-(f*-(1))**2))<=2:
			print "The Drone is Back."
			break
		time.sleep(1)

#Returns vehicle to ground		
def RTL(landingAlt):
    print "Landing"
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt>=landingAlt:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
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
def track(heading):
	global trkx
	global trky
	if vehicle.heading == 0:
		trkx = -1
		trky = 0
	elif vehicle.heading < 15:
		trkx = -1
		trky = -.25
	elif vehicle.heading>=15 and vehicle.heading<=30:
		trkx = -1
		trky = -.5
	elif vehicle.heading>30 and vehicle.heading<37.5:
		trkx = -1
		trky = -.75
	elif vehicle.heading>=37.5 and vehicle.heading<=52.5:
		trkx = -1
		trky = -1
	elif vehicle.heading>52.5 and vehicle.heading<60:
		trkx = -.75
		trky = -1
	elif vehicle.heading>=60 and vehicle.heading<=75:
		trkx = -.5
		trky = -1	
	elif vehicle.heading>75 and vehicle.heading<90:
		trkx = -.25
		trky = -1
	elif vehicle.heading == 90:
		trkx = -0
		trky = -1
	elif vehicle.heading>90 and vehicle.heading < 105:
		trkx = .25
		trky = -1
	elif vehicle.heading>=105 and vehicle.heading<=120:
		trkx = .5
		trky = -1
	elif vehicle.heading>120 and vehicle.heading<127.5:
		trkx = .75
		trky = -1
	elif vehicle.heading>=127.5 and vehicle.heading<=142.5:
		trkx = 1
		trky = -1
	elif vehicle.heading>142.5 and vehicle.heading<150:
		trkx = 1
		trky = -.75
	elif vehicle.heading>=150 and vehicle.heading<=165:
		trkx = 1
		trky = -.5	
	elif vehicle.heading>165 and vehicle.heading<180:
		trkx = 1
		trky = -.25
	elif vehicle.heading == 180:
		trkx = 1
		trky = 0
	elif vehicle.heading>180 and vehicle.heading < 195:
		trkx = 1
		trky = .25
	elif vehicle.heading>=195 and vehicle.heading<=210:
		trkx = 1
		trky = .5
	elif vehicle.heading>210 and vehicle.heading<217.5:
		trkx = 1
		trky = .75
	elif vehicle.heading>=217.5 and vehicle.heading<=232.5:
		trkx = 1
		trky = 1
	elif vehicle.heading>232.5 and vehicle.heading<240:
		trkx = .75
		trky = 1
	elif vehicle.heading>=240 and vehicle.heading<=255:
		trkx = .5
		trky = 1	
	elif vehicle.heading>255 and vehicle.heading<270:
		trkx = .25
		trky = 1
	elif vehicle.heading == 270:
		trkx = 0
		trky = 1
	elif vehicle.heading > 270 and vehicle.heading< 285:
		trkx = -.25
		trky = 1
	elif vehicle.heading>=285 and vehicle.heading<=300:
		trkx = -.5
		trky = 1
	elif vehicle.heading>300 and vehicle.heading<307.5:
		trkx = -.75
		trky = 1
	elif vehicle.heading>=307.5 and vehicle.heading<=322.5:
		trkx = -1
		trky = 1
	elif vehicle.heading>322.5 and vehicle.heading<330:
		trkx = -1
		trky = .75
	elif vehicle.heading>=330 and vehicle.heading<345:
		trkx = -1
		trky = .5	
	elif vehicle.heading>345:
		trkx = -1
		trky = .25

#"track" is nothing compared to this insanity	
def killingjoke():
	print "Holding Position"
	ser.write("Holding Postion\n")
#########################
## Starts loop for recieving and flying to multiple coords
	while True:
		global x
		global y	
		global z
		global key
		global vel
		global accel
		global correction_x
		global correction_y
		global correction_z
		global trkx
		global trky
		
		

	##Will ask user if a threat is identified; if not, will update coords and try again, otherwise will switch to manual	
		print "Do you want to land? Press(Y or N)"
		ser.write("To land, press (y). To lower net, press (t).\n")
		#key=msvcrt.getch()
		while True:
			[Name, key] = rec_char("key")
			break
		
		
		#uncomment for XBee
		
		#print "Do you see the threat Y/N"
		#while True:
			#[Name, key] = rec_char("Threat Seen Y/N?")
			#break
		#print "key is %s" % key
		
		if key == "Y" or key == "y":
			ser.write("Drone is now RTB\n")
			break
			
		elif key == "T" or key == "t":
			ser.write("Lowering Net.\n")
			#pwm.stop()
			pwm.start(5)
			time.sleep(6)
			pwm.stop
			#submode = "goto"
			
		
		else:
			pass
		

			
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
	
	


print " Current-Location: Lat:%s, Lon:%s, Alt: %s" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
#############################################
#Get WP and enemy's WP

ser.write("Battery: %s\n") % vehicle.battery.level

"""
[Name, x, y, z] = rec_full_data("WP")

print "Latitude is: ", x
print "Longitude is: ", y
print "Altitude is: ", z
print "Type is: ", Name

[Name, elat, elon, ealt] = rec_full_data("EnemyWP")

print "Enemy Latitude is: ", elat
print "Enemy Longitude is: ", elon
print "Enemy Altitude is: ", ealt
print "Type is: ", Name
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
z=5
#int(z)


##############################
##Sets home point coordinates
home_lat = float(vehicle.location.global_frame.lat)
int(home_lat)
home_lon = float(vehicle.location.global_frame.lon)
int(home_lon)
home_alt = 5

##############################
## Transit code
print "Arming"
ser.write("Arming\n")
time.sleep(2)
##arms drone and lifts drone
arm_and_takeoff(5)
time.sleep(2)	

##This is where the crazy happens
killingjoke()
time.sleep(3)	

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
#newLoc = LocationGlobal (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
Home = LocationGlobal (vehicle.location.global_frame.lat+a, vehicle.location.global_frame.lon+b, vehicle.location.global_frame.alt+c)
gotoHome(Home)	
	
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
