import serial
import time
import socket
import struct
import msvcrt

ser = serial.Serial('com7', 9600, timeout = 0.5)
UDP_IP = "10.6.3.1"
UDP_PORT = 5005

#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock.bind((UDP_IP, UDP_PORT))
#sock.listen(1)
#conn, addr = sock.accept()
#print "Connected by: ", addr



def send_full_data(Name, arg1, arg2, arg3):
	while True:
		ser.write("%s\n" % Name)
		print Name
		time.sleep(.5)
		incoming = ser.readline().strip()
		if incoming == "Go":
			print "writing arg1"
			time.sleep(1)
			ser.write("%s\n" % arg1)
			time.sleep(1)
			becoming = ser.readline().strip()
			print "should receive"
			print becoming
			if becoming == "Received arg1":
				print "writing arg2"
				time.sleep(1)
				ser.write("%s\n" % arg2)
				time.sleep(1)
				#print "I got to here"
				becoming = ser.readline().strip()
				print becoming
				if becoming == "Received arg2":
					print "writing arg3"
					time.sleep(1)
					ser.write("%s\n" % arg3)
					#print "I got to here"
					time.sleep(1)
					becoming = ser.readline().strip()
					print becoming
					if becoming == "Received arg3":
						break
					print "woohoo"
					break

			
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
					incoming = ser.readline().strip()
					print "incoming is: %s" % incoming
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
					time.sleep(.5)
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
		print "waiting for GO"
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


#i = 1
lat = None
lon = None
alt = None
elat = None
elon = None
ealt = None

while True:
	"""
	data = conn.recv(1024)
	lat,lon,alt,elat,elon,ealt = data.split(",")
	print "received message: ", data
	lat = float(lat)
	lon = float(lon)
	alt = float(alt)
	elat = float(elat)
	elon = float(elon)
	ealt = float(ealt)	
	"""
	incoming = ser.readline().strip()
	print "Drone says: %s" % incoming
	if incoming == "WP":
		print "Asked for WP"
		time.sleep(.5)
		send_full_data("WP", lat, lon, alt)
		#send_full_data("WP", 39.793828, -84.171092, 12)
	elif incoming == "EnemyWP":
		print "Asked for EnemyWP"
		time.sleep(.5)
		send_full_data("EnemyWP", elat, elon, ealt)
		#send_full_data("EnemyWP", 42, 42, 3)
	elif incoming == "Key":
		print "asked for key"
		time.sleep(.5)
		print "Type Key Now"
		key = msvcrt.getch()
		send_key("key", key)

	

ser.close()
#sock.close()
