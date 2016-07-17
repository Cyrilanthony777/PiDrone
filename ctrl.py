####################################Credits##################################################

# Code Written by Cyril Anthony mail: cyril.anthony777@yahoo.com

# Code Ver. 1.05a











##################################### Things to be Done ######################################




# Error Counter for ADC , PPM and PX4FLOW
# Failsafe Entire System
# Object tracking interface
# UDP for reading Object tracking Data
# Programlink for OpticalFlow data.
# Optical Flow position hold
# Flight Controller Support for APM/PIXHAWK
# MAVLink Support
# Mission Commands
# GPS Support
# ROS Support
# PID Support








################################### Things Done ########################################

# I2C Read and write PPM and analog data
# UDP Data Send / Receive data and passcode protection
# Collision avoidance using Propotional algorithm.
# PX4FLOW Support
# Config File












#########################################################################################


import smbus
import time
import numpy
from struct import *
import socket
import os

conf_path = './piqd.conf'


f = os.popen('ifconfig wlan0 | grep "inet\ addr" | cut -d: -f2 | cut -d" " -f1')
ip=f.read()

cas = 1
passkey="a1856d0"
remoteip = ""
IP_ADDR = ip
PORT = 5006
udp = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
udp.bind((IP_ADDR,PORT))
udp.settimeout(0.01)


bus = smbus.SMBus(1)
currcmd = 0 
xdir=0
ydir=0
qual=0
altitude=0
CH1=1500
CH2=1500
CH3=1000
CH4=1500
CH5=1520
CH6=1500 
d1 = 0
d2 = 0
d3 = 0
d4 = 0
d5 = 0
d6 = 0
rcBuff = []
fl=1
an=1
pm=1
ini=0

fMode = 0						# 1 = Manual Mode PPM ; 2 = Manual Mode UDP ; 3 = Mission Execution Mode

client = 0
rxStream = []
txStream = []
i2caddr = 0x0A
addrADC = 0x19
addrP4F = 0x42
addrPPM = 0x14

tmps = ["RC",1500,1500,1000,1500,1520]


INTRC = 0
ACEN = 0
TRKEN = 0
ALTEN = 0
ARMEN = 0
DARM = 0
ARMED = 0
flying = 0
Sp = 60
Kp = 3

############################################################
def mapInt(vare,iLow,iHigh,oLow,oHigh):
	xmp = (vare - iLow) * (oHigh - oLow) / (iHigh - iLow) + oLow
	return xmp
############################################################

def byte2Int(lbyte,hbyte):
		nt = chr(lbyte)+chr(hbyte)
		mt = unpack("h",nt)
		return mt[0]

############################################################

def int2Byte(xf):
		xg =int(xf)
		mx = pack("i",xg)
		return mx[0],mx[1]

############################################################
def writeRC(lst):
		c1h,c1l = int2Byte(lst[1])
		c2h,c2l = int2Byte(lst[2])
		c3h,c3l = int2Byte(lst[3])
		c4h,c4l = int2Byte(lst[4])
		c5h,c5l = int2Byte(lst[5])
		del txStream[:]
		txStream.append(ord(c1h))
		txStream.append(ord(c1l))
		txStream.append(ord(c2h))
		txStream.append(ord(c2l))
		txStream.append(ord(c3h))
		txStream.append(ord(c3l))
		txStream.append(ord(c4h))
		txStream.append(ord(c4l))
		txStream.append(ord(c5h))
		txStream.append(ord(c5l))
		txStream.append(0x80)
		#print txStream
		try:
			   bus.write_i2c_block_data(i2caddr,0x01,txStream)
		except:
				print "Error Handled"
				bus.read_byte(i2caddr)
				#exit()
			   

############################################################
def msg2int(incomming):
	lst = incomming.split(',')
	cmd = lst[0]
	p1 = lst[1]
	p2 = lst[2]
	p3 = lst[3]
	p4 = lst[4]
	p5 = lst[5]
	p6 = lst[6]
############################################################


def readConfig():
	global fl,an,pm,Sp,Kp
	print "Looking for Config File"
	if os.path.isFile(conf_path):
		print "Config  File Found"

		if os.path.getsize(conf_path)>0:
			print "Loading Data"
			# Read file and write data
		else:
			print "Config file Empty. Loading Defaults"
			fi = open(conf_path,"w")
			fi.write("LOAD,1,1,1,60,3")
			fi.close()
			# File Empty Load defaults
	else:
		print "Config File Not found."
		print "Creating Config file and writing defaults"
		fi = open(conf_path,"w")
		fi.write("LOAD,1,1,1,60,3")
		fi.close()
	fi = open(conf_path,"r")
	stri = fi.readline()
	fi.close()
	gdx = stri.split(",")
	fl = int(gdx[1])
	an = int(gdx[2])
	pm = int(gdx[3])
	Sp = int(gdx[4])
	Kp = int(gdx[5])


		# Create file and write Defaults



############################################################
def readFlow():
		byteStream = []
		try:
				byteStream = bus.read_i2c_block_data(addrP4F,0x00)
		except:
				print "Flow ERR"
		xdir=byte2Int(byteStream[2],byteStream[3])
		ydir=byte2Int(byteStream[4],byteStream[5])
		qty=byte2Int(byteStream[10],byteStream[11])
		alti=byte2Int(byteStream[20],byteStream[21])/10
		return xdir,ydir,qty,alti

############################################################
def getAlt():
	p,q,r,alt = readFlow()
	return alt
		
############################################################
def readADC():
		byteStr = []
		try:
				byteStr= bus.read_i2c_block_data(addrADC,0x00)
		except:
				print "ADC ERR"
				byteStr = [0,0,0,0,0,0,0,0,0,0,0,0]
		ADC1 = byte2Int(byteStr[1],byteStr[0])
		ADC2 = byte2Int(byteStr[3],byteStr[2])
		ADC3 = byte2Int(byteStr[5],byteStr[4])
		ADC4 = byte2Int(byteStr[7],byteStr[6])
		ADC5 = byte2Int(byteStr[9],byteStr[8])
		ADC6 = byte2Int(byteStr[11],byteStr[10])
		return ADC1,ADC2,ADC3,ADC4,ADC5,ADC6

############################################################

def readPPM():
		byteStr = []
		try:
                        byteStr= bus.read_i2c_block_data(addrPPM,0x00)
                except:
                        print "PPM ERR"
                        byteStr = [0,0,0,0,0,0,0,0,0,0,0,0]
                                
		C1 = byte2Int(byteStr[0],byteStr[1])
		C2 = byte2Int(byteStr[2],byteStr[3])
		C3 = byte2Int(byteStr[4],byteStr[5])
		C4 = byte2Int(byteStr[6],byteStr[7])
		C5 = byte2Int(byteStr[8],byteStr[9])
		C6 = byte2Int(byteStr[10],byteStr[11])
		return C1,C2,C3,C4,C5,C6

############################################################

def readPixy():
		exit()

############################################################

def ARM():
	am1 = ["RC",1000,1000,1000,2000,1520]
	am2 = ["RC",1500,1500,1500,1500,1520]
	writeRC(am1)
	time.sleep(2)
	writeRC(am2)
	time.sleep(1)

############################################################

def disARM():
		writeRC(["RC",1500,1500,1000,1500,1520])
		time.sleep(5)
		
############################################################

def takeOff():
	global flying
	altt = getAlit()
	if flying == 1:
		print "Quadcopter is already flying"
	else:
		ARM()
		while altt<150:
			writeRC(["RC",1500,1500,1600,1500,1520])
			flying = 1

################################################################        
		

def land():
	global flying
	if flying == 1:
		altt = getAlit()
		while altt>5:
			writeRC(["RC",1500,1500,1350,1500,1520])
		if altt<=5:
			writeRC(["RC",1500,1500,1000,1500,1520])
	else:
		print "Quadcopter Has Already Landed"
		
			
		
################################################################
def setALT():
		exit()
		
############################################################

def moveTo():
		exit()
		
############################################################

def Initailize():
	global fl,an,pm,fMode
	readConfig()
	print "Initializing Hardwares"
	try:
		readFlow()
		print "PX4Flow Detected"
		fl = 1
	except:
		fl=0
		print "Failed to Initailize PX4Flow"
	time.sleep(0.01)

	try:
		readADC()
		print "ADC Detected"
		an=1
	except:
		an=0
		print "Failed to Initailize Analog"
	time.sleep(0.01)
	try:
		pi1,pi2,pi3,pi4,pi5,pi6 = readPPM()
		print "PPM Input Detected"
		pm=1
	except:
		pm=0
		print "Failed to Initailize PPM Input"

	if pi1 == 0 or pi2 == 0 or pi3 == 0 or pi4 == 0:
		fMode = 0
	else:
		fMode = 1




	time.sleep(0.01)

############################################################

def avoidCollision(sense):
	global d1,d2,d3,d4,d5,d6
	if sense == 1:   #Sharp IR sensors
		e1 = d1-Sp
		e2 = d2-Sp
		e3 = d3-Sp
		e4 = d4-Sp
		cr1 = Kp*e1
		cr2 = Kp*e2
		cr3 = Kp*e3
		cr4 = Kp*e4
		ale = cr1-cr3
		ele = cr4-cr2
	elif sense == 2:
		d1 =d1/2
		d2 =d2/2
		d3 =d3/2
		d4 =d4/2
		e1 = d1-Sp
		e2 = d2-Sp
		e3 = d3-Sp
		e4 = d4-Sp
		cr1 = Kp*e1
		cr2 = Kp*e2
		cr3 = Kp*e3
		cr4 = Kp*e4
		ale = cr1-cr3
		ele = cr4-cr2
	return ale,ele


############################################################
def incData(incd):
	global rcBuff
	global currcmd
	global fMode,ACEN
	lst = incd.split(",")
	if lst[0] == "RC":
		del rcBuff[:]
		rcBuff.append(lst[0])
		rcBuff.append(lst[1])
		rcBuff.append(lst[2])
		rcBuff.append(lst[3])
		rcBuff.append(lst[4])
		rcBuff.append(1520)
		if fMode == 0:
			fMode = 2

	elif lst[0] == "TOFF":
		#Takeoff
		currcmd = 1
	elif lst[0]=="LAND":
		#land
		currcmd = 2
	elif lst[0] == "KILL":
		currcmd = 5
	elif lst[0] == "ACEN":
		ACEN = 1
	elif lst[0] == "ACDE":
		ACEN = 0
	else:
		currcmd = 0



############################################################
def telemetry():
	global client
	global remoteip
	if client == 0:
		try:
			data,addr= udp.recvfrom(7)
		except:
			client = 0
			data = ""
			addr = ""
		if data==passkey:
                        time.sleep(0.1)
			udp.sendto("OK",addr)
			remoteip = addr
			client = 1
			#sys.exit(0)
	elif client == 1:
		# Main Telemetry loop
		try:
			datastr,ads = udp.recvfrom(25)
		except:
			#print "Data Timeout"
			datastr = ""
		if datastr and ads==remoteip:
			incData(datastr)
		


############################################################

def sendFD():
	strFlow = "FLOW"+","+str(xdir)+","+str(ydir)+","+str(qual)+","+str(altitude)
	strPPM = "PPM"+","+str(CH1)+","+str(CH2)+","+str(CH3)+","+str(CH4)+","+str(CH5)+","+str(CH6)
	strADC = "ADC"+","+str(d1)+","+str(d2)+","+str(d3)+","+str(d4)+","+str(d5)+","+str(d6)
	strChk = "CHK"+","+str(fl)+","+str(an)+","+str(pm)+","+"0"
	if client == 1:
		udp.sendto(strFlow,remoteip)
		udp.sendto(strADC,remoteip)
		udp.sendto(strPPM,remoteip)
		udp.sendto(strChk,remoteip)
		return 1
	else:
		return 0

	

############################################################

def execMission():
	print "Exec"

############################################################


while 1:

	if ini==0:
		Initailize()
		ini = 1
	xdir,ydir,qual,altitude=readFlow()
	time.sleep(0.05)
	CH1,CH2,CH3,CH4,CH5,CH6 = readPPM()

	time.sleep(0.03)
	d1,d2,d3,d4,d5,d6 = readADC()
	telemetry()
	sendFD()

	if fMode == 1 and CH5 >= 1500:
		ACEN = 1
	elif fMode == 1 and CH5<1500:
		ACEN = 0

	if currcmd == 1 and fMode!=0:
		takeOff()
		currcmd = 0
	elif currcmd == 2 and flying == 1:
		land()
		currcmd = 0
	elif currcmd == 3 and flying == 1:
		writeRC(["RC",1500,1500,1000,1500,1520])
		currcmd = 0
		time.sleep(5)


	if ACEN == 1:
		aile,elev = avoidCollision(cas)
		CH1 = CH1 + aile
		CH2 = CH2 + elev
		if rcBuff:
			rcBuff[1] = rcBuff[1] + aile
			rcBuff[2] = rcBuff[2] + elev

		#Do collision avoidance
	if TRKEN ==1:
		print "TRK"

	
	if fMode == 1:
		writeRC(["RC",CH1,CH2,CH3,CH4,1520])
	elif fMode == 2:
		writeRC(rcBuff)




	#ARM()
	#writeRC(["RC",1500,1500,1650,1500,1520])
	#time.sleep(8)
	#writeRC(["RC",1500,1500,1000,1500,1520])
	#exit()
		

		
		
		


