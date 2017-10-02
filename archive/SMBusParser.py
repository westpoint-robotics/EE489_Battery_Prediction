# !/usr/bin/env python

"""
SMBUS Parser 
Authors: USMA GVR-BOT
Contact Dominique Tan for any questions pertaining to operation of this code

Purpose:  To pull data from BB2590 family of batteriesand insert it into a database
Using MYSQL.  Could be used with MARIADB and SQLlite (not tested)
Computing platform: RASPI
Hardware needed: RASPI 2 or 3, a way to connect to smbus via the i2c port.

"""
import mysql.connector
from smbus2 import SMBus
import time
from datetime import datetime

# ---- Database INIT ---- #
db = mysql.connector.connect(host="localhost", user="root", password="toor", database="batterystudy")
curs = db.cursor()


# ---- Battery SMBUS ---- #
bus = SMBus(1)  #SMBUS init

addr = 0x0b #SMBUS Address
serial = ""
cycle = ""
failedcycles = 0

#Exit when data is not being pulled from smbus
while cycle == "":
	print failedcycles
	if failedcycles == 13:
		print "SMBus is broken/diconnected"
		exit()
	try:
  		serial = str(bus.read_word_data(addr, 0x1c))     #Serial Number
    		cycle = str(bus.read_word_data(addr, 0x17))  # Cycle Count

	except IOError:
    		print "Could not connect to bus"
    		time.sleep(1)
        	failedcycles += 1
    		


# ---- MySQL data table init ---- #

table = raw_input("Make a new table name: ")
addtable = "CREATE TABLE " + str(table) + "(time DECIMAL(20,2), temperature NUMERIC, voltage NUMERIC, current NUMERIC, relsoc NUMERIC, abssoc NUMERIC, ttoempty NUMERIC,remcap NUMERIC, fullcap NUMERIC, changecurr NUMERIC, changevolt NUMERIC, maxerror NUMERIC);"

# ---- Table Management ---- #
try:
    curs.execute(addtable)
except:
    newerror = raw_input("Are you sure you want to overwrite table(This will delete table)? y/n:") #Overwrites if test are of the same name
    if newerror == ("y" or "yes"):
        deletetable = "DROP TABLE " + str(table)
        print "Deleting " + str(table)
        curs.execute(deletetable)
        curs.execute(addtable)
    else:
        print "Restart the program and use a different table name"
        time.sleep(2)
        exit()

# ---- Test Information ---- #    

batterynum = raw_input("Enter Battery name/num:") #Battery Number
testtype = raw_input("Enter Test Type(mobile, desk test, etc.): ") #Type of test the tester is running
testspeed = raw_input("Enter Test Speed(Low, Med, Fast, Idle): ") #The test speed.  Can be ommitted if doing different type of testing
gvrbottype = raw_input("Enter GVR Bot Name(Turtle, Bull): ") #GVRbot 23 = Bull GVRbot 25 = Turtle
testtime = raw_input("Enter test time(put in min or empty): ") #TODO: add a timer on logging
testdata = "\n\n\n\n\n" + table + "\n" + batterynum + "\nTest Type: " + testtype + "\nTest Speed: " + testspeed + "\nGVR Bot: " + gvrbottype + "\nTest Time: " + testtime + "\nSerial number: " + str(serial) + "\nCharge Cycles: " + str(cycle)

file = open(str(table) + ".txt", 'w') #Will overwrite old data
file.write(testdata)
file.close()

# ---- TODO Timed termination of test ----#
'''
Time tables
t_end = time.time() + 60 * 10

'''


# ---- Data pull from SMBUS ----#
print "Running Test CTRL+C to exit"
while "TRUE":
    # time.time() < t_end:
    try:


#--- SMBUS get commands ----#
    # miltime = datetime.utcnow().strftime('%H%M%S.%f')[:-2]
        miltime = time.time() #Common time logging (Dataq has the same time logging)
        voltage = bus.read_word_data(addr, 0x09)  # voltage
        current = bus.read_word_data(addr, 0x0a)  # current
        temperature = bus.read_word_data(addr, 0x08)  # temperature
        relsoc = bus.read_word_data(addr, 0x0d)  # relative state of charge
        abssoc = bus.read_word_data(addr, 0x0e)  # absolute state of charge
        ttoempty = bus.read_word_data(addr, 0x12)  # Average time to empty
        remcap = bus.read_word_data(addr, 0x0f)  # Remaining Capacity
        fullcap = bus.read_word_data(addr, 0x10)  # Full charge Capacity
        changingcurr = bus.read_word_data(addr, 0x14) #Changing current
        changingvolt = bus.read_word_data(addr, 0x15) #changing voltage
	maxerror = bus.read_word_data(addr, 0x0c) # max error soc

# ---- Troubleshooting ----#
        
        #print smbus
        #print voltage, current, temperature, relsoc, abssoc, cycle, ttoempty, remcap, fullcap, changingcurr, changingvolt
        

# ----- Insert data into the sql database ----#
        myquery = "INSERT INTO " + str(table) + "(time, temperature, voltage, current, relsoc, abssoc, ttoempty, remcap, fullcap, changecurr, changevolt, maxerror ) VALUES (" + str(miltime)[4:] + "," + str(temperature) + ", " + str(voltage) + "," + str(current) + "," + str(relsoc) + "," + str(abssoc) + "," + str(ttoempty) + "," + str(remcap) + "," + str(fullcap) + "," + str(changingcurr) + "," + str(changingvolt) + "," + str(maxerror) +  ")"
        curs.execute(myquery)
        db.commit()

    except IOError: #Catches common error of fast freqeuncy of logging
        print "Too fast"
        continue 
