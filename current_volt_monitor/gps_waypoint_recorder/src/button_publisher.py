#!/usr/bin/env python
# license removed for brevity
import rospy, time
import RPi.GPIO as GPIO
from std_msgs.msg import Bool



GPIO.setmode(GPIO.BCM) # configure RPi pinout reference to GPIO (not physical layout)
GPIO.setup(5, GPIO.IN, GPIO.PUD_UP) #configure pull-up resistor on push-button input, GPIO #5


def button_talker():
    pub = rospy.Publisher('button_state', Bool, queue_size=1)
    rospy.init_node('button_poller', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    button = 5 # using GPIO 5 as push button input for recording GPS location
    prevButtonReading = 1 #record previous button reading; initialize to HIGH (due to pull-up resistors) 
    pushState = Bool()
    pushState = False # initialize push button state variable to False (toggles after every press); 
    while not rospy.is_shutdown():
        buttonReading = GPIO.input(button) #read push button        
        #check both current & past button reading;
	#previous button reading is needed so that, if button is pushed for a long time, 
	#only 1 state change is made, regardless of the push duration
	if buttonReading == GPIO.LOW and prevButtonReading == 1: # if button transition, then update state machine
            print ("Button Pushed")
            pushState = not (pushState) 	#toggle the state variable
            prevButtonReading = 0
	elif buttonReading == GPIO.HIGH:  	#if button not read as "pushed", then button is HIGH
	    prevButtonReading = 1 	  	#change previous button state to HIGH
        pub.publish(pushState)
        rate.sleep()

if __name__ == '__main__':
    try:
        button_talker()
    except rospy.ROSInterruptException:
        pass

