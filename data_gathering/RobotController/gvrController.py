
#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

currentState = 0 #stop is 0, forward is 1, reverse is 2, spin iz 3

def getter(msg):
    global currentState
    if msg.buttons[1]: #B button to STOP
        print("STAHP")
        currentState = 0
    elif(msg.buttons[0]): #A button to GO
        print("GO")
        currentState = 1
    elif(msg.buttons[2]): #X button to REVERSE
        currentState = 2
        print("REVERSE")
    elif(msg.buttons[3]): #Y button to SPIN in circle
        print("SPIN")
        currentState = 3
    elif(msg.axes[1] == 1 and msg.axes[-3] == -1): # max speed if going foward and R-trigger is pulled
        currentState = 4

# this is executed if ran from cli
if __name__ == '__main__':
    try:
        # create local node this script will identify as
        rospy.init_node('listener', anonymous=True)

        # Subscriber capability on given path enabled
        #       /joy:  the path to listen on
        #        Joy:  passes the object definition will recv
        #     getter:  passes ptr to callback function on recv
        rospy.Subscriber("/joy", Joy, getter)

        # Publisher capability on given path enabled
        #   /cmd_vel: the path to publish on
        #      Twist: passes a sample data type to send
        # queue_size: limits recv_node queue
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # sets cmd send rate to 10 times/ second
        rate = rospy.Rate(10) #10 Hz

        # inits the payloads to send to the robot
        stop = Twist(Vector3(0,0,0),Vector3(0,0,0))
        forward = Twist(Vector3(.2,0,0),Vector3(0,0,0))
        reverse = Twist(Vector3(-.2,0,0),Vector3(0,0,0))
        spin = Twist(Vector3(0,1,0),Vector3(0,0,0))
        lightspeed = Twist(Vector3(1,0,0),Vector3(0,0,0))
        
        # creates dict to reference and send payload
        msg = {0:stop,1:forward,2:reverse,3:spin,4:lightspeed}

        # loop to send commands until halt
        while not rospy.is_shutdown():

            # each loop depending on which button is pushed we send it
            # to the robot to be executed
            pub.publish(msg[currentState])

            # must sleep so as to not overwhelm the robot
            # with unnecessary amounts of commands
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    
    except KeyboardInterrupt:
        print "User canceled...Exiting"
        #TODO handle this and clean exit
