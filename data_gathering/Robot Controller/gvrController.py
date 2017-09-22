
#!/usr/bin/env python
# license removed for brevity
import rospy
import time
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
    #elif(msg.axes[-1]): #Last reference is last element in the array      
    #    pub.queue_size=10
    elif(msg.axes[1] == 1 and msg.axes[-3] == -1):
        currentState = 4

if __name__ == '__main__':
    try:
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/joy", Joy, getter)
        rate = rospy.Rate(10) #10 Hz
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        post = Twist(Vector3(0,0,0),Vector3(0,0,0))
        forward = Twist(Vector3(.2,0,0),Vector3(0,0,0))
        reverse = Twist(Vector3(-.2,0,0),Vector3(0,0,0))
        spin = Twist(Vector3(0,1,0),Vector3(0,0,0))
        lightspeed = Twist(Vector3(1,0,0),Vector3(0,0,0))       
#        rospy.spin()
        while not rospy.is_shutdown():
            # do stuff w globals
            msg = {0:post,1:forward,2:reverse,3:spin,4:lightspeed}
            pub.publish(msg[currentState])
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
