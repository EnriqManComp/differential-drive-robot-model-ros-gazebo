#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


twist_msg = Twist()

control = {
    'w': (0.2,0.0)
}

def shutdown_hook():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist_msg.linear.x = 0.0 # Stop linear movement
    twist_msg.angular.z = 0.0 # Stop angular movement
    print('Shutting Down ...')
    pub.publish(twist_msg)

def controls_callback(msg):
    (x,z) = control[msg.data]
    twist_msg.linear.x = x # Modify linear velocity
    twist_msg.angular.z = z # Modify angular velocity
    print('Velocities:['+str(x)+','+str(z)+']')
def controls():
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    rospy.Subscriber('/controls',String,controls_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.on_shutdown(shutdown_hook) # Shut down callback function
    rate = rospy.Rate(10) # message publish frequency
    while not rospy.is_shutdown():        
        print('Listening ...')
        pub.publish(twist_msg)
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        controls()
    
    except rospy.ROSInterruptException:
        pass