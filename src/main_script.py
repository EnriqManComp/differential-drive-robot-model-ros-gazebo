#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

class MainScript:
    def __init__(self):
        self.laser_data_sub = rospy.Subscriber('/laser_data', Float32MultiArray, self.control_callback)
        self.pub = rospy.Publisher("/controls", String, queue_size=10)
        self.laser_data = None
        self.control_order = String()
        self.control = 'w'
        self.control_order.data = self.control

    def control_callback(self, data):
        self.laser_data = data

    def print_data(self):
        print(self.laser_data)


if __name__=="__main__":
    rospy.init_node("Main_script")   
    main_script = MainScript()
         
    rate = rospy.Rate(10) # message publish frequency
    while not rospy.is_shutdown():        
        print('Listening')
        main_script.print_data()    
        main_script.pub.publish(main_script.control_order)
        rate.sleep()
    rospy.spin()
    



        
