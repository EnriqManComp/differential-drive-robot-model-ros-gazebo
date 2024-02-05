#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import threading

import roslib
import sys, select, termios, tty
msg= """
Reading from the keyboard  and Publishing to Twist!



"""

moveBindings = {
        "w": (1,0,0,0),
        "s": (-1,0,0,0),
        "a": (0,0,0,1),
        "d": (0,0,0,-1),        
    }



class Control(threading.Thread):
    def __init__(self, rate):
        super(Control, self).__init__()
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.done = False


        self.condition = threading.Condition()

        # Set timeout to None if rate is 0
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, th, speed, turn):
        self.condition.acquire()
        
        self.x = x        
        self.y = y        
        self.th = th
        self.speed = speed
        self.turn = turn
        
        # Notify publish thread that we have a new message
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout
            self.condition.wait(self.timeout)
            # Copy state into twist message
            twist.linear.x = (self.x * self.speed) 
            twist.linear.y = (self.y * self.speed) 
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = (self.th * self.turn) 

            

            self.condition.release()

            # Publish update
            self.publisher.publish(twist)

        # Publish stop message when thread exits
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("control_robot_node")    

    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 0.2)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = Control(repeat)
    
    x = 0
    y = 0    
    th = 0
    status = 0

    try:
        pub_thread.update(x, y, th, speed, turn)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][3]
            else:
                # Skip updating cmd_vel if key timeout and robot already stopped
                if key == '' and x == 0 and y == 0 and th == 0:
                    continue
                x = 0
                y = 0
                th = 0
                if key == '\x03':
                    break
            
            pub_thread.update(x, y, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

