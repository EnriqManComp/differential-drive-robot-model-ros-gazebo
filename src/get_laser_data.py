#!/usr/bin/env python3

import rospy
import threading
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty

class GetLaserData(threading.Thread):
    def __init__(self):
        super(GetLaserData, self).__init__()        
        self.sub_node = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)        
        self.data = []
        self.condition = threading.Condition()        
        self.done = False

        self.start()

    def update(self, data):
        self.condition.acquire()

        self.data = list(data.ranges)
        print(self.data)

        # Notify publish thread that we have a new message
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True        
        self.join()
    
    def laser_callback(self, data):
        while not self.done:
            self.condition.acquire()
            self.update(data)
            self.condition.release()

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
    rospy.init_node('laser_subscriber')
    get_laser_data = GetLaserData()

    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    try:
        while(1):        
            key = getKey(key_timeout)
            if key == '\x03':
                break
            get_laser_data.update()
            
    except Exception as e:
        print(e)

    finally:
        get_laser_data.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



    
    


