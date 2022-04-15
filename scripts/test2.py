#! /home/ubuntu/Env/py38/bin/python3.8
import time
import sys
import rospy
import argparse
from std_msgs.msg import Int32, Float32

# axis z

EMULATE_HX711=False
referenceUnit = 1

if not EMULATE_HX711:
    import RPi.GPIO as GPIO
    from hx711 import HX711
else:
    from emulated_hx711 import HX711

def cleanAndExit():
    print("Cleaning...")

    if not EMULATE_HX711:
        GPIO.cleanup()
        
    print("Bye!")
    sys.exit()

def main(args):
    rospy.init_node('laser_controller')
    rate=rospy.Rate(100)

    axis1_pub = rospy.Publisher('/axis_2', Float32, queue_size=1)
    axis1_msg = Float32()
    
    time_start = rospy.Time.now()

    hx = HX711(7, 8)
    hx.set_reading_format("MSB", "MSB")
    hx.set_reference_unit(referenceUnit)
    hx.reset()

    while not rospy.is_shutdown():
        val = hx.get_weight(10)
        axis1_msg.data = val
        rospy.loginfo('Load cell reading: %f'%val)
        axis1_pub.publish(axis1_msg)
        rate.sleep()

    cleanAndExit()
                
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run robot closeloop simulation for 2000 times')
    parser.add_argument('--timeout', default=40, type=float, help='total time of image streaming')
    args = parser.parse_args()
    
    main(args)